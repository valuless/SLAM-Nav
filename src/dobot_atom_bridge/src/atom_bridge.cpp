/**
 * @file atom_bridge.cpp
 * @brief Dobot Atom ROS2 桥接节点 — 使用官方 dobot_atom_sdk
 *
 * 将机器人的 RPC 接口（FSM、速度、上肢控制）桥接到 ROS2 话题/服务。
 * 支持交互式 CLI 和 ROS2 launch 两种使用方式。
 */

#include <csignal>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "dobot_atom_bridge/atom_bridge_node.hpp"

static std::atomic<bool> g_running{true};

static void signalHandler(int /*sig*/)
{
  g_running = false;
  rclcpp::shutdown();
}

static void printUsage()
{
  std::cout << "\n"
            << "========================================\n"
            << "  Dobot Atom Bridge - CLI\n"
            << "========================================\n"
            << "  fsm <id>        设置 FSM 状态\n"
            << "  vel <x> <y> <z> 设置速度 (x,y 线速度, z 角速度)\n"
            << "  upper <0|1>     开关上肢控制\n"
            << "  status          查看连接状态\n"
            << "  quit            退出\n"
            << "========================================\n\n";
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 解析命令行参数
  std::string node_name = "atom_bridge_node";
  std::string rpc_ip = "192.168.8.234";
  int rpc_port = 51234;

  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--rpc-ip" && i + 1 < argc) {
      rpc_ip = argv[++i];
    } else if (arg == "--rpc-port" && i + 1 < argc) {
      rpc_port = std::stoi(argv[++i]);
    }
    // ROS2 标准方式: 用 --ros-args -r __node:=<name> 或 launch 中 name 属性改节点名
    // --rpc-ip 和 --rpc-port 不属于 ROS2 参数，自行解析
  }

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // 创建并初始化节点
  auto node = std::make_shared<dobot_atom_bridge::AtomBridgeNode>(
    node_name, rpc_ip, rpc_port);

  if (!node->initialize()) {
    std::cerr << "Failed to initialize AtomBridgeNode" << std::endl;
    return 1;
  }

  // 启动 ROS2 spinner 线程
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  auto spin_thread = std::thread([&executor]() { executor->spin(); });

  // 等待 RPC 连接建立
  std::cout << "Connecting to RPC server at " << rpc_ip << ":" << rpc_port
            << "..." << std::endl;

  for (int i = 0; i < 10 && g_running && !node->isConnected(); ++i) {
    std::cout << "Waiting for connection... (" << (i + 1) << "/10)" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  if (!node->isConnected()) {
    std::cout << "Warning: unable to establish RPC connection, "
              << "some features may not work" << std::endl;
  } else {
    uint32_t fsm_id = 0;
    if (node->getFsmId(fsm_id) == 0) {
      std::cout << "Connected. Current FSM ID: " << fsm_id << std::endl;
    }
  }

  // 交互式 CLI
  if (isatty(STDIN_FILENO)) {
    printUsage();

    std::string line;
    while (g_running && std::getline(std::cin, line)) {
      if (line.empty()) continue;

      std::istringstream iss(line);
      std::string cmd;
      iss >> cmd;

      if (cmd == "quit" || cmd == "exit" || cmd == "q") {
        break;
      }
      if (cmd == "fsm") {
        int id = 0;
        if (iss >> id) {
          int ret = node->setFsmId(static_cast<uint32_t>(id));
          if (ret != 0) {
            std::cout << "SetFsmId failed, error: " << ret << std::endl;
          }
        }
      } else if (cmd == "vel") {
        double x = 0, y = 0, z = 0;
        if (iss >> x >> y >> z) {
          int ret = node->setVel(static_cast<float>(x),
                                 static_cast<float>(y),
                                 static_cast<float>(z));
          if (ret != 0) {
            std::cout << "SetVel failed, error: " << ret << std::endl;
          }
        }
      } else if (cmd == "upper") {
        int val = 0;
        if (iss >> val) {
          int ret = node->switchUpperLimb(val != 0);
          if (ret != 0) {
            std::cout << "SwitchUpperLimb failed, error: " << ret << std::endl;
          }
        }
      } else if (cmd == "status") {
        uint32_t fsm_id = 0;
        (void)node->getFsmId(fsm_id);
        std::cout << "Node:   " << node->get_name() << std::endl;
        std::cout << "RPC:    " << rpc_ip << ":" << rpc_port;
        if (node->isConnected()) {
          std::cout << " (connected, FSM=" << fsm_id << ")";
        } else {
          std::cout << " (disconnected)";
        }
        std::cout << std::endl;
      } else {
        std::cout << "Unknown command: " << cmd << std::endl;
        printUsage();
      }
    }
  } else {
    // 非终端模式 — 等待信号退出
    while (g_running) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
