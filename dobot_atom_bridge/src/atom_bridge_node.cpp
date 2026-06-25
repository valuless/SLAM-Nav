#include "dobot_atom_bridge/atom_bridge_node.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace dobot_atom_bridge
{

AtomBridgeNode::AtomBridgeNode(
  const std::string & node_name,
  const std::string & rpc_ip,
  int rpc_port)
: Node(node_name),
  rpc_ip_(rpc_ip),
  rpc_port_(rpc_port)
{
}

AtomBridgeNode::~AtomBridgeNode()
{
  initialized_ = false;
  rpc_connected_ = false;

  if (rpc_bridge_thread_.joinable()) {
    rpc_bridge_thread_.join();
  }
}

bool AtomBridgeNode::initialize()
{
  if (initialized_) {
    return true;
  }

  try {
    // 创建 RPC 客户端
    rpc_client_ = std::make_unique<Atom::AlgsRpcClient>(rpc_ip_, rpc_port_);

    // 订阅者 — 接收外部控制命令
    fsm_cmd_sub_ = create_subscription<std_msgs::msg::UInt32>(
      "~/fsm_cmd", 10,
      std::bind(&AtomBridgeNode::fsmCmdCallback, this, std::placeholders::_1));

    vel_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", 10,
      std::bind(&AtomBridgeNode::velCmdCallback, this, std::placeholders::_1));

    upper_limb_cmd_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/upper_limb_cmd", 10,
      std::bind(&AtomBridgeNode::upperLimbCmdCallback, this, std::placeholders::_1));

    // 发布者 — 对外发布机器人状态
    fsm_state_pub_ = create_publisher<std_msgs::msg::UInt32>("~/fsm_state", 10);
    connection_state_pub_ = create_publisher<std_msgs::msg::Bool>("~/connection_state", 10);

    // 服务
    upper_limb_service_ = create_service<std_srvs::srv::SetBool>(
      "~/upper_limb_service",
      std::bind(&AtomBridgeNode::upperLimbServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    // 状态查询定时器（10Hz）
    status_timer_ = create_wall_timer(
      100ms,
      std::bind(&AtomBridgeNode::statusTimerCallback, this));

    // 尝试连接 RPC 服务器
    if (connectToRpcServer()) {
      rpc_bridge_thread_ = std::thread(&AtomBridgeNode::rpcBridgeLoop, this);
      RCLCPP_INFO(get_logger(), "Connected to RPC server at %s:%d",
                  rpc_ip_.c_str(), rpc_port_);
    } else {
      RCLCPP_WARN(get_logger(),
                  "Failed to connect to RPC server at %s:%d, running without RPC",
                  rpc_ip_.c_str(), rpc_port_);
    }

    initialized_ = true;
    RCLCPP_INFO(get_logger(), "AtomBridgeNode initialized");

    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize: %s", e.what());
    return false;
  }
}

// ============================================================
// 公开方法（供 CLI 和外部调用）
// ============================================================

int AtomBridgeNode::setFsmId(uint32_t fsm_id)
{
  std::lock_guard<std::mutex> lock(rpc_mutex_);
  if (!rpc_client_ || !rpc_connected_) {
    return -1;
  }
  auto result = rpc_client_->SetFsmId(static_cast<int32_t>(fsm_id));
  if (result == Atom::RpcErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "FSM set to %u", fsm_id);
    return 0;
  }
  RCLCPP_WARN(get_logger(), "SetFsmId failed: %d", static_cast<int>(result));
  return static_cast<int>(result);
}

int AtomBridgeNode::setVel(float vx, float vy, float vyaw)
{
  std::lock_guard<std::mutex> lock(rpc_mutex_);
  if (!rpc_client_ || !rpc_connected_) {
    return -1;
  }
  auto result = rpc_client_->SetVel(vx, vy, vyaw);
  if (result != Atom::RpcErrorCode::SUCCESS) {
    RCLCPP_WARN(get_logger(), "SetVel failed: %d", static_cast<int>(result));
    return static_cast<int>(result);
  }
  return 0;
}

int AtomBridgeNode::switchUpperLimb(bool enable)
{
  std::lock_guard<std::mutex> lock(rpc_mutex_);
  if (!rpc_client_ || !rpc_connected_) {
    return -1;
  }
  auto result = rpc_client_->SwitchUpperLimbControl(enable);
  if (result == Atom::RpcErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "Upper limb %s", enable ? "enabled" : "disabled");
    return 0;
  }
  RCLCPP_WARN(get_logger(), "SwitchUpperLimbControl failed: %d",
              static_cast<int>(result));
  return static_cast<int>(result);
}

int AtomBridgeNode::getFsmId(uint32_t & fsm_id)
{
  std::lock_guard<std::mutex> lock(rpc_mutex_);
  if (!rpc_client_ || !rpc_connected_) {
    return -1;
  }
  int32_t id = 0;
  auto result = rpc_client_->GetFsmId(id);
  if (result == Atom::RpcErrorCode::SUCCESS) {
    fsm_id = static_cast<uint32_t>(id);
    return 0;
  }
  return static_cast<int>(result);
}

// ============================================================
// RPC 命令回调
// ============================================================

void AtomBridgeNode::fsmCmdCallback(const std_msgs::msg::UInt32::SharedPtr msg)
{
  setFsmId(msg->data);
}

void AtomBridgeNode::velCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  setVel(
    static_cast<float>(msg->linear.x),
    static_cast<float>(msg->linear.y),
    static_cast<float>(msg->angular.z));
}

void AtomBridgeNode::upperLimbCmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  switchUpperLimb(msg->data);
}

// ============================================================
// 服务回调
// ============================================================

void AtomBridgeNode::upperLimbServiceCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::lock_guard<std::mutex> lock(rpc_mutex_);
  if (!rpc_client_ || !rpc_connected_) {
    response->success = false;
    response->message = "RPC client not connected";
    return;
  }

  auto result = rpc_client_->SwitchUpperLimbControl(request->data);
  response->success = (result == Atom::RpcErrorCode::SUCCESS);
  response->message = response->success
    ? "Upper limb switched successfully"
    : "Failed to switch upper limb, error code: " + std::to_string(static_cast<int>(result));
}

// ============================================================
// 状态发布
// ============================================================

void AtomBridgeNode::statusTimerCallback()
{
  auto connection_msg = std_msgs::msg::Bool();
  connection_msg.data = rpc_connected_.load();
  connection_state_pub_->publish(connection_msg);

  // 查询当前 FSM 状态
  std::lock_guard<std::mutex> lock(rpc_mutex_);
  if (rpc_client_ && rpc_connected_) {
    int32_t fsm_id = 0;
    auto result = rpc_client_->GetFsmId(fsm_id);
    if (result == Atom::RpcErrorCode::SUCCESS) {
      current_fsm_id_ = static_cast<uint32_t>(fsm_id);
      auto fsm_msg = std_msgs::msg::UInt32();
      fsm_msg.data = static_cast<uint32_t>(fsm_id);
      fsm_state_pub_->publish(fsm_msg);
    } else {
      RCLCPP_DEBUG(get_logger(), "GetFsmId failed in status timer, error: %d",
                   static_cast<int>(result));
    }
  }
}

// ============================================================
// RPC 连接管理
// ============================================================

bool AtomBridgeNode::connectToRpcServer()
{
  if (!rpc_client_) {
    return false;
  }

  // 通过 GetFsmId 测试连接
  int32_t test_fsm_id = 0;
  auto result = rpc_client_->GetFsmId(test_fsm_id);

  if (result == Atom::RpcErrorCode::SUCCESS) {
    rpc_connected_ = true;
    current_fsm_id_ = static_cast<uint32_t>(test_fsm_id);
    RCLCPP_INFO(get_logger(), "RPC connection established, current FSM: %d", test_fsm_id);
    return true;
  }

  rpc_connected_ = false;
  RCLCPP_WARN(get_logger(), "RPC connection test failed, error code: %d",
              static_cast<int>(result));
  return false;
}

void AtomBridgeNode::rpcBridgeLoop()
{
  RCLCPP_INFO(get_logger(), "RPC bridge loop started");

  while (rclcpp::ok() && initialized_) {
    if (!rpc_client_) {
      std::this_thread::sleep_for(1s);
      continue;
    }

    // 检查连接
    if (rpc_connected_) {
      int32_t fsm_id = 0;
      auto result = rpc_client_->GetFsmId(fsm_id);

      if (result != Atom::RpcErrorCode::SUCCESS) {
        RCLCPP_WARN(get_logger(),
                    "RPC connection lost (error: %d), attempting reconnect...",
                    static_cast<int>(result));
        rpc_connected_ = false;

        std::this_thread::sleep_for(1s);
        connectToRpcServer();
      }
    } else {
      // 尝试重新连接
      connectToRpcServer();
      if (!rpc_connected_) {
        std::this_thread::sleep_for(2s);
      }
    }

    std::this_thread::sleep_for(500ms);
  }

  RCLCPP_INFO(get_logger(), "RPC bridge loop stopped");
}

}  // namespace dobot_atom_bridge
