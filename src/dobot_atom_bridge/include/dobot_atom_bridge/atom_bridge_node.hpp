#ifndef DOBOT_ATOM_BRIDGE__ATOM_BRIDGE_NODE_HPP_
#define DOBOT_ATOM_BRIDGE__ATOM_BRIDGE_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rpc/algs_rpc_client.h"

namespace dobot_atom_bridge
{

class AtomBridgeNode : public rclcpp::Node
{
public:
  AtomBridgeNode(const std::string & node_name,
                 const std::string & rpc_ip,
                 int rpc_port);
  ~AtomBridgeNode();

  bool initialize();

  // ============================================================
  // 公开方法（供 CLI 直接调用）
  // ============================================================
  int setFsmId(uint32_t fsm_id);
  int setVel(float vx, float vy, float vyaw);
  int switchUpperLimb(bool enable);
  int getFsmId(uint32_t & fsm_id);
  bool isConnected() const { return rpc_connected_.load(); }

private:
  // ============================================================
  // RPC 命令回调（ROS2 → RPC）
  // ============================================================
  void fsmCmdCallback(const std_msgs::msg::UInt32::SharedPtr msg);
  void velCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void upperLimbCmdCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // ============================================================
  // 服务回调
  // ============================================================
  void upperLimbServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // ============================================================
  // 状态发布
  // ============================================================
  void statusTimerCallback();

  // ============================================================
  // RPC 桥接（连接维护）
  // ============================================================
  bool connectToRpcServer();
  void rpcBridgeLoop();

  // ============================================================
  // ROS2 接口
  // ============================================================
  // 订阅者
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr fsm_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr upper_limb_cmd_sub_;

  // 发布者
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr fsm_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connection_state_pub_;

  // 服务
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr upper_limb_service_;

  // 定时器
  rclcpp::TimerBase::SharedPtr status_timer_;

  // ============================================================
  // 状态
  // ============================================================
  std::unique_ptr<Atom::AlgsRpcClient> rpc_client_;
  std::string rpc_ip_;
  int rpc_port_;
  std::atomic<bool> rpc_connected_{false};
  std::atomic<uint32_t> current_fsm_id_{0};
  std::atomic<bool> initialized_{false};

  // 线程
  std::thread rpc_bridge_thread_;
  std::mutex rpc_mutex_;
};

}  // namespace dobot_atom_bridge

#endif  // DOBOT_ATOM_BRIDGE__ATOM_BRIDGE_NODE_HPP_
