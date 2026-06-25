/**
 * @file keyboard_control.cpp
 * @brief 键盘控制机器人运动
 * @details 使用 WASD 控制移动，左右箭头控制旋转
 */

#include "rpc/algs_rpc_client.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

// 运动参数
#define CONTROL_FREQUENCY 50      // 控制频率 50Hz
#define DEFAULT_DURATION 0.5      // 每个指令持续时间（秒）

// 速度预设值
const float SPEED_LINEAR = 0.3f;   // 线速度 m/s
const float SPEED_ANGULAR = 0.5f;  // 角速度 rad/s

class KeyboardController
{
public:
    KeyboardController(const std::string& ip = "192.168.8.234", int port = 51234)
        : running_(true), vx_(0.0f), vy_(0.0f), vyaw_(0.0f)
    {
        rpc_client_ = std::make_shared<Atom::AlgsRpcClient>(ip, port);
        set_terminal_nonblock();
        print_help();
    }
    
    ~KeyboardController()
    {
        running_ = false;
        if (control_thread_.joinable())
            control_thread_.join();
        if (keyboard_thread_.joinable())
            keyboard_thread_.join();
        rpc_client_->SetVel(0.0f, 0.0f, 0.0f, 0.1f);
        restore_terminal();
    }
    
    void run()
    {
        control_thread_ = std::thread(&KeyboardController::control_loop, this);
        keyboard_thread_ = std::thread(&KeyboardController::keyboard_loop, this);
        
        // 等待退出信号
        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    void stop()
    {
        running_ = false;
    }
    
private:
    void print_help()
    {
        std::cout << "\n========== 键盘控制已启动 ==========" << std::endl;
        std::cout << "W/S: 前后移动" << std::endl;
        std::cout << "A/D: 左右移动" << std::endl;
        std::cout << "←/→: 左右旋转" << std::endl;
        std::cout << "空格/Q: 紧急停止" << std::endl;
        std::cout << "ESC: 退出程序" << std::endl;
        std::cout << "===================================" << std::endl;
    }
    
    void set_terminal_nonblock()
    {
        tcgetattr(STDIN_FILENO, &old_tio_);
        new_tio_ = old_tio_;
        new_tio_.c_lflag &= ~(ICANON | ECHO);
        new_tio_.c_cc[VMIN] = 0;
        new_tio_.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);
        
        old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK);
    }
    
    void restore_terminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
        fcntl(STDIN_FILENO, F_SETFL, old_flags_);
    }
    
    void control_loop()
    {
        while (running_) {
            auto ret = rpc_client_->SetVel(vx_, vy_, vyaw_, DEFAULT_DURATION);
            if (ret != Atom::RpcErrorCode::SUCCESS) {
                static int error_count = 0;
                if (error_count++ % 50 == 0) {
                    std::cerr << "RPC 调用失败: " << static_cast<int>(ret) << std::endl;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / CONTROL_FREQUENCY));
        }
    }
    
    // 读取转义序列（方向键）
    bool read_escape_sequence(int& key)
    {
        char c;
        if (read(STDIN_FILENO, &c, 1) != 1) return false;
        
        if (c == 27) {  // ESC
            // 检查是否是方向键
            if (read(STDIN_FILENO, &c, 1) != 1) {
                // 单次 ESC 键，退出程序
                key = 27;
                return true;
            }
            if (c != '[') return false;
            if (read(STDIN_FILENO, &c, 1) != 1) return false;
            key = c;
            return true;
        }
        
        key = c;
        return true;
    }
    
    void keyboard_loop()
    {
        int key;
        while (running_) {
            if (read_escape_sequence(key)) {
                if (key == 27) {  // ESC 键退出
                    std::cout << "\n正在退出..." << std::endl;
                    running_ = false;
                    break;
                }
                handle_key(key);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    void handle_key(int key)
    {
        switch (key) {
            // 前后移动
            case 'w':
            case 'W':
                vx_ = SPEED_LINEAR;
                vy_ = 0.0f;
                vyaw_ = 0.0f;
                std::cout << "向前移动 " << vx_ << " m/s" << std::endl;
                break;
            case 's':
            case 'S':
                vx_ = -SPEED_LINEAR;
                vy_ = 0.0f;
                vyaw_ = 0.0f;
                std::cout << "向后移动 " << -vx_ << " m/s" << std::endl;
                break;
            // 左右移动
            case 'a':
            case 'A':
                vx_ = 0.0f;
                vy_ = SPEED_LINEAR;
                vyaw_ = 0.0f;
                std::cout << "向左移动 " << vy_ << " m/s" << std::endl;
                break;
            case 'd':
            case 'D':
                vx_ = 0.0f;
                vy_ = -SPEED_LINEAR;
                vyaw_ = 0.0f;
                std::cout << "向右移动 " << -vy_ << " m/s" << std::endl;
                break;
            // 左右箭头旋转
            case '4':  // ← 左箭头
                vx_ = 0.0f;
                vy_ = 0.0f;
                vyaw_ = SPEED_ANGULAR;
                std::cout << "向左旋转 " << vyaw_ << " rad/s" << std::endl;
                break;
            case '6':  // → 右箭头
                vx_ = 0.0f;
                vy_ = 0.0f;
                vyaw_ = -SPEED_ANGULAR;
                std::cout << "向右旋转 " << -vyaw_ << " rad/s" << std::endl;
                break;
            // 停止
            case ' ':
            case 'q':
            case 'Q':
                vx_ = 0.0f;
                vy_ = 0.0f;
                vyaw_ = 0.0f;
                std::cout << "停止" << std::endl;
                break;
            default:
                break;
        }
    }
    
private:
    std::shared_ptr<Atom::AlgsRpcClient> rpc_client_;
    std::thread control_thread_;
    std::thread keyboard_thread_;
    std::atomic<bool> running_;
    
    std::atomic<float> vx_;
    std::atomic<float> vy_;
    std::atomic<float> vyaw_;
    
    termios old_tio_, new_tio_;
    int old_flags_;
};

std::atomic<bool> g_running(true);

void signal_handler(int sig)
{
    if (sig == SIGINT || sig == SIGTERM) {
        g_running = false;
        std::cout << "\n正在退出..." << std::endl;
    }
}

int main(int argc, char** argv)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::string ip = "192.168.8.234";
    int port = 51234;
    
    if (argc > 1) ip = argv[1];
    if (argc > 2) port = std::stoi(argv[2]);
    
    std::cout << "连接目标: " << ip << ":" << port << std::endl;
    
    KeyboardController controller(ip, port);
    
    // 设置信号处理来停止控制器
    std::thread([&]() {
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        controller.stop();
    }).detach();
    
    controller.run();
    
    return 0;
}