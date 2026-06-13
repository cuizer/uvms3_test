#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "hal/msg/hal_dvl.hpp" 
#include <std_srvs/srv/set_bool.hpp> // 【新增】用于工作模式切换的标准服务
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include <string>
#include <sstream> 
#include <atomic>
#include <condition_variable> // 【新增】用于服务线程与接收线程同步

// 引入 Linux 底层串口所需的系统头文件
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sys/select.h> 

int setup_native_uart(const std::string& port_name, speed_t baud_rate) {
    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return -1;

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);
    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);
    tty.c_cflag |= CREAD | CLOCAL; 
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIFLUSH);
    fcntl(fd, F_SETFL, O_NONBLOCK);

    return fd;
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

class HalDvlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    HalDvlNode(const std::string & node_name)
    : rclcpp_lifecycle::LifecycleNode(node_name)
    {
        this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        cached_msg_.connection_status = 0;
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        dvl_pub_ = this->create_publisher<hal::msg::HalDvl>("/hal/dvl", 10);
        publish_timer_ = this->create_wall_timer(
            20ms, std::bind(&HalDvlNode::publish_timer_callback, this));
            
        // 【新增 1】注册 ROS 2 工作模式服务
        mode_service_ = this->create_service<std_srvs::srv::SetBool>(
            "~/set_work_mode",
            std::bind(&HalDvlNode::set_work_mode_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
        dvl_pub_->on_activate();
        is_running_ = true;
        dvl_thread_ = std::thread(&HalDvlNode::dvl_thread_function, this);
        return LifecycleNode::on_activate(state);
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {
        dvl_pub_->on_deactivate();
        is_running_ = false;
        if (dvl_thread_.joinable()) dvl_thread_.join();
        
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
        return LifecycleNode::on_deactivate(state);
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        dvl_pub_.reset();
        publish_timer_.reset();
        mode_service_.reset(); // 清理服务
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        is_running_ = false;
        if (dvl_thread_.joinable()) dvl_thread_.join();
        
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
        return CallbackReturn::SUCCESS;
    }

private:
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalDvl>> dvl_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;

    hal::msg::HalDvl cached_msg_;
    std::mutex msg_mutex_;

    int serial_fd_ = -1;
    std::thread dvl_thread_;
    std::atomic<bool> is_running_{false};
    std::atomic<int64_t> last_valid_data_ns_{0};

    // 【新增 2】指令同步状态机与条件变量
    enum class CmdStatus { IDLE, WAITING, SUCCESS, FAILED };
    CmdStatus cmd_status_ = CmdStatus::IDLE;
    std::mutex cmd_mutex_;
    std::condition_variable cmd_cv_;

    // 【新增 3】处理来自上层的指令请求
    void set_work_mode_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (serial_fd_ < 0) {
            response->success = false;
            response->message = "底层串口未就绪";
            return;
        }

        // 以控制声学启用(工作/休眠)为例，按照手册 wcs 的格式下发
        // true = y(声学工作)，false = n(声学关闭休眠)，需确保以 \n 结尾
        std::string cmd = request->data ? "wcs,1500,,y,n\n" : "wcs,1500,,n,n\n";

        std::unique_lock<std::mutex> lock(cmd_mutex_);
        cmd_status_ = CmdStatus::WAITING;
        
        ssize_t bytes_written = write(serial_fd_, cmd.c_str(), cmd.length());
        if (bytes_written < 0) {
            cmd_status_ = CmdStatus::IDLE;
            response->success = false;
            response->message = "下发指令到串口失败";
            return;
        }

        // 阻塞当前服务回调等待底层的回执唤醒，最大等待 2.0 秒防止死锁
        bool signaled = cmd_cv_.wait_for(lock, 2s, [this]{ return cmd_status_ != CmdStatus::WAITING; });

        if (!signaled) {
            cmd_status_ = CmdStatus::IDLE;
            response->success = false;
            response->message = "等待 DVL 配置回执超时";
            RCLCPP_WARN(this->get_logger(), "DVL 模式切换指令响应超时");
        } else {
            response->success = (cmd_status_ == CmdStatus::SUCCESS);
            response->message = response->success ? "配置成功 (收到 ACK)" : "配置失败或被拒绝 (收到 NACK)";
            RCLCPP_INFO(this->get_logger(), "DVL 服务调用结果: %s", response->message.c_str());
            cmd_status_ = CmdStatus::IDLE; // 重置状态，迎接下次请求
        }
    }

    void publish_timer_callback() {
        if (dvl_pub_->is_activated()) {
            hal::msg::HalDvl msg_to_publish;
            {
                std::lock_guard<std::mutex> lock(msg_mutex_);
                msg_to_publish = cached_msg_;
            }

            int64_t now_ns = this->now().nanoseconds();
            int64_t last_ns = last_valid_data_ns_.load();
            if (last_ns == 0 || (now_ns - last_ns) > 2000000000LL) {
                msg_to_publish.connection_status = 0;
            }

            dvl_pub_->publish(msg_to_publish);
        }
    }

    void parse_and_cache(const std::string& data, int64_t capture_time_ns) {
        size_t start_idx = data.find("wrx");
        if (start_idx == std::string::npos) {
            start_idx = data.find("wrz");
        }
        if (start_idx == std::string::npos) return;

        std::string clean_data = data.substr(start_idx);
        size_t star_pos = clean_data.find('*');
        std::string data_body = (star_pos != std::string::npos) ? clean_data.substr(0, star_pos) : clean_data;
        
        std::vector<std::string> tokens;
        std::stringstream ss(data_body);
        std::string token;
        
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        if (tokens.empty()) return;

        bool is_valid = false;
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;

        try {
            if (tokens[0] == "wrx" && tokens.size() >= 8) {
                std::string valid_str = tokens[7];
                valid_str.erase(valid_str.find_last_not_of(" \n\r\t") + 1); 
                if (valid_str == "y" || valid_str == "Y") {
                    is_valid = true;
                    vx = std::stof(tokens[2]);
                    vy = std::stof(tokens[3]);
                    vz = std::stof(tokens[4]);
                }
            } 
            else if (tokens[0] == "wrz" && tokens.size() >= 5) {
                std::string valid_str = tokens[4];
                valid_str.erase(valid_str.find_last_not_of(" \n\r\t") + 1);
                if (valid_str == "y" || valid_str == "Y") {
                    is_valid = true;
                    vx = std::stof(tokens[1]);
                    vy = std::stof(tokens[2]);
                    vz = std::stof(tokens[3]);
                }
            } else {
                return; 
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(this->get_logger(), "DVL 数据解析转换异常: %s", e.what());
            return;
        }

        std::lock_guard<std::mutex> lock(msg_mutex_);
        cached_msg_.timestamp = capture_time_ns;
        cached_msg_.connection_status = 1;
        last_valid_data_ns_.store(capture_time_ns);

        if (is_valid) {
            cached_msg_.velocity_x = vx;
            cached_msg_.velocity_y = vy;
            cached_msg_.velocity_z = vz;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "DVL 底面失锁！已强制速度归零以保护系统");
            cached_msg_.velocity_x = 0.0f;
            cached_msg_.velocity_y = 0.0f;
            cached_msg_.velocity_z = 0.0f;
        }
    }

    void dvl_thread_function() {
        std::string buffer = "";
        char read_buf[512]; 

        while (rclcpp::ok() && is_running_) {
            try {
                if (serial_fd_ < 0) {
                    std::string port = this->get_parameter("port_name").as_string();
                    int baud_int = this->get_parameter("baud_rate").as_int();
                    speed_t baud_rate = (baud_int == 115200) ? B115200 : 
                                        (baud_int == 460800) ? B460800 : B9600; 

                    serial_fd_ = setup_native_uart(port, baud_rate);
                    if (serial_fd_ < 0) {
                        std::this_thread::sleep_for(1s);
                        continue;
                    }
                }

                fd_set read_fds;
                FD_ZERO(&read_fds);
                FD_SET(serial_fd_, &read_fds);

                struct timeval tv;
                tv.tv_sec = 0;
                tv.tv_usec = 5000; 

                int ret = select(serial_fd_ + 1, &read_fds, NULL, NULL, &tv);

                if (ret < 0) {
                    throw std::runtime_error("select 监听底层错误");
                } 
                else if (ret > 0 && FD_ISSET(serial_fd_, &read_fds)) {
                    int bytes_read = read(serial_fd_, read_buf, sizeof(read_buf));
                    
                    if (bytes_read < 0) {
                        if (errno != EAGAIN && errno != EWOULDBLOCK) {
                            throw std::runtime_error("原生 read 失败");
                        }
                    } 
                    else if (bytes_read == 0) {
                         throw std::runtime_error("检测到虚拟串口 EOF (对端未连接或已断开)");
                    }
                    else {
                        int64_t capture_time_ns = this->now().nanoseconds();
                        buffer.append(read_buf, bytes_read);
                        size_t pos = 0;
                        size_t processed_pos = 0;
                        
                        while ((pos = buffer.find('\n', processed_pos)) != std::string::npos) {
                            std::string line = buffer.substr(processed_pos, pos - processed_pos);
                            processed_pos = pos + 1; 

                            if (!line.empty() && line.back() == '\r') {
                                line.pop_back();
                            }
                            
                            // 【新增 4】拦截 DVL 的响应包并唤醒等待的服务线程
                            // 假设协议中 wra 代表 ACK(成功)，wrn 代表 NACK(失败/无效请求)
                            if (line.find("wra") != std::string::npos) {
                                std::lock_guard<std::mutex> lock(cmd_mutex_);
                                if (cmd_status_ == CmdStatus::WAITING) {
                                    cmd_status_ = CmdStatus::SUCCESS;
                                    cmd_cv_.notify_all(); // 通知服务回调：成功！
                                }
                                continue; // 这是控制协议，不再向下走速度解析
                            } 
                            else if (line.find("wrn") != std::string::npos) {
                                std::lock_guard<std::mutex> lock(cmd_mutex_);
                                if (cmd_status_ == CmdStatus::WAITING) {
                                    cmd_status_ = CmdStatus::FAILED;
                                    cmd_cv_.notify_all(); // 通知服务回调：失败！
                                }
                                continue; 
                            }
                            
                            // 常规的速度报文向下交付解析
                            if (line.find("wrx") != std::string::npos || line.find("wrz") != std::string::npos) {
                                parse_and_cache(line, capture_time_ns); 
                            }
                        }
                        
                        if (processed_pos > 0) {
                            buffer.erase(0, processed_pos);
                        }
                        if (buffer.size() > 4096) buffer.clear(); 
                    }
                }
            } 
            catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "DVL 串口异常: %s", e.what());
                if (serial_fd_ >= 0) {
                    close(serial_fd_);
                    serial_fd_ = -1; 
                }
                std::this_thread::sleep_for(2s); 
            }
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HalDvlNode>("hal_dvl_node")->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}