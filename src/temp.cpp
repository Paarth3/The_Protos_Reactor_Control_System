#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <deque>

class MovingAverage {
private:
    std::deque<float> temp_history;
    const size_t max_size = 10;
    double running_sum = 0.0;

public:
    float update(float new_temp) {
        if (temp_history.size() >= max_size) {
            running_sum -= temp_history.front();
            temp_history.pop_front();
        }

        temp_history.push_back(new_temp);
        running_sum += new_temp;

        return static_cast<float>(running_sum / temp_history.size());
    }
    
    float getCurrentAverage() const {
        if (temp_history.empty()) return 0.0f;
        return static_cast<float>(running_sum / temp_history.size());
    }
};

class SubscriberNode : public rclcpp::Node {
    public:
        SubscriberNode() : Node("subscriber_node") {
            sub_string = this->create_subscription<std_msgs::msg::String>(
                "/hyperion/telemetry", 10, 
                [this](const std::shared_ptr<std_msgs::msg::String> msg) {this->callback_fun(msg);}
            );
        }

        void callback_fun(std::shared_ptr<std_msgs::msg::String> msg) {
            int pos = msg->data.find_first_of('|');
            std::string temp_part = msg->data.substr(0, pos);
            std::string pressure_part = msg->data.substr(pos + 1);

            float temperature = std::stof(temp_part.substr((temp_part.find_first_of(':')) + 1));
            float pressure = std::stof(pressure_part.substr((pressure_part.find_first_of(':')) + 1));
          
            float prev_avg = moving_avg.getCurrentAverage();
            
            float new_avg = moving_avg.update(temperature);

            if ((pressure > 1500) || (temperature > 2000)) {
                RCLCPP_ERROR(this->get_logger(), 
                    "\n\t************************************************"
                    "\n\t* MELTDOWN IMMINENT"
                    "\n\t************************************************");
            }
            else if ((new_avg - prev_avg) > 5) {
                RCLCPP_WARN(this->get_logger(), 
                    "\n\t>>> RAPID TEMPERATURE RISE DETECTED >>>"
                    "\n\t>>> Rate: +%.2f deg/cycle", new_avg - prev_avg);
            }
            
            RCLCPP_INFO(this->get_logger(),
                "\n|------------------- REACTOR TELEMETRY -------------------|"
                "\n|  Temperature  : %8.2f C                             |"
                "\n|  Pressure     : %8.2f Pa                            |"
                "\n|  Avg Temp     : %8.2f C                             |"
                "\n|---------------------------------------------------------|", 
                temperature, pressure, new_avg);
        }

    private:
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_string;
        
        MovingAverage moving_avg; 
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto mySubscriber = std::make_shared<SubscriberNode>();
    rclcpp::spin(mySubscriber);
    rclcpp::shutdown();
    return 0;
}
