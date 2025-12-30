#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <vector>
#include <string>

std::string string_topic = "/hyperion/telemetry";
std::vector<float> temp_history;
float curr_temp_avg = 0;

double calculateAverage(const std::vector<float>& data) {
    if (data.empty()) {
        return 0.0;
    }

    double sum = 0;
    for (int element : data) {
        sum += element;
    }

    return sum / data.size();
}

class SubscriberNode : public rclcpp::Node {
    public:
        SubscriberNode() : Node("subscriber_node") {
            sub_string = this->create_subscription<std_msgs::msg::String>(string_topic, 10, [this](const std::shared_ptr<std_msgs::msg::String> msg) {this->callback_fun(msg);});
        }

        void callback_fun(std::shared_ptr<std_msgs::msg::String> msg) {
            int pos = msg->data.find_first_of('|');
            std::string temp_part = msg->data.substr(0, pos);
            std::string pressure_part = msg->data.substr(pos + 1);

            float temperature = std::stof(temp_part.substr((temp_part.find_first_of(':')) + 1));
            float pressure = std::stof(pressure_part.substr((pressure_part.find_first_of(':')) + 1));

            if (temp_history.size() < 10) {
                temp_history.push_back(temperature);
            }
            else {
                temp_history.erase(temp_history.begin());
                temp_history.push_back(temperature);
            }

            float new_avg = calculateAverage(temp_history);

            if ((pressure > 1500) || (temperature > 2000)) {
                RCLCPP_ERROR(this->get_logger(), 
                    "\n\t************************************************"
                    "\n\t* MELTDOWN IMMINENT"
                    "\n\t************************************************"
                    "\n\t* Pressure > 1500 or Temperature > 2000"
                    "\n\t************************************************");
            }
            else if ((new_avg - curr_temp_avg) > 5) {
                RCLCPP_WARN(this->get_logger(), 
                    "\n\t>>> RAPID TEMPERATURE RISE DETECTED >>>"
                    "\n\t>>> Rate: +%.2f deg/cycle", new_avg - curr_temp_avg);
            }
            
            RCLCPP_INFO(this->get_logger(),
                "\n|------------------- REACTOR TELEMETRY -------------------|"
                "\n|  Temperature  : %8.2f C                             |"
                "\n|  Pressure     : %8.2f Pa                            |"
                "\n|  Avg Temp     : %8.2f C                             |"
                "\n|---------------------------------------------------------|", 
                temperature, pressure, new_avg);

            curr_temp_avg = new_avg;
        }

    private:
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_string;
};

int main(int argc, char* argv[]){

    rclcpp::init(argc, argv);

    auto mySubscriber = std::make_shared<SubscriberNode>();

    rclcpp::spin(mySubscriber);

    rclcpp::shutdown();

    return 0;
}