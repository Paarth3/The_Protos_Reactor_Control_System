#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <thread>
#include <atomic>

using namespace std::chrono_literals;

std::string string_topic = "/hyperion/telemetry";

class ReactorPhysics : public rclcpp::Node{
    public:
        ReactorPhysics() : Node("publisher_node"){
            pub_string = this->create_publisher<std_msgs::msg::String>(string_topic, 5);

            period = 500ms;
            timer = this->create_wall_timer(period, [this]() {this->callback_fun();});
        }

        void update(){
            int randomIncreaseInTemp = rand() % 19;
            temperature += randomIncreaseInTemp;
            pressure += 0.9 * randomIncreaseInTemp;
        }

        void callback_fun(){
            auto string_msg = std_msgs::msg::String(); 
            
            this->update();

            string_msg.data = "T:" + std::to_string(temperature) + "|P:" + std::to_string(pressure);
            std::string log_msg = "Publishing --> " + string_msg.data;

            pub_string->publish(string_msg);

            RCLCPP_INFO(this->get_logger(), log_msg.c_str());

        }

    private:
        float temperature = 0.0;
        float pressure = 0.0;
        
        std::shared_ptr<rclcpp::TimerBase> timer;
        std::chrono::milliseconds period;

        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> pub_string;

};

int main(int argc, char* argv[]){

    srand(time(NULL));

    rclcpp::init(argc, argv);

    auto myReactor = std::make_shared<ReactorPhysics>();

    rclcpp::spin(myReactor);

    rclcpp::shutdown();

    return 0;
}