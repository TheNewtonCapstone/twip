#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Subscriber: public rclcpp::Node{
	private:
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

		void topic_callback(const std_msgs::msg::String &msg) const{
			RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());	
		}
	public:
		Subscriber(): Node("Subscriber"){
		subscription_ = this -> create_subscription<std_msgs::msg::String>(
				"topic", 10, std::bind(&Subscriber::topic_callback, this, _1));

		}
};

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Subscriber>());
	rclcpp::shutdown();
	return 0;
}
