#include <chrono>
#include <functional>
#include <memory>
#include <string> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/*
 * Subclass of Node, increases a counter and publishes it on the topic 10
 */ 

class Publisher: public rclcpp::Node {
	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
		size_t counter_;

		void timer_callback(){
		  	auto message = std_msgs::msg::String();
			message.data = "Publisher initialized! " + std::to_string(counter_++);
			RCLCPP_INFO(this->get_logger(), "Publishing : '%s'", message.data.c_str());
			publisher_->publish(message);
		}

	public: 
		Publisher(): Node("publisher_"), counter_(0){

			publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
			timer_ = this->create_wall_timer(500ms, std::bind(&Publisher::timer_callback,this));

		}

};


int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Publisher>());
	rclcpp::shutdown();
	return 0;
}
