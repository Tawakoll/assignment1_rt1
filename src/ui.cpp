#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
class UInode: public rclcpp::Node
{
public:
UInode(): Node("ui_node")
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&UInode::timer_callback, this));
}

    private:
    void timer_callback()
    {   
        
        publisher_->publish(message);

    }
    
    
    void topic_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
            RCLCPP_INFO(this->get_logger(), "The position of the turtle is (x,y): '%f','%f' ", msg->x,msg->y);
            x_=msg->x;
    }
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_; // <-- Add Timer

    usigned char turtle_choice = 0;

};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UInode>());
    rclcpp::shutdown();
    return 0;
}