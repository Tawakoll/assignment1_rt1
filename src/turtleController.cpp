#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
class TurtleSimController: public rclcpp::Node
{
public:
TurtleSimController(): Node("turtlesim_controller")
{
subscription_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,std::bind(&TurtleSimController::topic_callback, this, _1));
publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
timer_ = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&TurtleSimController::timer_callback, this));
}
/*Exercise 1:
create a new node that:
• Let the turtle move along x, until it reaches the end (x > 9.0)
• When x > 9.0 or x < 2.0, make it turn in a circular arc, until x<9.0 or x>2.0, then let the robot move again
only with a linear velocity
• Use an opposite angular velocity for the two cases
• Continue until the turtle reaches the y max or min value */
private:
void timer_callback()
{   if(x_ <= 9 && x_>=2.0)
   {
     message.linear.x =2.0;
    message.angular.z=0.0;

    }

    else if(x_>9)
    {
        message.linear.x =2.0;
        message.angular.z=2.0;

    }
    else if(x_<2.0)
     {

        message.linear.x =2.0;
        message.angular.z=2.0;

    }
    else
    {
         message.linear.x =0.0;
        message.angular.z=0.0;


    }
    
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
    geometry_msgs::msg::Twist message;   // <-- Add message object
    float x_;

};
int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<TurtleSimController>());
rclcpp::shutdown();
return 0;
}