#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"      // For publishing the turtle choice (1 or 2)
#include "std_msgs/msg/float32.hpp"   // For subscribing to the calculated distance
#include <iostream> //interaction with the user for input and displaying the distance

using std::placeholders::_1;
class UInode: public rclcpp::Node
{
    public:
    UInode(): Node("ui_node")
    {
        turtle_publisher = this->create_publisher<std_msgs::msg::UInt8>("turtle_choice", 10);
        velocity_publisher = this->create_publisher<std_msgs::msg::Float32>("velocity", 10);
        distance_subscriber = this->create_subscription<std_msgs::msg::Float32>("distance_t1_t2", 10,std::bind(&UInode::topic_callback_distance, this, _1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20),std::bind(&UInode::timer_callback, this)); // callback every one second as defined in the assignment
    }

        private:
        void timer_callback()
        {   
            std::cout<< "select your turtle! press 1 or 2.--- Current distance between turtles = "<< distance << std::endl;
            std::cin>> turtle_choice;
            std::cout<< "Enter the speed for your turtle'%d'"<< turtle_choice<< std::endl;
            std::cin>> turtle_velocity;

            velocity_msg.data = turtle_velocity;
            velocity_publisher-> publish(velocity_msg);

            turtle_msg.data = turtle_choice;
           turtle_publisher-> publish(turtle_msg);

        }
       
        void topic_callback_distance(const std_msgs::msg::Float32::SharedPtr msg)
        {
        //call back of the distance_publisher in the node distance_controller
           distance= msg->data;

        }        
        

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_subscriber;// declare disntace subscriber in the class to read from the other node.

        rclcpp::TimerBase::SharedPtr timer_; // timer

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_publisher; // declared the velocity publisher in this class & node to be read from the other node
        std_msgs::msg::Float32 velocity_msg; //message container for turtle choice to be published
        
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr turtle_publisher; // decalred the turtle choice publisher in this class & node to be read from the other node
        std_msgs::msg::UInt8 turtle_msg; //message container for turtle choice to be published
        float distance = 0, turtle_velocity = 0;
        unsigned char turtle_choice = 0;

};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UInode>());
    rclcpp::shutdown();
    return 0;
}