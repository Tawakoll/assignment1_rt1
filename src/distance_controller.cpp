#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define turtle1 1
#define turtle2 2

using std::placeholders::_1;
class DistanceController: public rclcpp::Node
{
    public:
        DistanceController(): Node("distance_controller")
    {
        
        //turtle 1 publisher and subscriber
        subscription_t1 = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,std::bind(&DistanceController::topic_callback, this, _1));
        publisher_t1 = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        
        //turtle 2 publisher and subscriber
        subscription_t2 = this->create_subscription<turtlesim::msg::Pose>("turtle2/pose", 10,std::bind(&DistanceController::topic_callback_t2, this, _1));
        publisher_t2 = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
        

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&DistanceController::timer_callback, this));
        }
        
        private:
        void timer_callback()
        {   
            // message.linear.x =2.0;
            // message.angular.z=0.0;
            // switch(turtle_choice)
            // {
            //     case turtle1: 
            // // message.linear.x =2.0;
            // // message.angular.z=0.0;          
            //     break;

            // }
       
                message.linear.x = 0.5;
             message.angular.z = 0.0;
            publisher_t1->publish(message);
            publisher_t2->publish(message);



            

        }
        void topic_callback(const turtlesim::msg::Pose::SharedPtr msg)
        {
                RCLCPP_INFO(this->get_logger(), "The position of the turtle1 is (x,y): '%f','%f' ", msg->x,msg->y);
                x_1=msg->x;
                y_1=msg->y;
        }

        void topic_callback_t2(const turtlesim::msg::Pose::SharedPtr msg)
        {
                RCLCPP_INFO(this->get_logger(), "The position of the turtle1 is (x,y): '%f','%f' ", msg->x,msg->y);
                x_2=msg->x;
                y_2=msg->y;

        }
            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_t1;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_t1;

            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_t2;            
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_t2;

            rclcpp::TimerBase::SharedPtr timer_; // <-- Add Timer
            geometry_msgs::msg::Twist message;   // <-- Add message object
            float x_1 , x_2 , y_1, y_2, dist_t1_t2; // positions (x,y) for turtles 1&2 and dist_t1_t2 stores the distance between turtles 1 & 2.
            unsigned char turtle_choice ; // t_choice variable stores turtle choosen by the user , if == 1 , turtle1, else if == 2 turtle2, else invalid input. 

};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceController>());
    rclcpp::shutdown();
    return 0;
}