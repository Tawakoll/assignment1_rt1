#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <cmath> // note to self don't need to add standard packages in cmake...
#define TURTLE1 1
#define TURTLE2 2
#define TOLERANCE 0.2
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
 
        //Distance between turtles! publisher and the subscriber is in the other node named Ui_node
        distance_publisher = this->create_publisher<std_msgs::msg::Float32>("distance_t1_t2", 10);

        turtle_subscriber = this->create_subscription<std_msgs::msg::UInt8>("turtle_choice", 10,std::bind(&DistanceController::topic_callback_turtleChoice, this, _1));
        velocity_subscriber = this->create_subscription<std_msgs::msg::Float32>("velocity", 10,std::bind(&DistanceController::topic_callback_turtleVelocity, this, _1));


        timer_ = this->create_wall_timer(std::chrono::milliseconds(20),std::bind(&DistanceController::timer_callback, this));
        }
        
    private:
     void timer_callback()
        {   


            // basically calculate the distance between turtles 1&2 , by square root of the difference between x 1,2 and y1,2 positions.
            dist_t1_t2 = std::sqrt(std::pow( (x_1-x_2) , 2 ) + std::pow( (y_1-y_2) , 2 )) ;
            distance_message.data = dist_t1_t2;
            distance_publisher-> publish(distance_message);

            //display the input from ui_node for debugging purposes
            RCLCPP_INFO(this->get_logger(), "The turtle choice is '%d' and velocity is '%f' ", turtle_choice, turtle_velocity);

            //uint8_t time_index = 0; 
            // i want to loop for a second then stop the turtle and 1sec = 1000ms, also timer loops every 20ms , so 1000/20 = 50 loops for 1 sec.
            // for(time_index=0 ; time_index <50 ; time_index++)
            // {
            //     switch(turtle_choice)
            //         {
            //         case TURTLE1: 
            //                 message.linear.x = 1.5;
            //                 message.angular.z = 0.0;

            //                 publisher_t1->publish(message);
            //         break;

            //         case TURTLE2:
            //                 message.linear.x = 1.5;
            //                 message.angular.z = 0.0;
            //                 publisher_t2->publish(message);    
            //         break;

            //         default:
            //         //print error message
            //             RCLCPP_INFO(this->get_logger(), "The input for turtle choice  is invalid ");
            //             break;
            //         }
            // }
           
                
        }
    void topic_callback(const turtlesim::msg::Pose::SharedPtr msg)
        {
                //RCLCPP_INFO(this->get_logger(), "The position of the turtle1 is (x,y): '%f','%f' ", msg->x,msg->y);
                x_1=msg->x;
                y_1=msg->y;
        }

     void topic_callback_t2(const turtlesim::msg::Pose::SharedPtr msg)
        {
                //RCLCPP_INFO(this->get_logger(), "The position of the turtle2 is (x,y): '%f','%f' ", msg->x,msg->y);
                x_2=msg->x;
                y_2=msg->y;

        }
       
     void topic_callback_turtleChoice(const std_msgs::msg::UInt8::SharedPtr msg)
        {
        //call back of the turtle_publisher in the node "ui_node"
           turtle_choice= msg->data;



        } 

        void topic_callback_turtleVelocity(const std_msgs::msg::Float32::SharedPtr msg)
        {
        //call back of the velocity_publisher in the node "ui_node"
            turtle_velocity= msg->data;
        }

            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_t1;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_t1;

            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_t2;            
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_t2;

            rclcpp::TimerBase::SharedPtr timer_; // Timer
            geometry_msgs::msg::Twist message;   //twist message object


            uint8_t turtle_choice ; // t_choice variable stores turtle choosen by the user , if == 1 , turtle1, else if == 2 turtle2, else invalid input. 
            
            float x_1 , x_2 , y_1, y_2, dist_t1_t2; // positions (x,y) for turtles 1&2 and dist_t1_t2 stores the distance between turtles 1 & 2.
            float turtle_velocity=0; // decalre turtle velocity to be taken as input by the user from the other node.
            std_msgs::msg::Float32 distance_message; //message container for distance to be published
           
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_publisher ;// declare disntace publisher in the class
            rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr turtle_subscriber;// declare turtle choice subscriber in the class to read from the other node.
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_subscriber;// declare velocity subscriber in the class to read from the other node.

};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceController>());
    rclcpp::shutdown();
    return 0;
}