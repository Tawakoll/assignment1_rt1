#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <cmath> // note to self don't need to add standard packages in cmake...
#define TURTLE1 1
#define TURTLE2 2
#define TOLERANCE 1.0f //tolerance distance between turtles to stop the moving turtle "using f for float at the end to explicitly type cast it."
#define Max_x_or_y 10.0f // maximum distance of x or y to the wall from the center to the right for x , y idk :D
#define Min_x_or_y 2.1f // minimum distance of x or y to the wall redundant same as tolerance but for clarity and readability , measured from center to left for x, y idk :D
#define TIME_LIMIT 50 // time limit to move the turtle in timer callback, 1000ms / 20ms = 50. We wanna move the turtle for 1 second only timeer callback is called every 20ms.
using std::placeholders::_1;

uint8_t time_counter= 255; // global variable to count the number of times the timer callback is called.

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
            RCLCPP_INFO(this->get_logger(), "The distance is '%f' ", dist_t1_t2);

            //display the input from ui_node for debugging purposes
           // RCLCPP_INFO(this->get_logger(), "The turtle choice is '%u' and velocity is '%f' ", turtle_choice, turtle_velocity);

            // My logic is basically if the distance between turtles is less than the tolerance value or if close to hit the wall , stop the moving turtle by publishing zero velocity 
            //else if (negation of the first condition) move the chosen turtle at the given velocity.
            // else print error message. "we should never reach this else statement "

            if ( dist_t1_t2 < TOLERANCE  
                || x_1 >Max_x_or_y  || x_1 < Min_x_or_y || y_1 >Max_x_or_y || y_1 < Min_x_or_y 
                || x_2 >Max_x_or_y  || x_2 < Min_x_or_y || y_2 >Max_x_or_y  || y_2 < Min_x_or_y 
                ||   time_counter >= TIME_LIMIT )
                {
                    //stop the moving turtle by publishing zero velocity
                    RCLCPP_INFO(this->get_logger(), "Stopping the moving turtle%u! ",turtle_choice);
                    message.linear.x = 0.0;
                    message.angular.z = 0.0;

                    switch(turtle_choice)
                    {
                    case TURTLE1: 
                            publisher_t1->publish(message);
                    break;

                    case TURTLE2:
                            publisher_t2->publish(message);    
                    break;

                    default:
                    //print error message
                        RCLCPP_INFO(this->get_logger(), "error in stopping the turtle , The input for turtle choice  is invalid ");
                        break;
                    }
                    
                }
    //moving condition is basically the negation of the stopping condition.
    else if ( dist_t1_t2 >= TOLERANCE  
                && x_1 <Max_x_or_y  && x_1 > Min_x_or_y && y_1 <Max_x_or_y  && y_1 > Min_x_or_y 
                && x_2 <Max_x_or_y  && x_2 > Min_x_or_y && y_2 <Max_x_or_y  && y_2 > Min_x_or_y 
                &&   time_counter < TIME_LIMIT )
                {
                    time_counter++; // increment 20ms for the time counter to contribute to the 1000ms time limit for moving the turtle
                     
                    // print for debugging purposes
                    RCLCPP_INFO(this->get_logger(), "Time counter =  %u ",time_counter);
                    message.linear.x = turtle_velocity;
                    message.angular.z = 0.0;
                    switch(turtle_choice)
                    {
                    case TURTLE1: 
                                                        
                            RCLCPP_INFO(this->get_logger(), "congrats you chose turtle 1! moving at velocity: '%f' ", turtle_velocity);


                            publisher_t1->publish(message);
                    break;

                    case TURTLE2:
                            RCLCPP_INFO(this->get_logger(), "congrats you chose turtle 2! moving at velocity: '%f' ", turtle_velocity);
                            publisher_t2->publish(message);    
                    break;

                    default:
                    //print error message
                        RCLCPP_INFO(this->get_logger(), "error in moving the turtle, The input for turtle choice  is invalid ");
                        break;
                    }
                    

                }
        else
                {
                    RCLCPP_INFO(this->get_logger(), "you reached an invalid state xd !");
                }  
           
                
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
           
            // reset the time counter each time a new velocity is received from the ui_node, i think it can also be placed in the turtle choice callback. 
            //I choose this callback because I take the velocity from the user after choosing the turtle to move.
             time_counter=0;
        }

            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_t1;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_t1;

            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_t2;            
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_t2;

            rclcpp::TimerBase::SharedPtr timer_; // Timer
            geometry_msgs::msg::Twist message;   //twist message object


            uint8_t turtle_choice =0; // t_choice variable stores turtle choosen by the user , if == 1 , turtle1, else if == 2 turtle2, else invalid input. 
            
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