#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <vector>
#include <cmath>

using std::placeholders::_1;
float threshold = 0.6, thetha =0 ,epsilonA = 0.1, epsilonB = 0.1, linear_speed, angular_speed;
bool target_found=false ,moving=false;
Point target,endpnt,camera; 

struct Point{
    float x;
    float y ;
    float z;

};

void calculateendpnt(){
    float bufferdist= 0.6; //60 cm
    thetha = atan((target.x+camera.x)/(target.z+camera.z)); // Confirm transformation
    endpnt.x= (target.x+camera.x)- bufferdist*sin(theta);
    endpnt.z= (target.z+camera.z)- bufferdist*cos(theta);
}

void sendVelAndTwist(float linear_speed, float angular_speed) {

}


class CoordinateSubscriber : public rclcpp::Node
{
public:
    CoordinateSubscriber()
    : Node("coordinate_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "get_point", 10, std::bind(&CoordinateSubscriber::callback, this, _1));
    }

private:
    void callback(const geometry_msgs::msg::Point::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received coordinates: x=%.2f, y=%.2f, z=%.2f",
                    msg->x, msg->y, msg->z);
                    target.x = msg->x;
                    target.y = msg->y;
                    target.z = msg->z;

                    //target location is wrt camera


                    //below code is executed in sync with the timer in the reciever node .. So in a way the entire mp/controls module works(or should work)
                    // on this one common "clock pulse", BE CAREFUL if you are creating any other timer/while loop 


                    if(target.z!=0){ // found target
                        target_found=true;
                    }
                    else{ // search
                        sendVelAndTwist
                    }
                    if(target_found && !moving){
                        calculateendpnt();
                        if (abs(ndpnt.z - threshold) > epsilonB) {
                            moving = true;
                            while(target.x > epsilonA) {
                                sendVelAndTwist(0, angular_speed);
                            }
                            sendVelAndTwist(linear_speed, 0);
                        }
                    }
                    if (moving) {
                        if (abs(endpnt.z - threshold) <= epsilonB){
                            sendVelAndTwist(0,0);
                            moving = false;
                        }
                    }




    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
};






int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateSubscriber>());

    rclcpp::shutdown();
    return 0;
}
