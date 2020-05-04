#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// Source: https://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/
class SubscribeAndPublish
{
private:
    ros::NodeHandle n_; 
    ros::Publisher pub_;
    ros::Subscriber sub_;
public:
    SubscribeAndPublish()
    {
        //Topic you want to publish
        pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        
        //Topic you want to subscribe
        sub_ = n_.subscribe("wanted_cmd_vel", 1000, &SubscribeAndPublish::callback, this);
    }
    
    void callback(const geometry_msgs::Twist& input)
    {
        pub_.publish(input);
    }
};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_given_vel");
    SubscribeAndPublish SandP;
    
    ros::Rate loop_rate(10);
    
    while(ros::ok())
    {
        ros::spin();
        loop_rate.sleep();
    }

    return 0;
}
