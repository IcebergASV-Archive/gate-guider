#include <ros/ros.h>
#include <navigation/yolo.h>
#include <navigation/Prop.h>


class AngleFinder {
public:
    AngleFinder()
    {
        // Set up subscribers and publishers
        yolo_sub_ = nh_.subscribe("/yolo", 1, &AngleFinder::yoloCallback, this);
        prop_pub_ = nh_.advertise<navigation::Prop>("/prop_angles", 1);
        
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // Callback for the /global_position/global topic
    void yoloCallback(const navigation::yolo::ConstPtr& msg)
    {
        //get the position of the bounding box
        x_min = msg->xmin;
        x_max = msg->xmax;

        // Calculate the angle range for the prop
        float theta1 = fov_end - ((x_max / realsense_res_x) * realsense_fov);
        float theta2 = fov_end - ((x_min / realsense_res_x) * realsense_fov);
        
        // Create and publish the Prop message with the prop coordinates
        navigation::Prop prop_msg;
        prop_msg.prop_type = msg->label; //assign object classification label to the prop
        prop_msg.theta_1 = theta1;
        prop_msg.theta_2 = theta2;
        prop_pub_.publish(prop_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber yolo_sub_;
    ros::Publisher prop_pub_;
    float x_min;
    float x_max;
    float const realsense_fov = 1.204277184; //radians - 69 degrees
    float const fov_end = 1.570796327 + (realsense_fov / 2 );
    int const realsense_res_x = 1920;
    int const realsense_res_y = 1080;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "angle_finder");
    AngleFinder angle_finder;
    angle_finder.spin();
    return 0;
}