#include <ros/ros.h>
#include <navigation/Prop.h>

void angle_range_finder() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<navigation::Prop>("prop_angle_range", 1);
    ros::Rate rate(10);
    navigation::Prop msg;
    msg.prop_type = "red_marker";
    msg.theta_1 = 4;
    msg.theta_2 = 177;

    while (ros::ok()) {
        ROS_INFO_STREAM(msg);
        pub.publish(msg);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "angle_range_finder_node");
    try {
        angle_range_finder();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the angle_range_finder_node: " << e.what());
    }
    return 0;
}