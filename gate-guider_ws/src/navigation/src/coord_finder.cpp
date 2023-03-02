#include <ros/ros.h>
//#include <sensor_msgs/NavSatFix.h> temporary
#include <navigation/Prop.h>
#include <navigation/SimpleGPS.h> //temporary
#include <geographic_msgs/GeoPoint.h>
#include <cmath> // added for M_PI

class PropLocator {
public:
    PropLocator()
    {
        // Set up subscribers and publishers
        gps_sub_ = nh_.subscribe("/rectbot_coords", 1, &PropLocator::gpsCallback, this);
        prop_sub_ = nh_.subscribe("/prop_closest_point", 1, &PropLocator::propCallback, this);
        prop_pub_ = nh_.advertise<navigation::Prop>("/prop_coordinates", 1);
        
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
    void gpsCallback(const navigation::SimpleGPS::ConstPtr& msg)
    {
        robot_lat_ = msg->latitude;
        robot_lon_ = msg->longitude;
        robot_alt_ = msg->altitude;
    }

    // Callback for the /prop_closest_point topic
    void propCallback(const navigation::Prop::ConstPtr& msg)
    {
        // Calculate the GPS coordinates of the prop
        float dist = msg->closest_pnt_dist;
        float angle = msg->closest_pnt_angle;
        float prop_lat = robot_lat_ + (dist * cos(angle)) / 111111.0; // 1 degree of latitude = 111111 meters
        float prop_lon = robot_lon_ + (dist * sin(angle)) / (111111.0 * cos(robot_lat_));
        float prop_alt = robot_alt_;
        
        // Create and publish the Prop message with the prop coordinates
        navigation::Prop prop_msg;
        prop_msg.prop_type = msg->prop_type;
        prop_msg.theta_1 = msg->theta_1;
        prop_msg.theta_2 = msg->theta_2;
        prop_msg.closest_pnt_dist = dist;
        prop_msg.closest_pnt_angle = angle;
        prop_msg.prop_coords.latitude = prop_lat;
        prop_msg.prop_coords.longitude = prop_lon;
        prop_msg.prop_coords.altitude = prop_alt;
        prop_pub_.publish(prop_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_;
    ros::Subscriber prop_sub_;
    ros::Publisher prop_pub_;
    float robot_lat_;
    float robot_lon_;
    float robot_alt_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "prop_locator");
    PropLocator prop_locator;
    prop_locator.spin();
    return 0;
}