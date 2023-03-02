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
        
        // get ROS parameters
        private_nh_.param<std::string>("prop_topic", prop_topic_, "/prop_closest_point");
        private_nh_.param<std::string>("scan_topic", gps_topic_, "/global_position/local");


        // set up publishers and subscribers
        sub_scan_ = nh_.subscribe(gps_topic_, 1, &PropLocator::gpsCallback, this);
        sub_prop_ = nh_.subscribe(prop_topic_, 1, &PropLocator::propCallback, this);
        pub_prop_closest_ = nh_.advertise<navigation::Prop>("/prop_closest_point", 1);
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_prop_;
    ros::Publisher pub_prop_closest_;
    std::string prop_topic_;
    std::string gps_topic_;
    navigation::Prop prop_msg_;
    
    navigation::SimpleGPS gps_msg_;


    void propCallback(const navigation::Prop::ConstPtr& msg) {
        // save the Prop message for later use
        prop_msg_ = *msg;
        ROS_INFO_STREAM("Received Prop message with closest_pnt_dist=" << prop_msg_.closest_pnt_dist
            << " and closest_pnt_angle=" << prop_msg_.closest_pnt_angle);
    }

    void gpsCallback(navigation::SimpleGPS::ConstPtr& msg) {
        // save the scan message for later use
        gps_msg_ = *msg;

        // check if the Prop message is valid
        if (prop_msg_.prop_type.empty() || std::isnan(prop_msg_.theta_1) || std::isnan(prop_msg_.theta_2)) {
            ROS_WARN("Invalid Prop message received");
            return;
        }

        float robot_lat = gps_msg_.latitude;
        float robot_lon  = gps_msg_.longitude;

        float prop_dist = prop_msg_.closest_pnt_dist;
        float prop_angle = prop_msg_.closest_pnt_angle;

        float prop_lat = robot_lat + (prop_dist * cos(prop_angle)) / 111111.0; // 1 degree of latitude = 111111 meters
        float prop_lon = robot_lon + (prop_dist * sin(prop_angle)) / (111111.0 * cos(robot_lat));
        float prop_alt = gps_msg_.altitude;

        // Publish the GPS coordinates of the prop
        navigation::Prop complete_prop_msg;
        complete_prop_msg.prop_type = prop_msg_.prop_type;
        complete_prop_msg.theta_1 = prop_msg_.theta_1;
        complete_prop_msg.theta_2 = prop_msg_.theta_2;
        complete_prop_msg.closest_pnt_dist = prop_msg_.closest_pnt_dist;
        complete_prop_msg.closest_pnt_angle = prop_msg_.closest_pnt_angle;
        complete_prop_msg.prop_coords.latitude = prop_lat;
        complete_prop_msg.prop_coords.longitude = prop_lon;
        complete_prop_msg.prop_coords.altitude = prop_alt;
        
        pub_prop_closest_.publish(complete_prop_msg);
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "prop_locator");
    PropLocator prop_locator;
    prop_locator.spin();
    return 0;
}