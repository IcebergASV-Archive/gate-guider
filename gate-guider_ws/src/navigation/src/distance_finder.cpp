#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <navigation/PropInProgress.h>
#include <cmath>
#include <vector>

class DistanceFinder {
public:
    DistanceFinder() : nh_(""), private_nh_("~") {
        // get ROS parameters
        private_nh_.param<std::string>("prop_topic", prop_topic_, "/prop_angle_range");
        private_nh_.param<std::string>("scan_topic", scan_topic_, "/rect_bot/laser/scan");
        private_nh_.param<double>("max_range", max_range_, 10.0);
        private_nh_.param<double>("laser_angle_min", laser_angle_min, -M_PI/2.0);
        private_nh_.param<double>("laser_angle_max", laser_angle_max, M_PI/2.0);

        sub_scan_ = nh_.subscribe(scan_topic_, 1, &DistanceFinder::scanCallback, this);
        sub_prop_ = nh_.subscribe(prop_topic_, 1, &DistanceFinder::propCallback, this);
        pub_prop_closest_ = nh_.advertise<navigation::PropInProgress>("/prop_closest_point", 1);
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
    std::string scan_topic_;
    double max_range_;
    double laser_angle_min;
    double laser_angle_max;
    double laser_angle_increment;
    float angle_safety_range = 0.0;
    navigation::PropInProgress prop_msg_;
    sensor_msgs::LaserScan scan_msg;

    void propCallback(const navigation::PropInProgress::ConstPtr& msg) {
        // save the PropInProgress message for later use
        prop_msg_ = *msg;
        ROS_INFO_STREAM("Received PropInProgress message with theta_1=" << prop_msg_.theta_1
            << " and theta_2=" << prop_msg_.theta_2);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // save the scan message for later use
        scan_msg = *msg;
        laser_angle_increment = scan_msg.angle_increment;

        // check if the PropInProgress message is valid
        if (prop_msg_.prop_type.empty() || std::isnan(prop_msg_.theta_1) || std::isnan(prop_msg_.theta_2)) {
            ROS_WARN("Invalid PropInProgress message received");
            return;
        }

        // calculate the range indexes for the given theta angles
        float steps = (laser_angle_max * 2) / laser_angle_increment; 
        int index1 = (int)(((prop_msg_.theta_1 + angle_safety_range + (laser_angle_max - (M_PI/2))) / (laser_angle_max*2))* steps);
        int index2 = (int)(((prop_msg_.theta_2 - angle_safety_range + (laser_angle_max - (M_PI/2))) / (laser_angle_max*2))* steps);

        // check that the range indexes are within the range of the scan message
        if (index1 < 0 || index2 < 0 || index1 >= scan_msg.ranges.size() || index2 >= scan_msg.ranges.size()) {
            ROS_WARN("PropInProgress message range indexes are out of bounds for the given scan message");
            return;
        }

        // find the closest point and angle within the given range

        int i = 0;
        float closest_pnt = scan_msg.ranges[i];
        int closest_angle_index = 0;
        for (i = index1; i <= index2; ++i) {
            if (std::isnan(scan_msg.ranges[i])) {
                continue; 
            }
            if(scan_msg.ranges[i] < closest_pnt){
                closest_pnt = scan_msg.ranges[i];
                closest_angle_index = i;
                
                
            }
        }

        // convert index to angle, how many degrees from 90.
        float closest_angle = ((closest_angle_index/steps)*(laser_angle_max*2)) - (laser_angle_max);


        navigation::PropInProgress closest_prop_msg;
        closest_prop_msg.prop_type = prop_msg_.prop_type;
        closest_prop_msg.theta_1 = prop_msg_.theta_1;
        closest_prop_msg.theta_2 = prop_msg_.theta_2;
        closest_prop_msg.closest_pnt_dist = closest_pnt;
        closest_prop_msg.closest_pnt_angle = closest_angle;
        pub_prop_closest_.publish(closest_prop_msg);
    }

    float calculateRadius(const std::vector<float>& distances, const std::vector<float>& angles) {
        // Check that we have at least 6 points - changed to a param later
        if (distances.size() < 6 || angles.size() < 6) {
            throw std::runtime_error("At least 6 points are required to calculate the radius of a cylinder.");
        }

        // Convert angles to x,y coordinates on a unit circle
        std::vector<float> x_coords(distances.size());
        std::vector<float> y_coords(distances.size());
        for (size_t i = 0; i < distances.size(); ++i) {
            x_coords[i] = std::cos(angles[i]);
            y_coords[i] = std::sin(angles[i]);
        }

        // Use linear regression to find the best-fit line for the x,y coordinates
        float x_mean = 0.0f;
        float y_mean = 0.0f;
        for (size_t i = 0; i < distances.size(); ++i) {
            x_mean += x_coords[i];
            y_mean += y_coords[i];
        }
        x_mean /= distances.size();
        y_mean /= distances.size();

        float slope = 0.0f;
        float intercept = 0.0f;
        float numerator = 0.0f;
        float denominator = 0.0f;
        for (size_t i = 0; i < distances.size(); ++i) {
            numerator += (x_coords[i] - x_mean) * (y_coords[i] - y_mean);
            denominator += std::pow(x_coords[i] - x_mean, 2);
        }
        slope = numerator / denominator;
        intercept = y_mean - slope * x_mean;

        // Calculate the center of the best-fit circle for the x,y coordinates
        float center_x = -slope / 2.0f;
        float center_y = intercept - slope * center_x;

        // Calculate the radius of the circle
        float radius = std::sqrt(std::pow(center_x, 2) + std::pow(center_y, 2));

        return radius;
    }

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_finder_node");
    DistanceFinder distance_finder;
    distance_finder.spin();
    return 0;
}