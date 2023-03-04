#include <ros/ros.h>
#include <navigation/Prop.h>
#include <navigation/PropArray.h>

class PropMapping {
public:
    PropMapping()
    {
        prop_sub_ = nh_.subscribe("/prop_coordinates", 1, &PropMapping::propCallback, this);
        prop_pub_ = nh_.advertise<navigation::PropArray>("/prop_array", 1);
        
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    void propCallback(const navigation::Prop::ConstPtr& msg)
    {
        // get prop info in variables
        navigation::Prop prop;
        prop.prop_type = msg->prop_type;
        prop.prop_coords.latitude = msg->prop_coords.latitude;
        prop.prop_coords.latitude = msg->prop_coords.longitude;
        prop.prop_coords.altitude = msg->prop_coords.altitude;
        prop.prop_coord_range.latitude = msg->prop_coord_range.latitude;
        prop.prop_coord_range.latitude = msg->prop_coord_range.longitude;
        prop.prop_coord_range.altitude = msg->prop_coord_range.altitude;


        //check if prop is not already in array
        for (int i = 0, i < len(prop_array); i++) {
            navigation::Prop checkprop = prop_array[i]
            if (prop_array[i] cout << i << " = " << cars[i] << "\n";
        }
        //pushback prop into array
        prop_array.props.pushback(prop);

        //publish array
        
        
        // Create and publish the Prop message with the prop coordinates
       
        prop_pub_.publish(prop_array);
    }

    ros::NodeHandle nh_;
    ros::Subscriber prop_sub_;
    ros::Publisher prop_pub_;
    navigation::PropArray prop_array;

    void propChecker(){
        
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "prop_mapping_node");
    PropMapping prop_mapper;
    prop_mapper.spin();
    return 0;
}