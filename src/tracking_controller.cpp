#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include "BSpline.h" // BSpline library is integrated from https://github.com/chen0040/cpp-spline

class BSplineTracker {
public:
    BSplineTracker(ros::NodeHandle& nh)
        : nh_(nh), initialized_(false), total_time_(50.0) {  // Total execution time is 50 seconds
        current_state_ = mavros_msgs::State();
        // Publisher for the target position
        target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

        // Subscriber for waypoints and state
        state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &BSplineTracker::stateCallback, this);
        waypoint_sub_ = nh_.subscribe<geometry_msgs::Polygon>("waypoints", 10, &BSplineTracker::waypointCallback, this);

        // Service clients for arming, setting mode, and landing
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
        // Wait for FCU connection
        waitForConnection();

    }

   

private:
    ros::NodeHandle nh_;
    ros::Publisher target_pub_;
    geometry_msgs::PoseStamped target;
    ros::Subscriber waypoint_sub_;
    ros::Subscriber state_sub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient land_client_;
    ros::Timer timer_;
    mavros_msgs::State current_state_;
    boost::shared_ptr<Curve> curve_;
    ros::Time start_time_;
    double total_time_;
    bool initialized_;

    // Callback to process waypoints
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    // Wait for FCU connection
    void waitForConnection() {
        ros::Rate rate(20);
        while (ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    void waypointCallback(const geometry_msgs::Polygon::ConstPtr& msg) {
         // Disable further callbacks
        waypoint_sub_.shutdown();
        curve_ = boost::make_shared<BSpline>();
        curve_->set_steps(100); // Interpolate 100 points between waypoints

        for (const auto& point : msg->points) {
            curve_->add_way_point(Vector(point.x, point.y, point.z));
        }

        initialized_ = true;
        start_time_ = ros::Time::now();

        ROS_INFO("Received %lu waypoints and initialized trajectory.", msg->points.size());
        offboardMode();
    }
    

    // Set to OFFBOARD mode
    void offboardMode() {
        ros::Rate rate(20);

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();

        while (ros::ok()) {
            if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("OFFBOARD enabled");
                }
                last_request = ros::Time::now();
                start_time_ = ros::Time::now();
            } else {
                if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                    start_time_ = ros::Time::now();
                }
            }
            
        
            ros::Time current_time = ros::Time::now();
            if(current_state_.mode == "OFFBOARD" && current_state_.armed && initialized_){
                double elapsed_time = (current_time - start_time_).toSec();

                // Calculate the current index based on elapsed time
                int total_steps = curve_->node_count();
                int current_index = static_cast<int>((elapsed_time / total_time_) * (total_steps - 1));

                if (current_index >= total_steps) {
                    // Trajectory complete, initiate landing
                    initiateLanding();
                    timer_.stop();
                    return;
                }

                // Get the node at the current index
                Vector pos = curve_->node(current_index);
                
                geometry_msgs::PoseStamped target;
                target.header.stamp = current_time;
                target.header.frame_id = "map";
                target.pose.position.x = pos.x;
                target.pose.position.y = pos.y;
                target.pose.position.z = pos.z;
                target.pose.orientation.w = 1.0; // Neutral orientation
                target_pub_.publish(target);
                // ROS_INFO("Publishing target: [%.2f, %.2f, %.2f] at index=%d", pos.x, pos.y, pos.z, current_index);
            }
            else{
                geometry_msgs::PoseStamped target;
                target.header.stamp = current_time;
                target.header.frame_id = "map";
                target.pose.position.x = 0.0;
                target.pose.position.y = 0.0;
                target.pose.position.z = 1.0;
                target.pose.orientation.w = 1.0; // Neutral orientation
                target_pub_.publish(target);
            }
        
            ros::spinOnce();
            rate.sleep();
            }
    }
    
    

    void initiateLanding() {
        mavros_msgs::CommandTOL land_cmd;
        land_cmd.request.altitude = 0.0;
        land_cmd.request.latitude = 0.0;
        land_cmd.request.longitude = 0.0;
        land_cmd.request.min_pitch = 0.0;
        land_cmd.request.yaw = 0.0;
        initialized_ = false;
        if (land_client_.call(land_cmd) && land_cmd.response.success) {
            ROS_INFO("Landing initiated.");
        } else {
            ROS_ERROR("Failed to initiate landing.");
        }
        waypoint_sub_ = nh_.subscribe<geometry_msgs::Polygon>("waypoints", 10, &BSplineTracker::waypointCallback, this);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracking_controller_cpp");
    ros::NodeHandle nh;

    BSplineTracker tracker(nh);

    ros::spin();
    return 0;
}
