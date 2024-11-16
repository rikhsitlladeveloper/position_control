#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <geometry_msgs/Point.h>

class OffboardNode {
public:
    OffboardNode() {
        // Initialize ROS node
        ros::NodeHandle nh;

        // Initialize state variable
        current_state_ = mavros_msgs::State();
        x_ = 0.0;
        y_ = 0.0;
        z_ = 0.0;
        data_received_ = false;

        // Subscribers
        state_sub_ = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardNode::stateCallback, this);
        position_sub_ = nh.subscribe<geometry_msgs::Point>("position_control/position", 10, &OffboardNode::attitudeCallback, this);

        // Publishers
        local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

        // Service clients
        arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        land_client_ = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

        // Wait for Flight Controller connection
        waitForConnection();

        // Prepare pose
        pose_.pose.position.x = x_;
        pose_.pose.position.y = y_;
        pose_.pose.position.z = z_;

        // Send initial setpoints
        sendInitialSetpoints();

        // Arm and set mode
        offboardMode();
    }

private:
    // State variables
    mavros_msgs::State current_state_;
    double x_;
    double y_;
    double z_;
    bool data_received_;
    geometry_msgs::PoseStamped pose_;

    // ROS subscribers, publishers, and clients
    ros::Subscriber state_sub_;
    ros::Subscriber position_sub_;
    ros::Publisher local_pos_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient land_client_;

    // Callback functions
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void attitudeCallback(const geometry_msgs::Point::ConstPtr& msg) {
        x_ = msg->x;
        y_ = msg->y;
        z_ = msg->z;
        data_received_ = true;
    }

    // Wait for FCU connection
    void waitForConnection() {
        ros::Rate rate(20);
        while (ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    // Send initial setpoints
    void sendInitialSetpoints() {
        ros::Rate rate(20);
        for (int i = 0; i < 100 && ros::ok(); ++i) {
            local_pos_pub_.publish(pose_);
            rate.sleep();
        }
    }

    // Land the vehicle
    void land() {
        mavros_msgs::CommandTOL land_cmd;
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = false;

        if (current_state_.armed && current_state_.mode == "OFFBOARD") {
            if (land_client_.call(land_cmd) && land_cmd.response.success) {
                ROS_INFO("Landing command sent.");
            } else {
                ROS_ERROR("Landing command failed.");
            }
        } else {
            ROS_WARN("Cannot land: Drone is not armed or not in OFFBOARD mode.");
        }

        if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle disarmed.");
        }
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
            } else {
                if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }

            // Update pose with the latest attitude
            pose_.pose.position.x = x_;
            pose_.pose.position.y = y_;
            pose_.pose.position.z = z_;
            if (z_ <= 0.0 && data_received_) {
                land();
                data_received_ = false;
            }

            if (data_received_) {
                local_pos_pub_.publish(pose_);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Position_Control_node_cpp");
    try {
        OffboardNode offboard_node;
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in OffboardNode: %s", e.what());
    }
    return 0;
}
