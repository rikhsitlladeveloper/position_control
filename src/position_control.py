#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL
from std_msgs.msg import Float64


class OffboardNode:
    def __init__(self):
        rospy.init_node("offb_node_py")

        # State variable
        self.current_state = State()
        self.attitude = 0.0
        self.data_received = False

        # Subscribers and Publishers
        self.state_sub = rospy.Subscriber("mavros/state", State, callback=self.state_cb)
        self.attitude_sub = rospy.Subscriber("position_control/attitude", Float64, callback=self.attitude_cb)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Service Clients
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        rospy.wait_for_service("/mavros/cmd/land")
        self.land_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)
        
        # Setpoint publishing rate
        self.rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        self.wait_for_connection()

        # Prepare pose
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = self.attitude

        # Send initial setpoints
        self.send_initial_setpoints()

        # Arm and set mode
        self.offboard_mode()

    def state_cb(self, msg):
        self.current_state = msg

    def attitude_cb(self, msg):
        self.attitude = msg.data
        self.data_received = True

    def wait_for_connection(self):
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

    def send_initial_setpoints(self):
        for _ in range(100):
            if rospy.is_shutdown():
                break
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

    def land(self):
         # Ensure the drone is armed and in OFFBOARD mode before landing
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = False
        
        if self.current_state.armed and self.current_state.mode == "OFFBOARD":
            if self.land_client.call().success:
                rospy.loginfo("Landing command sent.")
            else:
                rospy.logerr("Landing command failed.")
        else:
            rospy.logwarn("Cannot land: Drone is not armed or not in OFFBOARD mode.")
        if self.arming_client.call(arm_cmd).success:
            rospy.loginfo("Vehicle not armed")
            
    def offboard_mode(self):
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")
                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()

            # Update pose with the latest attitude
            self.pose.pose.position.z = self.attitude
            if self.attitude <= 0.0 and self.data_received:
                self.land()
                self.data_received = False
                

            if (self.data_received):
                self.local_pos_pub.publish(self.pose)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        OffboardNode()
    except rospy.ROSInterruptException:
        pass
