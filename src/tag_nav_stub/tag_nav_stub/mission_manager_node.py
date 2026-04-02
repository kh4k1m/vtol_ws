import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import PoseStamped
from marker_interfaces.msg import LocalizerStatus
from std_msgs.msg import String

class MissionState(Enum):
    IDLE = 0
    SEARCH_34 = 1
    ACQUIRE_34 = 2
    ALIGN_34 = 3
    ASCEND_TO_1 = 4
    ACQUIRE_1 = 5
    ASCEND_TO_55 = 6
    ACQUIRE_55 = 7
    TRANSIT_TO_56 = 8
    ACQUIRE_56 = 9
    PHOTO_56 = 10
    TRANSIT_TO_57 = 11
    ACQUIRE_57 = 12
    PHOTO_57 = 13
    RETURN_55 = 14
    ACQUIRE_RETURN_55 = 15
    DESCEND_TO_34 = 16
    LAND_ON_34 = 17
    COMPLETE = 99
    FAILSAFE_HOLD = -1

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.state = MissionState.IDLE
        self.declare_parameter('scenario', 'A')
        self.scenario = self.get_parameter('scenario').value
        
        self.sub_pose = self.create_subscription(PoseStamped, '/vision_pose_enu', self.pose_cb, 10)
        self.sub_status = self.create_subscription(LocalizerStatus, '/localizer_status', self.status_cb, 10)
        
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.photo_pub = self.create_publisher(String, '/photo_trigger', 10)
        
        self.current_status = LocalizerStatus()
        self.current_pose = PoseStamped()
        
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(f"Mission manager started with scenario {self.scenario}")

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def status_cb(self, msg: LocalizerStatus):
        self.current_status = msg

    def tick(self):
        if self.scenario == 'A':
            self.run_scenario_a()
        elif self.scenario == 'B':
            self.run_scenario_b()
        elif self.scenario == 'C':
            self.run_scenario_c()
            
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def run_scenario_a(self):
        # takeoff_land_pad34
        if self.state == MissionState.IDLE:
            self.state = MissionState.SEARCH_34
        elif self.state == MissionState.SEARCH_34:
            if self.current_status.pose_valid and "34" in self.current_status.active_marker_source:
                self.state = MissionState.ACQUIRE_34
        elif self.state == MissionState.ACQUIRE_34:
            self.state = MissionState.ALIGN_34
        elif self.state == MissionState.ALIGN_34:
            self.state = MissionState.LAND_ON_34
        elif self.state == MissionState.LAND_ON_34:
            self.state = MissionState.COMPLETE

    def run_scenario_b(self):
        # ascend_34_1_55
        if self.state == MissionState.IDLE:
            self.state = MissionState.SEARCH_34
        elif self.state == MissionState.SEARCH_34:
            if self.current_status.pose_valid and "34" in self.current_status.active_marker_source:
                self.state = MissionState.ACQUIRE_34
        elif self.state == MissionState.ACQUIRE_34:
            self.state = MissionState.ASCEND_TO_1
        elif self.state == MissionState.ASCEND_TO_1:
            if self.current_status.pose_valid and "1" in self.current_status.active_marker_source:
                self.state = MissionState.ACQUIRE_1
        elif self.state == MissionState.ACQUIRE_1:
            self.state = MissionState.ASCEND_TO_55
        elif self.state == MissionState.ASCEND_TO_55:
            if self.current_status.pose_valid and "55" in self.current_status.active_marker_source:
                self.state = MissionState.ACQUIRE_55
        elif self.state == MissionState.ACQUIRE_55:
            self.state = MissionState.COMPLETE

    def run_scenario_c(self):
        # full_marker_route: 34 -> 1 -> 55 -> 56 -> 57 -> 55
        if self.state == MissionState.IDLE:
            self.state = MissionState.SEARCH_34
        elif self.state == MissionState.SEARCH_34:
            if self.current_status.pose_valid and "34" in self.current_status.active_marker_source:
                self.state = MissionState.ACQUIRE_34
        elif self.state == MissionState.ACQUIRE_34:
            self.state = MissionState.ASCEND_TO_1
        elif self.state == MissionState.ASCEND_TO_1:
            if self.current_status.pose_valid and "1" in self.current_status.active_marker_source:
                self.state = MissionState.ACQUIRE_1
        elif self.state == MissionState.ACQUIRE_1:
            self.state = MissionState.ASCEND_TO_55
        elif self.state == MissionState.ASCEND_TO_55:
            if self.current_status.pose_valid and "55" in self.current_status.active_marker_source:
                self.state = MissionState.ACQUIRE_55
        elif self.state == MissionState.ACQUIRE_55:
            self.state = MissionState.TRANSIT_TO_56
        elif self.state == MissionState.TRANSIT_TO_56:
            if self.current_status.pose_valid and "56" in self.current_status.active_marker_source:
                self.state = MissionState.ACQUIRE_56
        elif self.state == MissionState.ACQUIRE_56:
            self.state = MissionState.PHOTO_56
            msg = String()
            msg.data = "capture_56"
            self.photo_pub.publish(msg)
            self.state = MissionState.TRANSIT_TO_57
        elif self.state == MissionState.TRANSIT_TO_57:
            if self.current_status.pose_valid and "57" in self.current_status.active_marker_source:
                self.state = MissionState.ACQUIRE_57
        elif self.state == MissionState.ACQUIRE_57:
            self.state = MissionState.PHOTO_57
            msg = String()
            msg.data = "capture_57"
            self.photo_pub.publish(msg)
            self.state = MissionState.RETURN_55
        elif self.state == MissionState.RETURN_55:
            if self.current_status.pose_valid and "55" in self.current_status.active_marker_source:
                self.state = MissionState.ACQUIRE_RETURN_55
        elif self.state == MissionState.ACQUIRE_RETURN_55:
            self.state = MissionState.COMPLETE

def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
