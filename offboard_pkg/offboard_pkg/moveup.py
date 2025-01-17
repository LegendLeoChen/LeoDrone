import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
import time

class UAVTakeoffNode(Node):
    def __init__(self):
        super().__init__('uav_takeoff_node')
        self.declare_parameter('takeoff_height', 2.0)
        self.takeoff_height = self.get_parameter('takeoff_height').get_parameter_value().double_value
        # 创建订阅者
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.current_state = State()
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/cmd/arming service not available, waiting again...')
        
        self.timer = self.create_timer(1, self.takeoff)

    def takeoff(self):
        self.timer.cancel()  # Ensure this method is called only once
        
        self.get_logger().info('Requesting to arm the UAV')
        arm_request = CommandBool.Request()
        arm_request.value = True
        
        future = self.arm_client.call_async(arm_request)
        future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        try:
            response = future.result()
            if self.current_state.armed:
                self.get_logger().info('UAV arming successful')
                self.set_guided_mode()
                self.send_takeoff_cmd()
                time.sleep(10)
                takeoff_pose = PoseStamped()
                takeoff_pose.header = Header()
                takeoff_pose.header.stamp = self.get_clock().now().to_msg()
                takeoff_pose.header.frame_id = "map"
                
                takeoff_pose.pose.position = Point(x=0.0, y=0.0, z=self.takeoff_height)
                takeoff_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                takeoff_vel = Twist()
                takeoff_vel.linear.x = 0.0
                takeoff_vel.linear.y = 0.0
                takeoff_vel.linear.z = 0.0
                takeoff_vel.angular.x = 0.0
                takeoff_vel.angular.y = 0.0
                takeoff_vel.angular.z = 0.0

                self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
                self.vel_pub = self.create_publisher(Twist, 'mavros/setpoint_velocity/cmd_vel_unstamped', 10)
                self.get_logger().info('Publishing takeoff position')
                self.pose_pub.publish(takeoff_pose)
                self.get_logger().info('Publishing takeoff velocity')
                self.vel_pub.publish(takeoff_vel)
                
            else:
                self.get_logger().error('UAV arming failed.')
                self.get_logger().error('Please run this node again.')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

    def state_callback(self, msg):
        self.current_state = msg

    def set_guided_mode(self):
        set_mode_request = SetMode.Request()
        set_mode_request.custom_mode = "GUIDED"
        future = self.set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().mode_sent:
            self.get_logger().info('Guided mode set successfully')
        else:
            self.get_logger().info('Failed to set Guided mode')

    def send_takeoff_cmd(self):
        takeoff_request = CommandTOL.Request()
        takeoff_request.altitude = 10.0
        takeoff_request.latitude = 0.0
        takeoff_request.longitude = 0.0
        takeoff_request.min_pitch = 0.0
        takeoff_request.yaw = 0.0

        future = self.takeoff_client.call_async(takeoff_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Takeoff cmd send successfully')
        else:
            self.get_logger().info('Failed to send Takeoff cmd')

def main(args=None):
    rclpy.init(args=args)
    node = UAVTakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()

