
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

class guiNode(Node):
    """
    Class to create the GUI ROS node.

    Inherits from:
        Node (rclpy.node.Node)
    
    Args:
        gui_obj(gui): The GUI object.
        
    Attributes:
        thrusters_subscriber: The subscriber to the thrusters topic.
        depth_subscriber: The subscriber to the depth topic.
        imu_subscriber: The subscriber to the IMU topic.
        vel_subscriber: The subscriber to the velocity topic.
        shapes_pub: The publisher to the shapes topic.
        line_pub: The publisher to the line topic.
        fish_pub: The publisher to the fish topic.
        timer: The timer to publish the flags.
    """
    def __init__(self, gui_obj) -> None:
        super().__init__("GUI_node")
        self.thrusters_subscriber = self.create_subscription(
            Int32MultiArray, "/ROV/thrusters", self.thrusters_callback, 10
        )
        self.depth_subscriber = self.create_subscription(
            Float64, "/ROV/depth", self.depth_callback, 10
        )
        self.imu_subscriber = self.create_subscription(
            Imu, "/ROV/imu", self.imu_callback, 10
        )
        self.vel_subscriber = self.create_subscription(
            Twist, "/ROV/cmd_vel", self.vel_callback, 10
        )

        self.shapes_pub = self.create_publisher(Bool, "/ROV/shapes", 10)
        self.line_pub = self.create_publisher(Bool, "/ROV/line", 10)
        self.fish_pub = self.create_publisher(Bool, "/ROV/fish", 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.gui_obj = gui_obj

    def thrusters_callback(self, 
                           msg: Int32MultiArray) -> None:
        """Method to get the thrusters values."""
        self.gui_obj.thrusters_gui.thrusters = msg.data
        self.gui_obj.thrusters_gui.update_thrusters()

    def depth_callback(self, 
                       msg: Float64) -> None:
        """Method to get the depth value."""
        print("Reached")
        self.gui_obj.vitals_gui.depth = msg.data
        self.gui_obj.vitals_gui.update_vitals()

    def imu_callback(self, 
                     msg: Imu) -> None:
        """Method to get the IMU values."""
        orientation_list = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]
        
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        self.roll = round(self.roll, 1)
        self.pitch = round(self.pitch, 1)
        self.yaw = round(self.yaw, 1)
        self.yaw *= -1
        self.pitch *= -1

        self.gui_obj.vitals_gui.roll = self.roll
        self.gui_obj.vitals_gui.pitch = self.pitch
        self.gui_obj.vitals_gui.yaw = self.yaw
        self.gui_obj.vitals_gui.update_vitals()

    def vel_callback(self, 
                     msg: Twist) -> None:
        """Method to get the velocity values."""
        self.gui_obj.vitals_gui.vx = msg.linear.x
        self.gui_obj.vitals_gui.vy = msg.linear.y
        self.gui_obj.vitals_gui.wz = msg.angular.z
        self.gui_obj.vitals_gui.update_vitals()

    def timer_callback(self) -> None:
        """Method to publish the flags to the respective topics."""
        shapes_msg = Bool()
        shapes_msg.data = self.gui_obj.shapes_bool
        self.shapes_pub.publish(shapes_msg)
        if shapes_msg.data:
            self.get_logger().info(f"Shapes: {shapes_msg.data}")

        fish_msg = Bool()
        fish_msg.data = self.gui_obj.fish_bool
        self.fish_pub.publish(fish_msg)
        if fish_msg.data:
            self.get_logger().info(f"Fish: {fish_msg.data}")

        line_msg = Bool()
        line_msg.data = self.gui_obj.line_bool
        self.line_pub.publish(line_msg)
        if line_msg.data:
            self.get_logger().info(f"Line: {line_msg.data}")

        self.gui_obj.shapes_bool = False
        self.gui_obj.fish_bool = False
        self.gui_obj.line_bool = False