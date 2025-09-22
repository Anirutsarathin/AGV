import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import serial
import math

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
WHEEL_BASE = 0.415
DT = 0.1  # วินาที

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_controller')

        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.last_theta = 0.0

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.create_timer(DT, self.read_serial)
        self.publish_odometry()
        self.publish_tf()

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.send_motor_command(linear_x, angular_z)

    def send_motor_command(self, linear_x, angular_z):
        if linear_x > 0:
            cmd = "RIGHT"  
        elif linear_x < 0:
            cmd = "LEFT"
        elif angular_z < 0:
            cmd = "FORWARD"
        elif angular_z > 0:
            cmd = "BACKWARD"
        else:
            cmd = "STOP"

        self.get_logger().info(f"Sending command to MCU: {cmd}")
        self.ser.write(f"{cmd}\n".encode())

    def read_serial(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode().strip()
                data = line.split(",")
                if data[0] == "ENC":
                    left_speed = float(data[1])
                    right_speed = float(data[2])
                    yaw_deg = float(data[3])

                    new_theta = -math.radians(yaw_deg)  # IMU ให้ clockwise เป็นบวก → ต้องกลับ
                    delta_theta = new_theta - self.theta
                    self.theta = new_theta  # อัปเดตมุม

                    v = (left_speed + right_speed) / 2.0
                    delta_x = v * math.cos(self.theta) * DT
                    delta_y = v * math.sin(self.theta) * DT

                    self.x += delta_x
                    self.y += delta_y

                    self.publish_odometry()
                    self.publish_tf()
            except Exception as e:
                self.get_logger().error(f"❌ Error reading serial: {e}")

    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(odom_msg)

    def publish_tf(self):
        now = self.get_clock().now().to_msg()

        def create_tf(parent, child, x=0.0, y=0.0, z=0.0):
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = parent
            tf.child_frame_id = child
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation.w = 1.0
            return tf

        tf1 = create_tf("odom", "base_footprint", self.x, self.y, 0.0)
        tf1.transform.rotation.z = math.sin(self.theta / 2.0)
        tf1.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(tf1)

        tf2 = create_tf("base_footprint", "base_link", 0.0, 0.0, 0.065)
        self.tf_broadcaster.sendTransform(tf2)

        tf3 = create_tf("base_link", "lidar", 0.10, 0.0, 0.185)
        self.tf_broadcaster.sendTransform(tf3)

        tf4 = create_tf("base_link", "imu_link", -0.12, 0.0, 0.215)
        self.tf_broadcaster.sendTransform(tf4)

        tf5 = create_tf("base_link", "motor_l", 0.0, -0.21, 0.0)
        self.tf_broadcaster.sendTransform(tf5)

        tf6 = create_tf("base_link", "motor_r", 0.0, 0.21, 0.0)
        self.tf_broadcaster.sendTransform(tf6)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
