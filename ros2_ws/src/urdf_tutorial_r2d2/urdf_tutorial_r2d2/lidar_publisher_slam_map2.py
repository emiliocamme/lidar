#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        
        # Get handles for motors, wheels, and base
        self.motor_left = self.sim.getObject('/leftMotor')
        self.motor_right = self.sim.getObject('/rightMotor')
        self.robot_base = self.sim.getObject('/robot_base_respondable')
        self.left_wheel = self.sim.getObject('/l_wheel')
        self.right_wheel = self.sim.getObject('/r_wheel')

        # ROS2 subscribers for motor velocity control
        self.left_motor_sub = self.create_subscription(Float32, 'leftMotorSpeed', self.set_left_motor_velocity, 10)
        self.right_motor_sub = self.create_subscription(Float32, 'rightMotorSpeed', self.set_right_motor_velocity, 10)
        
        # ROS2 publishers for position
        self.x_pub = self.create_publisher(Float32, '/x_position', 10)
        self.y_pub = self.create_publisher(Float32, '/y_position', 10)
        self.z_pub = self.create_publisher(Float32, '/z_position', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to periodically publish positions and TF
        self.timer = self.create_timer(0.1, self.publish_data)

    def set_left_motor_velocity(self, msg):
        if self.motor_left is not None:
            self.sim.setJointTargetVelocity(self.motor_left, msg.data)
        else:
            self.get_logger().error("Left motor handle is invalid.")

    def set_right_motor_velocity(self, msg):
        if self.motor_right is not None:
            self.sim.setJointTargetVelocity(self.motor_right, msg.data)
        else:
            self.get_logger().error("Right motor handle is invalid.")

    def publish_data(self):
        # Publish the robot's position and TF
        position = self.sim.getObjectPosition(self.robot_base, -1)
        orientation = self.sim.getObjectQuaternion(self.robot_base, -1)
        self.publish_tf(position, orientation, 'map', 'base_link')

        # Publish wheel transforms
        left_wheel_pos = self.sim.getObjectPosition(self.left_wheel, self.robot_base)
        left_wheel_ori = self.sim.getObjectQuaternion(self.left_wheel, self.robot_base)
        self.publish_tf(left_wheel_pos, left_wheel_ori, 'base_link', 'l_wheel')

        right_wheel_pos = self.sim.getObjectPosition(self.right_wheel, self.robot_base)
        right_wheel_ori = self.sim.getObjectQuaternion(self.right_wheel, self.robot_base)
        self.publish_tf(right_wheel_pos, right_wheel_ori, 'base_link', 'r_wheel')

        # Publish position components
        self.publish_position_components(position)

    def publish_tf(self, position, orientation, parent_frame, child_frame):
        # Broadcast a transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]
        t.transform.rotation.x = orientation[0]
        t.transform.rotation.y = orientation[1]
        t.transform.rotation.z = orientation[2]
        t.transform.rotation.w = orientation[3]
        self.tf_broadcaster.sendTransform(t)

    def publish_position_components(self, position):
        # Publish individual position components
        x_msg = Float32(data=position[0])
        y_msg = Float32(data=position[1])
        z_msg = Float32(data=position[2])
        self.x_pub.publish(x_msg)
        self.y_pub.publish(y_msg)
        self.z_pub.publish(z_msg)
        self.get_logger().info(f"Position published: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")




def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        rclpy.spin(node)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print("Shutdown initiated.")

if __name__ == "__main__":
    main()
