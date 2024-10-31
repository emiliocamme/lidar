#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_frame_tf2_broadcaster')
        # Create a TransformBroadcaster for publishing dynamic transformations
        self.tf_broadcaster = TransformBroadcaster(self)
        # Timer to repeatedly broadcast the transformation every 0.1 seconds
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        self.wheel_radius = 0.05  # Set the radius of your wheels (in meters)
        self.prev_time = time.time()
        self.prev_yaw = None  # To store the previous yaw value for smoothing
        self.cumulative_distance = 0.0  # Track the total distance traveled

    def broadcast_timer_callback(self):
        # Get current time in seconds
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Calculate the robot's position (circular trajectory)
        pos_x = math.sin(current_time)
        pos_y = math.cos(current_time)

        # Calculate the yaw (orientation) angle based on the direction of movement
        yaw = math.atan2(pos_y, pos_x)

        # If there's a previous yaw, unwrap the new yaw to avoid discontinuities
        if self.prev_yaw is not None:
            yaw = self.unwrap_angle(self.prev_yaw, yaw)
        
        self.prev_yaw = yaw  # Store the current yaw for the next iteration
        
        # Convert yaw to degrees
        yaw_degrees = math.degrees(yaw)
        
        # Normalize yaw to the range [-180, 180]
        yaw_degrees = (yaw_degrees + 180) % 360 - 180
        # Print the normalized yaw angle
        
        print(f"Yaw (deg): {yaw_degrees}")


        # Update the cumulative distance traveled by the robot
        linear_velocity = 1.0  # Assume a constant linear velocity
        distance_traveled = linear_velocity * dt
        self.cumulative_distance += distance_traveled

        # Calculate the wheel rotation angle based on the distance traveled
        wheel_rotation_angle = self.cumulative_distance / self.wheel_radius

        # Broadcast the robot's base link (position and orientation)
        self.broadcast_base_link_transform(pos_x, pos_y, yaw)

        # Broadcast the wheel rotation for both wheels (assuming front wheels rotate)
        self.broadcast_wheel_transform('wheel_left', wheel_rotation_angle)
        self.broadcast_wheel_transform('wheel_right', wheel_rotation_angle)

    def broadcast_base_link_transform(self, pos_x, pos_y, yaw):
        # Create a TransformStamped message for the base link
        t = TransformStamped()

        # Set timestamp and frame ID (base_link is the robot's base)
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'reference_point'  # Static reference point as the frame of reference
        t.child_frame_id = 'base_link'  # The moving robot (child frame)

        # Set the position of the robot to follow a circular path
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = 0.0  # No movement in the Z-axis

        # Convert the yaw angle into a quaternion (for 3D orientation)
        t.transform.rotation = self.euler_to_quaternion(0.0, 0.0, yaw)

        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(t)

    def broadcast_wheel_transform(self, wheel_name, rotation_angle):
        # Create a TransformStamped message for the wheel
        t = TransformStamped()

        # Set timestamp and frame ID (wheels are children of the base_link)
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # The robot's base link
        t.child_frame_id = wheel_name  # The specific wheel (left or right)

        # Set the position of the wheels (assuming wheels are fixed in place relative to base_link)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Set the rotation of the wheel based on the calculated rotation angle
        t.transform.rotation = self.euler_to_quaternion(rotation_angle, 0.0, 0.0)

        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(t)

    def unwrap_angle(self, prev_yaw, current_yaw):
        """
        Corrects yaw discontinuities by ensuring smooth transitions between angles.
        """
        delta = current_yaw - prev_yaw
        if delta > math.pi:
            current_yaw -= 2 * math.pi
        elif delta < -math.pi:
            current_yaw += 2 * math.pi
        return current_yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to a quaternion.
        Since the wheels rotate around the X-axis, only roll is used for the wheels.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return TransformStamped().transform.rotation.__class__(
            x=qx, y=qy, z=qz, w=qw
        )
        
def main(args=None):
	rclpy.init(args=args)
	nodeh = DynamicFrameBroadcaster()

	
	try: rclpy.spin(nodeh)
	except Exception as error: print(error)
	except KeyboardInterrupt: print("aios ctrl c")
	
if __name__ == "__main__":
	main()

