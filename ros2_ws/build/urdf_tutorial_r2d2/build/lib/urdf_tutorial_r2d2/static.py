#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class FixedFrameBroadcaster(Node):

   def __init__(self):
       super().__init__('fixed_frame_tf2_broadcaster')
       # Create a TransformBroadcaster for publishing transformations
       self.tf_broadcaster = TransformBroadcaster(self)
       # Timer to repeatedly broadcast the transformation every 0.1 seconds
       self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

   def broadcast_timer_callback(self):
       # Create a TransformStamped message
       t = TransformStamped()

       # Set timestamp and frame ID (base_link is the robot's base)
       t.header.stamp = self.get_clock().now().to_msg()
       t.header.frame_id = 'world'  # Static frame (robot base)
       t.child_frame_id = 'reference_point'  # The static reference point

       # Set the position and orientation of the static reference point
       t.transform.translation.x = 0.0
       t.transform.translation.y = 2.0
       t.transform.translation.z = 0.0
       t.transform.rotation.x = 0.0
       t.transform.rotation.y = 0.0
       t.transform.rotation.z = 0.0
       t.transform.rotation.w = 1.0  # No rotation (identity quaternion)

       # Broadcast the transformation
       self.tf_broadcaster.sendTransform(t)

def main(args=None):
	rclpy.init(args=args)
	nodeh = FixedFrameBroadcaster()

	
	try: rclpy.spin(nodeh)
	except Exception as error: print(error)
	except KeyboardInterrupt: print("aios ctrl c")
	
if __name__ == "__main__":
	main()

