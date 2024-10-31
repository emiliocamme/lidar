#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import String

class MyClassNode(Node):
	def __init__(self):
		super().__init__("chismoso")
		self.sub = self.create_subscription(String, "hablador", self.callback, 1)

	
	def callback(self, msg):
		self.get_logger().info("que he escuchado? "+msg.data)

def main(args=None):
	rclpy.init(args=args)
	nodeh = MyClassNode()

	
	try: rclpy.spin(nodeh)
	except Exception as error: print(error)
	except KeyboardInterrupt: print("aios ctrl c")
	
if __name__ == "__main__":
	main()

