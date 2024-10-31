#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import String

class MyClassNode(Node):
	def __init__(self):
		super().__init__("hablador")
		self.counter = 0
		self.create_timer(0.5, self.timer_callback)
		self.pub = self.create_publisher(String, "hablador", 10)
	
	def timer_callback(self):
		self.counter+=1
		message = "oa" + str(self.counter)
		self.get_logger().info(message)
		msg = String()
		msg.data = message
		self.pub.publish(msg)
	

def main(args=None):
	rclpy.init(args=args)
	nodeh = MyClassNode()

	
	try: rclpy.spin(nodeh)
	except Exception as error: print(error)
	except KeyboardInterrupt: print("aios ctrl c")
	
if __name__ == "__main__":
	main()

