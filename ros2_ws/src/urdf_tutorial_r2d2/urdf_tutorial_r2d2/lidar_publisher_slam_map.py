#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid , Odometry
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        # Publisher for LaserScan
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)
        
        # Set up the TransformBroadcaster for TF messages
        self.tf_broadcaster = TransformBroadcaster(self)
        
        
        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        time.sleep(1)

        # Get LiDAR object handle from CoppeliaSim
        self.lidar_handle = self.sim.getObjectHandle('/SickTIM310')  # Replace with your actual object name in CoppeliaSim

        # LiDAR setup parameters
        self.max_range = 4.0
        self.angle_min = -math.pi * 135 / 180
        self.angle_max = math.pi * 135 / 180
        self.angle_increment = math.pi / 180

        # Movement parameters
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = 0.0
        
        # Subscribe to the /map topic
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Twist, '/key_vel', self.key_callback, 10)

        # Initialize variable to hold map data
        self.map_data = None

        self.key_input_vel = Twist(linear=self.list_to_vector3(), angular=self.list_to_vector3())
        self.zero_cov_36 = [0.0]*36
        
        # Flag for the initial scan
        self.initial_scan_done = False
        
    
    def timer_callback(self):
        # Check if initial scan has been completed
        if not self.initial_scan_done:
            self.initial_scan()  # Perform the initial scan
        else:
            # After initial scan, switch to navigation
            self.navigate_based_on_map()
            self.broadcast_transforms(self.position_x, self.position_y, self.yaw)
            self.set_lidar_position_in_coppeliasim(self.position_x, self.position_y, self.yaw)
            self.publish_lidar_data()
            self.publish_odom_data()
    
    
    def initial_scan(self):
    	# algo de que se quede 10 segundos, luego lo haga
        # Rotate in place to perform a 360-degree scan
        rotation_step = (1*math.pi)/180  # Adjust for smoother rotation if needed
        self.yaw += rotation_step
        
        # Broadcast transformation for ROS2 TFs
        self.broadcast_transforms(self.position_x, self.position_y, self.yaw)
        
        # Set LiDAR position in CoppeliaSim
        self.set_lidar_position_in_coppeliasim(self.position_x, self.position_y, self.yaw)
        
        # Publish LiDAR data to create the map
        self.publish_lidar_data()

        self.publish_odom_data()
        
        # Check if map is generated
        if self.yaw >= 2*math.pi:
            # End initial scan once the map is populated
            self.initial_scan_done = True
            self.get_logger().info("Initial scan complete. Map created.")
        

        
    def navigate_based_on_map(self):
        # Parameters for navigation behavior
        movement_step = 0.005  # Step size for movement in meters
        rotation_step = 0.025  # Step size for rotation in radians

        # Define a target point slightly ahead of the current position
        target_x = self.position_x + movement_step * math.cos(self.yaw)
        target_y = self.position_y + movement_step * math.sin(self.yaw)

        # Check if there’s an obstacle in the target position based on the map
        obstacle_detected = self.check_obstacle_in_map(target_x, target_y)

        if obstacle_detected:
            # If an obstacle is detected, decide to turn left
            self.yaw += rotation_step  # Turn left as an example
            self.get_logger().info("Obstacle detected; turning left.")
        else:
            # If no obstacle, move forward
            self.position_x = target_x
            self.position_y = target_y
            self.get_logger().info(f"Moving to target position: ({self.position_x}, {self.position_y})")
    
    def navigate_based_on_keystrokes(self):
        self.position_x += (self.key_input_vel.linear.x * math.cos(self.yaw) \
                                    + self.key_input_vel.linear.y * -math.sin(self.yaw))
        self.position_y += (self.key_input_vel.linear.x * math.sin(self.yaw) \
                                    + self.key_input_vel.linear.y * math.cos(self.yaw))
        self.yaw += self.key_input_vel.angular.z
        #self.get_logger().info(f"Moving to target position: ({self.position_x}, {self.position_y}, {self.yaw})")


    def check_obstacle_in_map(self, target_x, target_y):
        # Check if map data is available
        if self.map_data is None:
            return False  # No map data available
    
        # Get the map's metadata
        resolution = self.map_data.info.resolution  # Size of each cell in meters
        origin_x = self.map_data.info.origin.position.x  # X position of the map's origin
        origin_y = self.map_data.info.origin.position.y  # Y position of the map's origin
    
        # Convert target position to map coordinates
        map_x = int((target_x - origin_x) / resolution)
        map_y = int((target_y - origin_y) / resolution)
    
        # Define a buffer zone radius (in meters) around the target point
        buffer_radius = 0.18  # Adjust this as needed; larger values increase stopping distance
    
        # Determine the number of cells to check around the target point
        buffer_cells = int(buffer_radius / resolution)
    
        # Check cells around the target point within the buffer zone
        for dx in range(-buffer_cells, buffer_cells + 1):
            for dy in range(-buffer_cells, buffer_cells + 1):
                # Calculate coordinates for neighboring cells
                neighbor_x = map_x + dx
                neighbor_y = map_y + dy
    
                # Ensure the neighbor coordinates are within the map bounds
                if (0 <= neighbor_x < self.map_data.info.width and
                    0 <= neighbor_y < self.map_data.info.height):
    
                    # Convert 2D coordinates to a 1D index
                    index = neighbor_y * self.map_data.info.width + neighbor_x
                    occupancy_value = self.map_data.data[index]
    
                    # Assume values above a certain threshold indicate an obstacle, 100 occupied 0 free 
                    if occupancy_value > 5:  # Threshold for detecting obstacles
                        return True  # Obstacle detected within buffer zone
    
        return False  # No obstacles detected within buffer zone


    def map_callback(self, msg):
        # Store the latest map data
        self.map_data = msg

    
    def key_callback(self, msg):
        key_vel : Twist = msg
        key_vel.angular.z *= 0.05
        key_vel.linear.x *= 0.05
        key_vel.linear.y *= 0.05
        self.key_input_vel = key_vel


    def broadcast_transforms(self, pos_x, pos_y, yaw):
        # Broadcast the transform from map to reference_point
        self.broadcast_map_to_reference_point(pos_x, pos_y, yaw)


    def broadcast_map_to_reference_point(self, pos_x, pos_y, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.euler_to_quaternion(0.0, 0.0, yaw)
        self.tf_broadcaster.sendTransform(t)
        #self.get_logger().info(f"Map to reference: ({self.position_x}, {self.position_y}), Yaw: {self.yaw}")


    def set_lidar_position_in_coppeliasim(self, pos_x, pos_y, yaw):
        # Set position in CoppeliaSim
        self.sim.setObjectPosition(self.lidar_handle, -1, [pos_x, pos_y, 0.1])

        # Set orientation in CoppeliaSim
        self.sim.setObjectOrientation(self.lidar_handle, -1, [0, 0, yaw])


    def publish_lidar_data(self):
        lidar_data = self.sim.readCustomTableData(self.sim.handle_scene, "lidarData")
        if lidar_data:
            scan_msg = LaserScan()
            scan_msg.header.frame_id = 'base_link'
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
            scan_msg.angle_increment = self.angle_increment
            scan_msg.range_min = 0.1
            scan_msg.range_max = self.max_range

            ranges = []

            st_angle = int(math.degrees(lidar_data[1]))
            end_angle = int(math.degrees(lidar_data[-1]))
            last_deg = st_angle

            # add data for missing angles at beginning of scan
            for i in range(int(math.degrees(self.angle_min)), st_angle):
                ranges.append(self.max_range + 10)

            for i in range(0, len(lidar_data), 2):
                distance = lidar_data[i]
                angle = lidar_data[i + 1]
                deg = int(math.degrees(angle))
                diff = deg - last_deg
                # only add in 1 deg increments, pad data when an angle is skipped
                for i in range(diff):
                    ranges.append(distance) 
                last_deg = deg

            # add data for missing angles at end of scan
            for i in range(end_angle, int(math.degrees(self.angle_max))+1):
                ranges.append(self.max_range + 10)

            scan_msg.ranges = ranges
            self.publisher_.publish(scan_msg)
            #self.get_logger().info("Published LaserScan data to RViz.")
        else:
            self.get_logger().info('No data retrieved from CoppeliaSim.')
    

    def publish_odom_data(self):
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, self.yaw)
        odom.pose.covariance = self.zero_cov_36
        odom.twist.twist = self.key_input_vel
        odom.twist.covariance = self.zero_cov_36
        self.odom_publisher_.publish(odom)
                

    def euler_to_quaternion(self, roll, pitch, yaw):
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

        return TransformStamped().transform.rotation.__class__(x=qx, y=qy, z=qz, w=qw)
    
    def list_to_vector3(self, l=[0.0, 0.0, 0.0]):
        if len(l) != 3:
            return None
        v = Vector3()
        v.x = l[0]
        v.y = l[1]
        v.z = l[2]
        return v
    


def main(args=None):
    rclpy.init(args=args)
    nodeh = LidarPublisher()

    try:
        rclpy.spin(nodeh)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Shutdown initiated.")

if __name__ == "__main__":
    main()

