#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid , Odometry
from std_msgs.msg import Float32  
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        # Publisher for LaserScan
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds
        
        # Set up the TransformBroadcaster for TF messages
        self.tf_broadcaster = TransformBroadcaster(self)

        # Motor speed publishers
        self.left_motor_pub = self.create_publisher(Float32, '/leftMotorSpeed', 10)
        self.right_motor_pub = self.create_publisher(Float32, '/rightMotorSpeed', 10)
        
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
        self.copp_pos_x, self.copp_pos_y, _ = self.sim.getObjectPosition(self.lidar_handle)
        _, _, self.copp_yaw = self.sim.getObjectOrientation(self.lidar_handle)
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = 0.0
        
        # Subscribe to the /map topic
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Subscribe to the /odom topic
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Initialize variable to hold map data
        self.map_data = None

        
        # Flag for the initial scan
        self.initial_scan_done = False
        self.last_turn_direction = "left"
        
    
    def timer_callback(self):
        # Check if initial scan has been completed
        if not self.initial_scan_done:
            self.initial_scan()  # Perform the initial scan
        else:
            # After initial scan, switch to navigation
            self.navigate_based_on_map()
            self.publish_lidar_data()
    
    
    def initial_scan(self):
        # Rotate in place to perform a 360-degree scan
        rotation_speed = 0.65  # Adjust for smoother rotation if needed
        
        self.publish_lidar_data()
        
        # Rotate both motors in opposite directions
        self.publish_motor_speeds(rotation_speed, rotation_speed) #(rig)

        rotation_step = (45*math.pi)/180  # Adjust for smoother updates
        self.yaw += rotation_step
        
        
        # Check if map is generated
        if self.yaw >= 2*math.pi:
            # End initial scan once the map is populated
            self.publish_motor_speeds(0.0, 0.0)  # Stop motors
            self.initial_scan_done = True
            self.get_logger().info("Initial scan complete. Map created.")
    

    def odom_callback(self, msg):
        # Update position and orientation based on odometry data
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y

        # Convert quaternion to yaw angle
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
    

    def navigate_based_on_map(self):
        # Parameters for navigation behavior
        movement_speed  = 0.05  # Step size for movement in meters
        rotation_speed  = 0.25  # Step size for rotation in radians
    
        # Define a target point slightly ahead of the current position
        target_x = self.copp_pos_x + movement_speed * math.cos(self.copp_yaw)
        target_y = self.copp_pos_y + movement_speed * math.sin(self.copp_yaw)
    
        # Check if there’s an obstacle in the target position based on the map
        obstacle_detected = self.check_obstacle_in_map(target_x, target_y)
    
        if obstacle_detected:
            # Decide turn direction based on the last turn
            if self.last_turn_direction == "left":
                self.publish_motor_speeds(-rotation_speed, -rotation_speed)  # Turn right
                self.get_logger().info("Obstacle detected; turning right.")
            else:
                self.publish_motor_speeds(rotation_speed, rotation_speed)  # Turn left
                self.get_logger().info("Obstacle detected; turning left.")
            
            # Recheck for obstacles after the turn, without moving forward
            turned_target_x = self.copp_pos_x + movement_speed * math.cos(self.copp_yaw)
            turned_target_y = self.copp_pos_y + movement_speed * math.sin(self.copp_yaw)
            if not self.check_obstacle_in_map(turned_target_x, turned_target_y):
                # Switch turn direction for the next obstacle
                self.last_turn_direction = "right" if self.last_turn_direction == "left" else "left"
        else:
            # If no obstacle, move forward
            self.publish_motor_speeds(-movement_speed, movement_speed)  # Move forward




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
        buffer_radius = 0.12  # Adjust this as needed; larger values increase stopping distance
    
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

    
    def publish_motor_speeds(self, left_speed, right_speed):
        # Publish speeds to the motor topics
        self.left_motor_pub.publish(Float32(data=left_speed))
        self.right_motor_pub.publish(Float32(data=right_speed))


    def publish_lidar_data(self):
        lidar_data = self.sim.readCustomTableData(self.sim.handle_scene, "lidarData")
        if lidar_data:
            scan_msg = LaserScan()
            scan_msg.header.frame_id = 'robot_base_respondable'
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
    


def main(args=None):
    rclpy.init(args=args)
    nodeh = LidarPublisher()

    try:
        rclpy.spin(nodeh)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        nodeh.destroy_node()
        rclpy.shutdown()
        print("Shutdown initiated.")

if __name__ == "__main__":
    main()

