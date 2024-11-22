#!/usr/bin/env python3
import rclpy, math, time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid , Odometry
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class ExtendedKalmanFilter:
    def __init__(self, process_noise=0.04, measurement_noise=0.15, estimate_error=0.75):
        # State covariance matrix
        self.P = np.diag([estimate_error, estimate_error, estimate_error])
        # Process noise covariance
        self.Q = np.diag([process_noise, process_noise, process_noise])
        # Measurement noise covariance
        self.R = np.diag([measurement_noise, measurement_noise])
        # State vector: [x, y, likelihood]
        self.x = np.zeros(3)

    def predict(self, u):
        """
        Prediction step: Update the state estimate using the process model.
        :param u: Control input [dx, dy] (motion).
        """
        # State transition model
        F = np.eye(3)
        F[0, 2] = u[0]  # Motion in x
        F[1, 2] = u[1]  # Motion in y

        # Update state estimate
        self.x = np.dot(F, self.x)

        # Update state covariance
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q

    def update(self, z):
        """
        Update step: Incorporate the measurement.
        :param z: Measurement [x_map, x_scan]
        """
        # Measurement model (maps the state space to measurement space)
        H = np.array([
            [1, 0, 0],  # Map-based obstacle
            [0, 1, 0],  # Scan-based obstacle
        ])

        # Compute the innovation (measurement residual)
        y = z - np.dot(H, self.x)

        # Compute the innovation covariance
        S = np.dot(np.dot(H, self.P), H.T) + self.R

        # Compute the Kalman gain
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))

        # Update the state estimate
        self.x += np.dot(K, y)
        
        self.x[2] = max(self.x[2], 0)

        # Update the covariance matrix
        self.P = np.dot((np.eye(3) - np.dot(K, H)), self.P)

    def get_obstacle_likelihood(self):
        return self.x[2]  # Return the obstacle likelihood




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
        
        # Subscribtions
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Initialize variable to hold map data
        self.map_data = None
        # Initialize variable to hold scan data
        self.scan_data = None

        
        self.initial_scan_done = False
        self.last_turn_direction = "left"

        # Initialize Kalman Filter
        self.ekf = ExtendedKalmanFilter()

        
    
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
        rotation_step = 0.02  # Step size for rotation in radians
    
        # Define a target point slightly ahead of the current position
        target_x = self.position_x + movement_step * math.cos(self.yaw)
        target_y = self.position_y + movement_step * math.sin(self.yaw)
    
        # Check if there’s an obstacle in the target position based on the map
        obstacle_detected = self.check_obstacle_combined(target_x, target_y)
    
        if obstacle_detected:
            self.get_logger().info(f"Obstacle detected at ({target_x:.2f}, {target_y:.2f}). Deciding turn direction.")

            # Decide turn direction based on the last turn
            if self.last_turn_direction == "left":
                self.yaw -= rotation_step  # Turn right
            else:
                self.yaw += rotation_step  # Turn left
            
            # Recheck for obstacles after the turn, without moving forward
            turned_target_x = self.position_x + movement_step * math.cos(self.yaw)
            turned_target_y = self.position_y + movement_step * math.sin(self.yaw)
            if not self.check_obstacle_combined(turned_target_x, turned_target_y):
                # Switch turn direction for the next obstacle
                self.last_turn_direction = "right" if self.last_turn_direction == "left" else "left"
            
        else:
            # If no obstacle, move forward
            self.position_x = target_x
            self.position_y = target_y
            self.get_logger().info(f"Moving to target position: ({self.position_x}, {self.position_y})")
    

    def check_obstacle_combined(self, target_x, target_y):
        # Map-based obstacle check
        map_based_obstacle = 1.0 if self.check_obstacle_in_map_from_map(target_x, target_y) else 0.0

        # LiDAR-based obstacle check
        lidar_based_obstacle = 1.0 if self.check_obstacle_in_map_from_lidar(target_x, target_y) else 0.0

         # Fuse measurements using the EKF
        self.ekf.predict([target_x - self.position_x, target_y - self.position_y])
        self.ekf.update([map_based_obstacle, lidar_based_obstacle])
        
        self.get_logger().info(f"Likelihood {self.ekf.get_obstacle_likelihood()}")

        # Get the estimated obstacle likelihood
        if self.ekf.get_obstacle_likelihood() < 0:
            return True
        

        return self.ekf.get_obstacle_likelihood() > 0
    

    def check_obstacle_in_map_from_lidar(self, target_x, target_y):
        # Check for obstacles based on scan data
        if self.scan_data is None:
            return False  # No scan data available

        # Calculate the angle to the target position
        target_angle = math.atan2(target_y - self.position_y, target_x - self.position_x)
        angle_to_target = target_angle - self.yaw  # Angle relative to the robot's orientation

        # Normalize angle to be within [-pi, pi]
        angle_to_target = (angle_to_target + math.pi) % (2 * math.pi) - math.pi

        # Find the closest angle in the scan data
        index = int((angle_to_target - self.angle_min) / self.angle_increment)

        if 0 <= index < len(self.scan_data.ranges):
            distance_to_obstacle = self.scan_data.ranges[index]
            if distance_to_obstacle < 0.5:  # Check if obstacle is within 0.3 meter
                return True  # Obstacle detected

        return False  # No obstacle detected


    def check_obstacle_in_map_from_map(self, target_x, target_y):
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

    
    def scan_callback(self, msg):
        # Store the latest scan data for later use
        self.scan_data = msg


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

