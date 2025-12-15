# utils/ros_data_collector.py

try:
    import rclpy
    import subprocess
    from rclpy.node import Node
    from nav_msgs.msg import Path
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import Odometry
    from cv_bridge import CvBridge
    import numpy as np

    # new imports needed for applying pose to habitat agent
    import habitat_sim
    import magnum as mn
    from scipy.spatial.transform import Rotation as SciRot
    from . import pose_utils

    class ROSDataCollector(Node):
        def __init__(self, ros_enabled=False):
            super().__init__('data_collector')
            self.ros_enabled = ros_enabled
            self.bridge = CvBridge()

            if self.ros_enabled:
                # Initialize ROS publishers
                self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
                self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
                self.pose_pub = self.create_publisher(Odometry, '/camera/pose', 10)
                self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)

        def publish_rgb(self, rgb_img):
            if self.ros_enabled:
                # Process RGB image (flip channels from RGB to BGR)
                rgb_img_processed = rgb_img[:, :, [2, 1, 0]]  # Convert RGB to BGR
                ros_img = self.bridge.cv2_to_imgmsg(rgb_img_processed, encoding="bgr8")
                ros_img.header.stamp = self.get_clock().now().to_msg()
                self.rgb_pub.publish(ros_img)

        def publish_depth(self, depth_img):
            if self.ros_enabled:
                # Process Depth image (convert depth to mm)
                depth_img_processed = (depth_img * 1000).astype(np.uint16)  # Convert meters to millimeters
                ros_depth = self.bridge.cv2_to_imgmsg(depth_img_processed, encoding="16UC1")
                ros_depth.header.stamp = self.get_clock().now().to_msg()
                self.depth_pub.publish(ros_depth)

        def publish_pose(self, pose):
            if self.ros_enabled:
                # Create Odometry message
                odom_msg = Odometry()

                # Set timestamp and coordinate frames
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = 'map'          # Parent coordinate frame (global coordinate frame)
                odom_msg.child_frame_id = ''              # Child coordinate frame (robot itself)

                # Set position (pose[:3]) and orientation (pose[3:7])
                odom_msg.pose.pose.position.x = pose[0]
                odom_msg.pose.pose.position.y = pose[1]
                odom_msg.pose.pose.position.z = pose[2]
                odom_msg.pose.pose.orientation.x = pose[3]
                odom_msg.pose.pose.orientation.y = pose[4]
                odom_msg.pose.pose.orientation.z = pose[5]
                odom_msg.pose.pose.orientation.w = pose[6]

                # Set velocity information (default to 0)
                odom_msg.twist.twist.linear.x = 0.0
                odom_msg.twist.twist.linear.y = 0.0
                odom_msg.twist.twist.linear.z = 0.0
                odom_msg.twist.twist.angular.x = 0.0
                odom_msg.twist.twist.angular.y = 0.0
                odom_msg.twist.twist.angular.z = 0.0

                # Publish Odometry message
                self.pose_pub.publish(odom_msg)

        def publish_camera_info(self, fx, fy, cx, cy, width, height):
            camera_info_msg = CameraInfo()

            # Intrinsic parameters
            camera_info_msg.width = width
            camera_info_msg.height = height
            camera_info_msg.k = [float(fx), 0.0, float(cx), 0.0, float(fy), float(cy), 0.0, 0.0, 1.0]  # 3x3 intrinsic matrix
            camera_info_msg.p = [float(fx), 0.0, float(cx), 0.0, 0.0, float(fy), float(cy), 0.0, 0.0, 0.0, 1.0, 0.0]  # Projection matrix, assuming no distortion

            # Timestamp
            current_time = self.get_clock().now().to_msg()
            camera_info_msg.header.stamp = current_time
            camera_info_msg.header.frame_id = 'camera_frame'

            # Publish
            self.camera_info_pub.publish(camera_info_msg)

    class ROSDataListener(Node):
        def __init__(self, ros_enabled=True, gazebo_topic: str = "/odom"):
            super().__init__('listener_node')
            self.ros_enabled = ros_enabled

            self.latest_path = None  # Used to store the most recently received path
            self.latest_gazebo_pose = None  # (position_list, quat_list) in Habitat coords

            if self.ros_enabled:
                # Subscriber for action_path
                self.action_path_subscriber = self.create_subscription(
                    Path,  # Change to nav_msgs/Path
                    '/action_path',  # Topic to listen to
                    self.action_path_callback,
                    10
                )

                # Subscriber for gazebo robot pose (nav_msgs/Odometry expected)
                self.gazebo_subscriber = self.create_subscription(
                    Odometry,
                    gazebo_topic,
                    self.gazebo_pose_callback,
                    10
                )

                self.get_logger().info(f"Listener initialized and ready to subscribe to topics. Gazebo topic: {gazebo_topic}")

        def action_path_callback(self, msg):
            """
            Callback for the /action_path topic.

            Args:
                msg (Path): The Path message containing the path data.
            """
            # Convert the received path to a simple list representation (x, y, z)
            current_path = [
                (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in msg.poses
            ]
            current_path = self.transform_path_to_habitat(current_path)  # Transform path to Habitat world coordinates

            # Skip processing if the new path is identical to the previous path
            if self.latest_path == current_path:
                # self.get_logger().info("Received path is identical to the latest path. Skipping processing.")
                return

            # Update the latest path
            self.latest_path = current_path
            self.get_logger().info(f"Received global_path with {len(current_path)} poses.")

        def get_latest_path(self):
            """
            Get the latest path received by the listener.

            Returns:
                list or None: The latest path as a list of (x, y, z) tuples, or None if no path is available.
            """
            return self.latest_path

        def transform_path_to_habitat(self, path_sys):

            pose_from_ros = [np.array(pose) for pose in path_sys]

            pose_in_habitat = []

            for point in pose_from_ros:
                # Extend to homogeneous coordinates
                pose_sys = np.eye(4)
                pose_sys[:3, 3] = point  # Only set the translation part
                # Call the get_habitat_pose function
                transformed_pose = self.get_habitat_pose(pose_sys)
                # Extract the transformed 3D coordinates
                transformed_point = transformed_pose[:3, 3]
                # Append the transformed point
                pose_in_habitat.append(tuple(transformed_point))

            return pose_in_habitat

        # --- new gazebo pose handling methods ---
        def gazebo_pose_callback(self, msg: Odometry):
            """
            Callback to receive Odometry from Gazebo, transform it to Habitat coordinates,
            and store the latest pose for application to the Habitat agent.

            Args:
                msg (Odometry): The Odometry message containing the pose data from Gazebo.
            """
            try:
                # Extract position and orientation from Odometry
                px = msg.pose.pose.position.x
                py = msg.pose.pose.position.y
                pz = msg.pose.pose.position.z
                ox = msg.pose.pose.orientation.x
                oy = msg.pose.pose.orientation.y
                oz = msg.pose.pose.orientation.z
                ow = msg.pose.pose.orientation.w

                # Convert quaternion to roll, pitch, yaw (in radians)
                rotation = SciRot.from_quat([ox, oy, oz, ow])
                roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)

                # Print pose information with Euler angles
                print("-" * 25 + "before transformation" + "-" * 25)
                print(f"Position - x: {px:.3f}, y: {py:.3f}, z: {pz:.3f}")
                print(f"Orientation (quat) - x: {ox:.3f}, y: {oy:.3f}, z: {oz:.3f}, w: {ow:.3f}")
                print(f"Orientation (RPY) - roll: {roll:.3f}, pitch: {pitch:.3f}, yaw: {yaw:.3f} (rad)")
                print(f"Orientation (RPY) - roll: {np.degrees(roll):.1f}°, pitch: {np.degrees(pitch):.1f}°, yaw: {np.degrees(yaw):.1f}°")
                print("-" * 60)

                # 变换过程

                hab_pos = [-py, pz, -px]
                # 第1个轴对应实际的 2个对应实际的 3个
                hab_euler = [-pitch, yaw, -roll] #rpy
                hab_quat = pose_utils.euler_to_quat(roll = hab_euler[0],
                                                pitch = hab_euler[1],
                                                yaw = hab_euler[2])  # returns [w, x, y, z]
                hab_quat = [hab_quat[1], hab_quat[2], hab_quat[3], hab_quat[0]]


                # Print pose information with Euler angles
                print("-" * 25 + "after transformation" + "-" * 25)
                print(f"Position - x: {hab_pos[0]:.3f}, y: {hab_pos[1]:.3f}, z: {hab_pos[2]:.3f}")
                print(f"Orientation (quat) - x: {hab_quat[0]:.3f}, y: {hab_quat[1]:.3f}, z: {hab_quat[2]:.3f}, w: {hab_quat[3]:.3f}")
                print(f"Orientation (RPY) - roll: {hab_euler[2]:.3f}, pitch: {hab_euler[1]:.3f}, yaw: {hab_euler[0]:.3f} (rad)")
                print(f"Orientation (RPY) - roll: {np.degrees(hab_euler[2]):.1f}°, pitch: {np.degrees(hab_euler[1]):.1f}°, yaw: {np.degrees(hab_euler[0]):.1f}°")
                print("-" * 60)


                # Store as lists: position list and quaternion [x,y,z,w]
                self.latest_gazebo_pose = (hab_pos, hab_quat)
                self.get_logger().debug(f"Received gazebo odom -> habitat pos: {self.latest_gazebo_pose[0]}, quat: {self.latest_gazebo_pose[1]}")
            except Exception as e:
                self.get_logger().error(f"Error processing gazebo odom: {e}")

        def get_latest_gazebo_pose(self):
            """
            Return the latest gazebo pose transformed into Habitat coordinates.
            Returns:
                tuple or None: (position_list [x,y,z], quat_list [x,y,z,w]) or None if no pose received.
            """
            return self.latest_gazebo_pose

        def apply_pose_to_agent(self, sim: 'habitat_sim.Simulator', agent_id: int = 0, reset_sensors: bool = True):
            """
            Apply the latest received Gazebo pose (already transformed to Habitat coordinates)
            to the given Habitat agent.

            Args:
                sim: Habitat simulator instance.
                agent_id: target agent id (default 0).
                reset_sensors: whether to reset sensors after setting state.
            """
            if self.latest_gazebo_pose is None:
                # nothing to apply
                return False

            pos_list, quat_list = self.latest_gazebo_pose
            try:
                # Construct AgentState with habitat_sim.AgentState
                agent_state = habitat_sim.AgentState()
                agent_state.position = [float(pos_list[0]),
                                        float(pos_list[1]),
                                        float(pos_list[2])]

                # SciPy returns quat as [x,y,z,w], magnum Quaternion uses ((x,y,z), w) format
                qx, qy, qz, qw = float(quat_list[0]), float(quat_list[1]), float(quat_list[2]), float(quat_list[3])
                # Correct way to construct magnum Quaternion: ((vector), scalar)
                agent_state.rotation = [qx, qy, qz, qw]
                # Apply state to agent
                sim.get_agent(agent_id).set_state(agent_state, reset_sensors=reset_sensors, infer_sensor_states=False)
                return True
            except Exception as e:
                self.get_logger().error(f"Failed to apply gazebo pose to agent: {e}")
                return False

        def transform_path_to_habitat(self, path_sys):
            """
            Transform a path from the system coordinate frame to the Habitat coordinate frame.

            This function transforms a given path from the system coordinate frame to the
            Habitat coordinate frame using predefined transformation matrices.

            Args:
                path_sys (list): A list of tuples representing the path in the system
                                 coordinate frame, where each tuple is (x, y, z).

            Returns:
                list: A list of tuples representing the path in the Habitat coordinate frame.
            """
            pose_from_ros = [np.array(pose) for pose in path_sys]

            pose_in_habitat = []

            for point in pose_from_ros:
                # Extend to homogeneous coordinates
                pose_sys = np.eye(4)
                pose_sys[:3, 3] = point  # Only set the translation part
                # Call the get_habitat_pose function
                transformed_pose = self.get_habitat_pose(pose_sys)
                # Extract the transformed 3D coordinates
                transformed_point = transformed_pose[:3, 3]
                # Append the transformed point
                pose_in_habitat.append(tuple(transformed_point))

            return pose_in_habitat

        def get_habitat_pose(self, pose_sys):
            """
            Convert a pose from the system coordinate frame to the Habitat coordinate frame.

            This function transforms a given pose from the system coordinate frame to the
            Habitat coordinate frame using predefined transformation matrices.

            Args:
                pose_sys (numpy.ndarray): A 4x4 transformation matrix representing the
                                          pose in the system coordinate frame.

            Returns:
                numpy.ndarray: A 4x4 transformation matrix representing the pose in the
                               Habitat coordinate frame.
            """
            world_habi_to_world_sys = np.array(
                [[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]
            )
            cam_sys_to_cam_habi = np.array(
                [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
            )
            roll_offset = np.array(
                [[0, -1, 0, 0],
                 [1, 0, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]
            )

            world_sys_to_world_habi = np.linalg.inv(world_habi_to_world_sys)
            cam_habi_to_cam_sys = np.linalg.inv(cam_sys_to_cam_habi)

            # world_sys_to_world_habi @ cam_sys_to_world_sys @ cam_habi_to_cam_sys
            cam_habi_to_world_habi = world_sys_to_world_habi @ pose_sys  @ cam_habi_to_cam_sys

            return cam_habi_to_world_habi

    def start_rosbag_recording(output_path):
        """
        Starts recording rosbag for given topics.

        :param topics: List of ROS topics to record.
        :param output_path: Path to save the rosbag file.
        """

        topics_to_record = ['/camera/rgb/image_raw', '/camera/depth/image_raw', '/camera/pose', '/camera_info']

        # Prepare the command for recording
        command = ['ros2', 'bag', 'record', '-o', output_path] + topics_to_record
        # Start recording in a subprocess
        rosbag_process = subprocess.Popen(command)
        return rosbag_process

    def stop_rosbag_recording(rosbag_process):
        """
        Stops the rosbag recording process.

        :param rosbag_process: The subprocess handle for rosbag recording.
        """
        rosbag_process.terminate()


except ImportError:
    print("ROS 2 environment not found. Skipping ROS 2 integration.")
    ROSDataCollector = None