#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState, Imu, NavSatFix, TimeReference
from std_msgs.msg import String, Bool, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np
from datetime import datetime, timezone
import os
import json

# Import astronomical libraries
try:
    from astropy.coordinates import EarthLocation, AltAz, get_sun, get_moon
    from astropy.time import Time
    from astropy import units as u
    import astropy.coordinates as coord
    ASTROPY_AVAILABLE = True
except ImportError:
    ASTROPY_AVAILABLE = False
    print("Warning: astropy not available. Using basic calculations.")


class StarTrackerNode(Node):
    def __init__(self):
        super().__init__('star_tracker_node')
        
        # Parameters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('location_lat', 40.7128)  # degrees (fallback if no GPS)
        self.declare_parameter('location_lon', -74.0060)  # degrees (fallback if no GPS)
        self.declare_parameter('location_alt', 10.0)  # meters (fallback if no GPS)
        self.declare_parameter('target_object', 'polaris')
        self.declare_parameter('tracking_mode', 'continuous')
        self.declare_parameter('use_imu', False)
        self.declare_parameter('use_gps', True)  # Use GPS for location and time
        self.declare_parameter('goto_mode', False)
        self.declare_parameter('alignment_file', 'star_alignment.json')
        self.declare_parameter('gps_timeout', 30.0)  # Seconds to wait for GPS fix
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.fallback_lat = self.get_parameter('location_lat').value
        self.fallback_lon = self.get_parameter('location_lon').value
        self.fallback_alt = self.get_parameter('location_alt').value
        self.target = self.get_parameter('target_object').value
        self.tracking_mode = self.get_parameter('tracking_mode').value
        self.use_imu = self.get_parameter('use_imu').value
        self.use_gps = self.get_parameter('use_gps').value
        self.goto_mode = self.get_parameter('goto_mode').value
        self.alignment_file = self.get_parameter('alignment_file').value
        self.gps_timeout = self.get_parameter('gps_timeout').value
        
        # Current location (will be updated by GPS if available)
        self.lat = self.fallback_lat
        self.lon = self.fallback_lon
        self.alt = self.fallback_alt
        
        # GPS state
        self.gps_fix = False
        self.gps_time = None
        self.last_gps_update = None
        
        # Initialize location
        if ASTROPY_AVAILABLE:
            self.location = EarthLocation(lat=self.lat*u.deg, 
                                         lon=self.lon*u.deg, 
                                         height=self.alt*u.m)
        
        # Joint names for SO-100 arm
        self.joint_names = [
            'Shoulder_Rotation',
            'Shoulder_Pitch', 
            'Elbow',
            'Wrist_Pitch',
            'Wrist_Roll'
        ]
        
        # Action client for trajectory execution
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/so_100_arm_controller/follow_joint_trajectory'
        )
        
        # Publishers
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/so_100_arm_controller/joint_trajectory',
            10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # GPS subscribers
        if self.use_gps:
            self.gps_fix_sub = self.create_subscription(
                NavSatFix, 'gps/fix', self.gps_fix_callback, 10
            )
            self.gps_time_sub = self.create_subscription(
                TimeReference, 'gps/time', self.gps_time_callback, 10
            )
            self.gps_status_sub = self.create_subscription(
                Bool, 'gps/has_fix', self.gps_status_callback, 10
            )

        # IMU subscribers (for GoTo mode)
        if self.use_imu:
            self.imu_sub = self.create_subscription(
                Imu, 'imu/data', self.imu_callback, 10
            )
            self.euler_sub = self.create_subscription(
                Vector3, 'imu/euler', self.euler_callback, 10
            )
            self.alignment_status_sub = self.create_subscription(
                Bool, 'alignment/is_aligned', self.alignment_status_callback, 10
            )
        
        # State
        self.current_joint_positions = [0.0] * 5
        self.target_alt_az = (0.0, 0.0)  # altitude, azimuth in radians
        self.current_imu_orientation = None
        self.current_euler = None
        self.is_aligned = False
        self.alignment_transform = None
        
        # Load alignment if in GoTo mode
        if self.goto_mode:
            self.load_alignment()
        
        # Timer for tracking updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.tracking_callback)
        
        self.get_logger().info(f'Star Tracker Node initialized')
        self.get_logger().info(f'Location: Lat={self.lat}, Lon={self.lon}, Alt={self.alt}')
        self.get_logger().info(f'Tracking target: {self.target}')
        if self.use_gps:
            self.get_logger().info(f'GPS integration enabled - waiting for fix...')
        if self.use_imu:
            self.get_logger().info(f'IMU integration enabled - GoTo mode: {self.goto_mode}')
    
    def joint_state_callback(self, msg):
        """Update current joint positions from joint states."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joint_positions[i] = msg.position[idx]
    
    def gps_fix_callback(self, msg):
        """Update location from GPS fix."""
        if msg.status.status >= 0:  # Valid fix
            self.lat = msg.latitude
            self.lon = msg.longitude
            self.alt = msg.altitude
            self.last_gps_update = self.get_clock().now()
            
            # Update astropy location if available
            if ASTROPY_AVAILABLE:
                self.location = EarthLocation(
                    lat=self.lat*u.deg,
                    lon=self.lon*u.deg,
                    height=self.alt*u.m
                )
            
            self.get_logger().info(
                f'GPS location updated: Lat={self.lat:.6f}, Lon={self.lon:.6f}, Alt={self.alt:.1f}m'
            )
    
    def gps_time_callback(self, msg):
        """Update time reference from GPS."""
        self.gps_time = msg.time_ref
        
        # Log time synchronization
        if self.gps_time:
            gps_datetime = datetime.fromtimestamp(self.gps_time, tz=timezone.utc)
            self.get_logger().debug(f'GPS time: {gps_datetime.isoformat()}')
    
    def gps_status_callback(self, msg):
        """Update GPS fix status."""
        prev_status = self.gps_fix
        self.gps_fix = msg.data
        
        if self.gps_fix and not prev_status:
            self.get_logger().info('GPS fix acquired')
        elif not self.gps_fix and prev_status:
            self.get_logger().warn('GPS fix lost - using last known position')
    
    def imu_callback(self, msg):
        """Store current IMU orientation for GoTo calculations."""
        self.current_imu_orientation = msg.orientation
    
    def euler_callback(self, msg):
        """Store current Euler angles from IMU."""
        self.current_euler = np.array([msg.x, msg.y, msg.z])
    
    def alignment_status_callback(self, msg):
        """Update alignment status from calibration node."""
        self.is_aligned = msg.data
        if self.is_aligned and self.goto_mode:
            self.get_logger().info('Alignment confirmed - GoTo mode active')
    
    def tracking_callback(self):
        """Main tracking loop - calculate target position and move arm."""
        if self.goto_mode and self.use_imu:
            # GoTo mode: Use IMU feedback for closed-loop control
            if not self.is_aligned:
                self.get_logger().warn('Waiting for alignment...')
                return
            
            # Get target position
            alt, az = self.calculate_target_position()
            if alt is None or az is None:
                return
            
            # Get current pointing from IMU
            current_alt, current_az = self.get_current_pointing()
            
            # Calculate error
            alt_error = alt - current_alt
            az_error = self.normalize_angle(az - current_az)
            
            # Only move if error is significant
            if abs(alt_error) > np.radians(1) or abs(az_error) > np.radians(1):
                # Calculate corrective joint positions
                joint_positions = self.calculate_goto_trajectory(
                    current_alt, current_az, alt, az
                )
                self.send_trajectory(joint_positions)
                
                self.get_logger().info(
                    f'GoTo {self.target}: Target Alt={np.degrees(alt):.2f}°, Az={np.degrees(az):.2f}° | '
                    f'Error: Alt={np.degrees(alt_error):.2f}°, Az={np.degrees(az_error):.2f}°'
                )
        else:
            # Standard tracking mode (open-loop)
            alt, az = self.calculate_target_position()
            
            if alt is None or az is None:
                return
            
            # Convert to joint angles
            joint_positions = self.altaz_to_joint_angles(alt, az)
            
            # Send trajectory command
            self.send_trajectory(joint_positions)
            
            # Log tracking info
            self.get_logger().info(
                f'Tracking {self.target}: Alt={np.degrees(alt):.2f}°, '
                f'Az={np.degrees(az):.2f}°'
            )
    
    def calculate_target_position(self):
        """Calculate altitude and azimuth of target object."""
        if not ASTROPY_AVAILABLE:
            # Simple calculation for testing without astropy
            if self.use_gps and self.gps_time:
                current_time = datetime.fromtimestamp(self.gps_time, tz=timezone.utc)
            else:
                current_time = datetime.utcnow()
            
            hour_angle = (current_time.hour + current_time.minute/60.0) * 15.0
            
            if self.target == 'polaris':
                # Polaris is approximately at celestial north pole
                alt = np.radians(self.lat)  # Altitude equals latitude
                az = 0.0  # North
            else:
                # Simple sun approximation
                alt = np.radians(45.0)  # Fixed altitude for testing
                az = np.radians(hour_angle)
            
            return alt, az
        
        # Use astropy for accurate calculations
        # Use GPS time if available, otherwise system time
        if self.use_gps and self.gps_time:
            current_time = Time(self.gps_time, format='unix')
        else:
            current_time = Time.now()
        
        altaz_frame = AltAz(obstime=current_time, location=self.location)
        
        if self.target == 'sun':
            obj_coord = get_sun(current_time)
        elif self.target == 'moon':
            obj_coord = get_moon(current_time)
        elif self.target == 'polaris':
            # Polaris coordinates (RA: 2h 31m 49s, Dec: +89° 15' 51")
            obj_coord = coord.SkyCoord(ra='02h31m49s', dec='+89d15m51s')
        elif self.target == 'sirius':
            # Sirius coordinates
            obj_coord = coord.SkyCoord(ra='06h45m09s', dec='-16d42m58s')
        else:
            self.get_logger().warn(f'Unknown target: {self.target}')
            return None, None
        
        # Transform to altitude-azimuth
        obj_altaz = obj_coord.transform_to(altaz_frame)
        
        # Check if object is above horizon
        if obj_altaz.alt.rad < 0:
            self.get_logger().info(f'{self.target} is below horizon')
            return None, None
        
        return obj_altaz.alt.rad, obj_altaz.az.rad
    
    def altaz_to_joint_angles(self, alt, az):
        """Convert altitude/azimuth to robot joint angles."""
        # This is a simplified mapping - adjust based on your robot's kinematics
        # and mounting orientation
        
        # Shoulder rotation controls azimuth
        shoulder_rotation = az
        
        # Shoulder pitch and elbow control altitude
        # Simple approach: use shoulder pitch primarily
        shoulder_pitch = alt - np.pi/2  # Adjust for robot's zero position
        
        # Keep elbow straight for now
        elbow = 0.0
        
        # Wrist angles to keep camera level
        wrist_pitch = -shoulder_pitch  # Compensate for shoulder pitch
        wrist_roll = 0.0
        
        # Clamp to joint limits
        joint_positions = [
            np.clip(shoulder_rotation, -np.pi, np.pi),
            np.clip(shoulder_pitch, -np.pi, np.pi),
            np.clip(elbow, -np.pi, np.pi),
            np.clip(wrist_pitch, -np.pi, np.pi),
            np.clip(wrist_roll, -np.pi, np.pi)
        ]
        
        return joint_positions
    
    def send_trajectory(self, target_positions, duration=2.0):
        """Send joint trajectory command to robot."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(target_positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        trajectory.points = [point]
        
        # Publish trajectory
        self.joint_pub.publish(trajectory)
        
        # Optionally use action client for feedback
        if self.trajectory_client.wait_for_server(timeout_sec=1.0):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            self.trajectory_client.send_goal_async(goal)
    
    def get_current_pointing(self):
        """Get current telescope pointing from IMU data."""
        if self.current_euler is None:
            return 0.0, 0.0
        
        if self.alignment_transform:
            # Apply alignment transformation
            current_alt, current_az = self.apply_alignment_transform(self.current_euler)
        else:
            # Direct mapping (assumes IMU mounted with telescope)
            # This is simplified - adjust based on actual mounting
            current_az = self.current_euler[0]  # Yaw
            current_alt = self.current_euler[2]  # Pitch
        
        return current_alt, current_az
    
    def calculate_goto_trajectory(self, current_alt, current_az, target_alt, target_az):
        """Calculate joint positions for GoTo movement with IMU feedback."""
        # Calculate required movement
        delta_alt = target_alt - current_alt
        delta_az = self.normalize_angle(target_az - current_az)
        
        # Get current joint positions
        current_joints = list(self.current_joint_positions)
        
        # Apply corrections
        # This assumes direct mapping - adjust based on robot kinematics
        current_joints[0] += delta_az  # Shoulder rotation for azimuth
        current_joints[1] += delta_alt  # Shoulder pitch for altitude
        
        # Ensure wrist compensation
        current_joints[3] = -current_joints[1]  # Wrist pitch compensates shoulder
        
        # Clamp to limits
        for i in range(len(current_joints)):
            current_joints[i] = np.clip(current_joints[i], -np.pi, np.pi)
        
        return current_joints
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def apply_alignment_transform(self, euler_angles):
        """Apply alignment transformation to IMU data."""
        if not self.alignment_transform:
            return euler_angles[2], euler_angles[0]  # Default: pitch, yaw
        
        # Apply transformation matrix from alignment
        # This would use the alignment matrix from calibration
        transformed = self.alignment_transform @ euler_angles
        
        return transformed[0], transformed[1]
    
    def load_alignment(self):
        """Load alignment calibration for GoTo mode."""
        filepath = os.path.expanduser(f'~/{self.alignment_file}')
        
        if not os.path.exists(filepath):
            self.get_logger().warn('No alignment file found - GoTo mode will use direct mapping')
            return
        
        try:
            with open(filepath, 'r') as f:
                calib_data = json.load(f)
            
            if calib_data.get('is_aligned', False):
                self.is_aligned = True
                self.alignment_transform = np.array(calib_data.get('alignment_matrix', np.eye(3).tolist()))
                self.get_logger().info('Alignment loaded successfully')
            else:
                self.get_logger().warn('Alignment file exists but system is not aligned')
                
        except Exception as e:
            self.get_logger().error(f'Failed to load alignment: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    # Get environment variables
    lat = float(os.environ.get('LOCATION_LAT', '40.7128'))
    lon = float(os.environ.get('LOCATION_LON', '-74.0060'))
    alt = float(os.environ.get('LOCATION_ALT', '10.0'))
    target = os.environ.get('TARGET_OBJECT', 'polaris')
    
    node = StarTrackerNode()
    
    # Override parameters from environment
    node.lat = lat
    node.lon = lon
    node.alt = alt
    node.target = target
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()