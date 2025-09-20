#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header, Float32MultiArray
import numpy as np
import serial
import struct
import time

try:
    import adafruit_bno055
    import board
    import busio
    ADAFRUIT_AVAILABLE = True
except ImportError:
    ADAFRUIT_AVAILABLE = False


class BNO055Interface(Node):
    """ROS2 interface for BNO055 9-DOF IMU sensor."""
    
    # BNO055 Registers
    CHIP_ID = 0xA0
    
    # Operation modes
    OPERATION_MODE_CONFIG = 0x00
    OPERATION_MODE_NDOF = 0x0C
    
    # Output registers
    EULER_H_LSB = 0x1A
    QUATERNION_W_LSB = 0x20
    LINEAR_ACCEL_X_LSB = 0x28
    GRAVITY_X_LSB = 0x2E
    TEMP = 0x34
    CALIB_STAT = 0x35
    
    def __init__(self):
        super().__init__('bno055_interface')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x28)  # or 0x29
        self.declare_parameter('serial_port', '')
        self.declare_parameter('update_rate', 50.0)  # Hz
        self.declare_parameter('use_magnetometer', True)
        self.declare_parameter('calibration_file', '')
        
        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.serial_port = self.get_parameter('serial_port').value
        self.update_rate = self.get_parameter('update_rate').value
        self.use_mag = self.get_parameter('use_magnetometer').value
        self.calib_file = self.get_parameter('calibration_file').value
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)
        self.euler_pub = self.create_publisher(Vector3, 'imu/euler', 10)
        self.calib_pub = self.create_publisher(Float32MultiArray, 'imu/calibration_status', 10)
        
        # Initialize sensor
        if not self.init_sensor():
            self.get_logger().error('Failed to initialize BNO055')
            return
        
        # Load calibration if available
        if self.calib_file:
            self.load_calibration(self.calib_file)
        
        # Timer for sensor updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.sensor_callback)
        
        self.get_logger().info('BNO055 interface initialized')
    
    def init_sensor(self):
        """Initialize BNO055 sensor via I2C or serial."""
        try:
            if ADAFRUIT_AVAILABLE and not self.serial_port:
                # Use Adafruit library for I2C
                i2c = busio.I2C(board.SCL, board.SDA)
                self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=self.i2c_address)
                self.sensor.mode = adafruit_bno055.NDOF_MODE
                return True
            elif self.serial_port:
                # Use serial interface
                self.serial = serial.Serial(self.serial_port, 115200, timeout=1)
                # Add serial initialization commands here
                return True
            else:
                self.get_logger().error('No valid interface available')
                return False
        except Exception as e:
            self.get_logger().error(f'Sensor init failed: {e}')
            return False
    
    def sensor_callback(self):
        """Read sensor data and publish to ROS topics."""
        try:
            # Read all sensor data
            quaternion = self.read_quaternion()
            euler = self.read_euler()
            linear_accel = self.read_linear_acceleration()
            angular_vel = self.read_gyroscope()
            magnetic = self.read_magnetometer()
            temperature = self.read_temperature()
            calibration = self.read_calibration_status()
            
            # Create timestamp
            stamp = self.get_clock().now().to_msg()
            
            # Publish IMU data
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = 'imu_link'
            
            imu_msg.orientation.x = quaternion[1]
            imu_msg.orientation.y = quaternion[2]
            imu_msg.orientation.z = quaternion[3]
            imu_msg.orientation.w = quaternion[0]
            
            imu_msg.angular_velocity.x = angular_vel[0]
            imu_msg.angular_velocity.y = angular_vel[1]
            imu_msg.angular_velocity.z = angular_vel[2]
            
            imu_msg.linear_acceleration.x = linear_accel[0]
            imu_msg.linear_acceleration.y = linear_accel[1]
            imu_msg.linear_acceleration.z = linear_accel[2]
            
            # Set covariance based on calibration status
            calib_level = min(calibration)
            if calib_level == 3:  # Fully calibrated
                orientation_cov = 0.001
                angular_vel_cov = 0.001
                linear_accel_cov = 0.01
            elif calib_level == 2:  # Partially calibrated
                orientation_cov = 0.01
                angular_vel_cov = 0.01
                linear_accel_cov = 0.1
            else:  # Poor calibration
                orientation_cov = 0.1
                angular_vel_cov = 0.1
                linear_accel_cov = 1.0
            
            imu_msg.orientation_covariance[0] = orientation_cov
            imu_msg.orientation_covariance[4] = orientation_cov
            imu_msg.orientation_covariance[8] = orientation_cov
            
            self.imu_pub.publish(imu_msg)
            
            # Publish Euler angles
            euler_msg = Vector3()
            euler_msg.x = euler[0]  # Heading/Yaw
            euler_msg.y = euler[1]  # Roll
            euler_msg.z = euler[2]  # Pitch
            self.euler_pub.publish(euler_msg)
            
            # Publish magnetometer data
            if self.use_mag:
                mag_msg = MagneticField()
                mag_msg.header.stamp = stamp
                mag_msg.header.frame_id = 'imu_link'
                mag_msg.magnetic_field.x = magnetic[0] * 1e-6  # Convert to Tesla
                mag_msg.magnetic_field.y = magnetic[1] * 1e-6
                mag_msg.magnetic_field.z = magnetic[2] * 1e-6
                self.mag_pub.publish(mag_msg)
            
            # Publish temperature
            temp_msg = Temperature()
            temp_msg.header.stamp = stamp
            temp_msg.header.frame_id = 'imu_link'
            temp_msg.temperature = temperature
            self.temp_pub.publish(temp_msg)
            
            # Publish calibration status
            calib_msg = Float32MultiArray()
            calib_msg.data = [float(c) for c in calibration]
            self.calib_pub.publish(calib_msg)
            
        except Exception as e:
            self.get_logger().error(f'Sensor read failed: {e}')
    
    def read_quaternion(self):
        """Read quaternion orientation."""
        if ADAFRUIT_AVAILABLE and hasattr(self, 'sensor'):
            quat = self.sensor.quaternion
            if quat:
                return quat
        return [1.0, 0.0, 0.0, 0.0]  # Identity quaternion
    
    def read_euler(self):
        """Read Euler angles (heading, roll, pitch) in radians."""
        if ADAFRUIT_AVAILABLE and hasattr(self, 'sensor'):
            euler = self.sensor.euler
            if euler:
                # Convert to radians
                return [np.radians(e) if e is not None else 0.0 for e in euler]
        return [0.0, 0.0, 0.0]
    
    def read_linear_acceleration(self):
        """Read linear acceleration (gravity compensated) in m/s^2."""
        if ADAFRUIT_AVAILABLE and hasattr(self, 'sensor'):
            accel = self.sensor.linear_acceleration
            if accel:
                return [a if a is not None else 0.0 for a in accel]
        return [0.0, 0.0, 0.0]
    
    def read_gyroscope(self):
        """Read angular velocity in rad/s."""
        if ADAFRUIT_AVAILABLE and hasattr(self, 'sensor'):
            gyro = self.sensor.gyro
            if gyro:
                # Convert to rad/s
                return [np.radians(g) if g is not None else 0.0 for g in gyro]
        return [0.0, 0.0, 0.0]
    
    def read_magnetometer(self):
        """Read magnetic field in microTesla."""
        if ADAFRUIT_AVAILABLE and hasattr(self, 'sensor'):
            mag = self.sensor.magnetic
            if mag:
                return [m if m is not None else 0.0 for m in mag]
        return [0.0, 0.0, 0.0]
    
    def read_temperature(self):
        """Read temperature in Celsius."""
        if ADAFRUIT_AVAILABLE and hasattr(self, 'sensor'):
            temp = self.sensor.temperature
            if temp is not None:
                return float(temp)
        return 25.0
    
    def read_calibration_status(self):
        """Read calibration status (sys, gyro, accel, mag) 0-3."""
        if ADAFRUIT_AVAILABLE and hasattr(self, 'sensor'):
            calib = self.sensor.calibration_status
            if calib:
                return calib
        return [0, 0, 0, 0]
    
    def save_calibration(self, filename):
        """Save calibration data to file."""
        if ADAFRUIT_AVAILABLE and hasattr(self, 'sensor'):
            calib_data = self.sensor.calibration
            with open(filename, 'wb') as f:
                f.write(bytearray(calib_data))
            self.get_logger().info(f'Calibration saved to {filename}')
    
    def load_calibration(self, filename):
        """Load calibration data from file."""
        try:
            with open(filename, 'rb') as f:
                calib_data = f.read()
            if ADAFRUIT_AVAILABLE and hasattr(self, 'sensor'):
                self.sensor.calibration = calib_data
            self.get_logger().info(f'Calibration loaded from {filename}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load calibration: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Interface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()