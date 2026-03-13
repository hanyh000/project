#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import numpy as np
from tensorflow import keras
import json
import os
import math

class DRLDriveAgent(Node):
    def __init__(self):
        super().__init__('drl_drive_agent')
        
        # 모델 로드
        self.load_model()
        
        # 골 위치 설정
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 2.0)
        
        self.goal_pose_x = self.get_parameter('goal_x').value
        self.goal_pose_y = self.get_parameter('goal_y').value
        
        # 로봇 현재 위치
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.robot_pose_theta = 0.0
        
        # 골 정보
        self.goal_distance = 0.0
        self.goal_angle = 0.0
        
        # LiDAR 데이터
        self.scan_ranges = []
        self.front_ranges = []
        self.min_obstacle_distance = 10.0
        self.front_min_obstacle_distance = 10.0
        
        # QoS 설정
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        cmd_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, sensor_qos)
        
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, sensor_qos)
        
        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', cmd_qos)
        
        self.get_logger().info(f'DRL Drive Agent Ready! Goal: ({self.goal_pose_x}, {self.goal_pose_y})')
    
    def load_model(self):
        """모델 로드"""
        try:
            training_pkg = get_package_share_directory('turtlebot3_dqn')
            model_dir = os.path.join(training_pkg, 'models')
            
            model_path = os.path.join(model_dir, 'stage1_model.h5')
            params_path = os.path.join(model_dir, 'stage1_params.json')
            
            self.get_logger().info(f'Loading model: {model_path}')
            self.model = keras.models.load_model(model_path)
            
            with open(params_path, 'r') as f:
                self.params = json.load(f)
            
            self.get_logger().info('Model loaded successfully!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            raise
    
    def euler_from_quaternion(self, quat):
        """
        Quaternion을 Euler 각도로 변환
        학습 시와 동일한 방식
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def odom_callback(self, msg):
        """
        Odometry 콜백 - 학습 시와 동일한 방식으로 골 정보 계산
        """
        # 로봇 위치 업데이트
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        _, _, self.robot_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        # 골까지의 거리 계산
        goal_distance = math.sqrt(
            (self.goal_pose_x - self.robot_pose_x) ** 2
            + (self.goal_pose_y - self.robot_pose_y) ** 2)
        
        # 골까지의 각도 계산
        path_theta = math.atan2(
            self.goal_pose_y - self.robot_pose_y,
            self.goal_pose_x - self.robot_pose_x)

        goal_angle = path_theta - self.robot_pose_theta
        
        # 각도 정규화 (-π ~ π)
        if goal_angle > math.pi:
            goal_angle -= 2 * math.pi
        elif goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = goal_distance
        self.goal_angle = goal_angle
    
    def scan_callback(self, scan):
        """LiDAR 콜백 - 학습 시와 동일한 전처리"""
        # 1. 전체 스캔 데이터 처리
        self.scan_ranges = []
        num_of_lidar_rays = len(scan.ranges)
        
        for i in range(num_of_lidar_rays):
            distance = scan.ranges[i]
            if distance == float('Inf') or np.isnan(distance):
                distance = 3.5
            self.scan_ranges.append(distance)
        
        # 2. 전방 24개 샘플 추출
        lidar_samples = 24
        self.front_ranges = []
        
        for i in range(-(lidar_samples // 2), lidar_samples // 2):  # -12 ~ 11
            index = i if i >= 0 else (num_of_lidar_rays + i)
            self.front_ranges.append(self.scan_ranges[index])
        
        # 3. 최소 거리 계산
        self.min_obstacle_distance = min(self.scan_ranges)
        self.front_min_obstacle_distance = min(self.front_ranges) if self.front_ranges else 10.0
        
        # 4. State 준비 및 추론
        state = self.prepare_state()
        action = self.model.predict(state, verbose=0)
        
        # 5. 제어 명령 발행
        self.publish_action(action)
    
    def prepare_state(self):
        """
        State 구성: [goal_distance, goal_angle, lidar_ray1, ..., lidar_ray24]
        학습 시와 동일한 방식
        """
        state = np.concatenate([
            [self.goal_distance, self.goal_angle],  # 골 정보 2개
            self.front_ranges  # LiDAR 24개
        ]).reshape(1, -1)
        
        return state
    
    def publish_action(self, action):
        """제어 명령 발행"""
        twist = Twist()
        
        # TurtleBot3 속도 제한
        twist.linear.x = float(np.clip(action[0][0], -0.22, 0.22))
        twist.angular.z = float(np.clip(action[0][1], -2.84, 2.84))
        
        self.cmd_pub.publish(twist)
        
        # 로깅
        self.get_logger().info(
            f'Goal: dist={self.goal_distance:.2f}m, angle={math.degrees(self.goal_angle):.1f}° | '
            f'Action: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f} | '
            f'Front min: {self.front_min_obstacle_distance:.2f}m',
            throttle_duration_sec=1.0  # 1초마다 출력
        )

def main(args=None):
    rclpy.init(args=args)
    agent = DRLDriveAgent()
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        # 정지 명령
        twist = Twist()
        agent.cmd_pub.publish(twist)
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()