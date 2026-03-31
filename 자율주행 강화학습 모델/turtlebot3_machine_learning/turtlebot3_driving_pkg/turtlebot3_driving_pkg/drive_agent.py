import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan, CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import numpy as np
from tensorflow import keras
import json
import os
import math
from rclpy.callback_groups import ReentrantCallbackGroup

class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving_node')
        
        self.main_group = ReentrantCallbackGroup()

        # 모델 불러오기
        self.model = None
        self.params = None
        self.model_load()

        # 기존 학습 패키지에서 사용된 변수 모음
        # 각 변수를 통해 주행 값을 계산함
        self.goal_pose_x = 2.0
        self.goal_pose_y = 1.8
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.robot_pose_theta = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.scan_ranges = []
        self.front_ranges = [3.5] * 24
        self.min_obstacle_distance = 10.0
        self.front_min_obstacle_distance = 10.0
        self.goal_distance = 1.0
        self.goal_angle = 0.0

        self.camera_count = 0
        self.is_rotating = False
        self.rotation_score = 0.0
        self.last_box_center_x = 0.0
        self.normal_checkpoints = [
            [1.0, 0.5],
            [1.3, 2.0]
        ]
        self.normal_cp_idx = 0
        self.num_checkpoints = len(self.normal_checkpoints)

        self.vel = [1.5, 0.75, 0.0, -0.75, -1.5]

        self.cv_bridge = CvBridge()
        model_path = os.path.expanduser('/home/dev/turtlebot3_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/turtlebot3_dqn/best.pt')
        self.yolo_model = YOLO(model_path)
        self.detected_object_info = [0.0]

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
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, sensor_qos, callback_group=self.main_group)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, sensor_qos)
        self.camera_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.camera_sub_callback, qos_profile, callback_group=self.main_group)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', cmd_qos)
        self.image_pub = self.create_publisher(CompressedImage, '/yolo_debug/compressed', 10)

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('================================================')
        self.get_logger().info('=========== turtlebot3 driving start ===========')
        self.get_logger().info('================================================')

    def model_load(self):
        '''
            학습된 모델 로드 함수
            다른 패키지에서 학습된 학습 모델을 불러와 터틀봇의 상황에 맞는 학습 결과를 도출.
        '''
        try:
            dqn_pkg_path = get_package_share_directory('turtlebot3_dqn')
            model_h5_path = os.path.join(dqn_pkg_path, 'saved_model', 'stage1_best_17.h5')
            model_json_path = os.path.join(dqn_pkg_path, 'saved_model', 'stage1_best_17.json')
            
            self.get_logger().info(f'Loading model from: {model_h5_path}')
            
            # 학습시킨 주행 모델 및 파라미터 로드
            self.model = keras.models.load_model(
                model_h5_path,
                compile=False
            )
            
            with open(model_json_path, 'r') as f:
                self.params = json.load(f)
            
            self.get_logger().info('================================================')
            self.get_logger().info('=========== MODEL LOAD SUCCESSFULLY ============')
            self.get_logger().info('================================================')
            
        except Exception as e:
            self.get_logger().info('================================================')
            self.get_logger().info('=============== MODEL LOAD FAILED ==============')
            self.get_logger().info('================================================')
            self.get_logger().info(f'error : {e}')
            raise
    
    def odom_callback(self, msg):
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

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)

        # 1. 무한대/결측치 처리
        ranges[np.isinf(ranges)] = 3.5
        ranges[np.isnan(ranges)] = 3.5

        num_ranges = len(ranges) 
        lidar_samples = 24
        
        self.front_ranges = []
        
        # 전방 180도만 계산되도록
        for i in range(-(lidar_samples // 2), lidar_samples // 2):
            idx = int(i * (num_ranges / 48))
            if idx < 0:
                idx = num_ranges + idx
                
            self.front_ranges.append(ranges[idx])

        self.min_obstacle_distance = np.min(ranges)
        self.front_min_obstacle_distance = np.min(self.front_ranges)

    # 주행 함수 - 학습 데이터를 통해 예측한 angular_z 값을 발행

    def camera_sub_callback(self, msg):
        self.camera_count += 1

        if self.camera_count % 10 != 0:
            return
        #print("DEBUG: camera_sub_callback 호출됨!")
        try:
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

            #height, width, channels = cv_image.shape

            #self.get_logger().info(f"카메라 수신 중: {width}x{height} 해상도") # <-- 이렇게 고쳐야 함
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        results = self.yolo_model(cv_image, verbose=False, conf=0.95)

        # color_id = 0.0
        direction_id = 0.0
        
        # 시각화용 이미지 복사 (원본 보존)
        debug_image = cv_image.copy()

        img_width = cv_image.shape[1] # 화면 전체 가로 길이
        self.box_found_in_current_frame = False

        for r in results:
            for box in r.boxes:
                # 1. 정보 추출
                label = self.yolo_model.names[int(box.cls)]
                conf = float(box.conf)

                # self.get_logger().info(f"🎯 [YOLO] 객체 탐지: {label} (신뢰도: {conf:.2f})")

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # 2. 박스 그리기 (녹색 선)
                cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                        # 1. 박스 면적(또는 높이) 계산
                box_width = x2 - x1
                box_height = y2 - y1
                area = box_width * box_height
                center_x = (x1 + x2) / 2

                #self.get_logger().info(f"🔍 [DETECT] Label: {label}, Area: {area:.0f}")

                # 3. 라벨 텍스트 쓰기
                display_text = f"{label} {conf:.2f}"
                cv2.putText(debug_image, display_text, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if "blue_left" in label: direction_id = 1.0
                elif "blue_right" in label: direction_id = 2.0

                if not self.is_rotating and 40000 < area < 100000 and direction_id == 1.0:
                    self.is_rotating = True
                    self.rotation_target_label = "blue_left"
                    self.rotation_score = 0 # 시작 시 점수 초기화
                    self.last_box_center_x = center_x # 기준점 초기화

                    self.get_logger().info(f"🌀 회전 시작 감지: {self.rotation_target_label}, 면적: {area:.0f}")

                elif not self.is_rotating and 40000 < area < 100000 and direction_id == 2.0:
                    self.is_rotating = True
                    self.rotation_target_label = "blue_right"
                    self.rotation_score = 0 # 시작 시 점수 초기화
                    self.last_box_center_x = center_x # 기준점 초기화

                    self.get_logger().info(f"🌀 회전 시작 감지: {self.rotation_target_label}, 면적: {area:.0f}")

                # 2. 회전 중 점수 계산 로직
                if self.is_rotating and label == self.rotation_target_label:
                    self.box_found_in_current_frame = True
                    if self.rotation_target_label == "blue_left":
                        # 로봇 좌회전 -> 박스는 오른쪽(+)으로 이동해야 함
                        delta = center_x - self.last_box_center_x
                    else:
                        # 로봇 우회전 -> 박스는 왼쪽(-)으로 이동해야 함
                        delta = self.last_box_center_x - center_x

                    # 점수 증감 판정
                    if delta >= 0:
                        self.rotation_score += 1.0
                        self.get_logger().info(f"↗️ [Score UP] {label}: delta={delta:.1f}, Total={self.rotation_score}")
                    elif delta < 0:
                        self.rotation_score -= 1.0
                        self.get_logger().info(f"🔻 [Score DOWN] {label}: delta={delta:.1f}, Total={self.rotation_score}")
                    
                    # 다음 프레임을 위해 현재 위치 저장
                    self.last_box_center_x = center_x

                if self.is_rotating and not self.box_found_in_current_frame:
                    self.is_rotating = False

        msg_out = self.cv_bridge.cv2_to_compressed_imgmsg(debug_image)
        msg_out.header = msg.header # 원본 이미지의 타임스탬프 유지
        self.image_pub.publish(msg_out)

        self.detected_object_info = [float(direction_id)]

    def control_loop(self):
        # 현재 상태에 따라 모델이 예측
        if len(self.front_ranges) != 24:
            return
        state = self.prepare_state()
        action = self.model.predict(state, verbose=0)
        action_idx = int(np.argmax(action))
        self.get_logger().info(f'Action Index: {action_idx} | Dist: {self.front_min_obstacle_distance:.2f}')
        self.publish_action(action_idx)
        # action = self.model.predict(self.latest_state, verbose=0)
        # self.publish_action(np.argmax(action))

    def publish_action(self, action_idx):
        twist = Twist()
        twist.linear.x = 0.12
        twist.angular.z = self.vel[action_idx]
        
        self.get_logger().info(f'현재 명령 : {self.vel[action_idx]}')
        self.cmd_pub.publish(twist)
        
    def prepare_state(self):
        if self.normal_cp_idx < len(self.normal_checkpoints):
            # 1순위: 아직 통과 못한 상시 CP
            target_x, target_y = self.normal_checkpoints[self.normal_cp_idx]
        else:
            # 2순위: 최종 목적지
            target_x, target_y = self.goal_pose_x, self.goal_pose_y

        diff_x = target_x - self.robot_pose_x
        diff_y = target_y - self.robot_pose_y
        t_dist = math.sqrt(diff_x**2 + diff_y**2)
        t_angle = math.atan2(diff_y, diff_x) - self.robot_pose_theta
        
        while t_angle > math.pi: t_angle -= 2 * math.pi
        while t_angle < -math.pi: t_angle += 2 * math.pi

        state = np.concatenate([
            np.array([t_dist], dtype=np.float32),
            np.array([t_angle], dtype=np.float32),
            np.array(self.front_ranges, dtype=np.float32),
            np.array([1.0 if self.is_rotating else 0.0], dtype=np.float32),
            np.array([self.rotation_score], dtype=np.float32),
            np.array([self.detected_object_info[0]], dtype=np.float32),
        ]).reshape(1, -1)

        if self.normal_cp_idx < self.num_checkpoints:
            target_x, target_y = self.normal_checkpoints[self.normal_cp_idx]
            is_final_goal = False

        else:
            target_x, target_y = self.goal_pose_x, self.goal_pose_y
            is_final_goal = True

        current_dist = math.sqrt(
            (target_x - self.robot_pose_x) ** 2 + (target_y - self.robot_pose_y) ** 2
        )

        if not is_final_goal and current_dist < 0.3:
            self.normal_cp_idx += 1
            self.get_logger().info(f'--- Checkpoint {self.normal_cp_idx} Cleared! ---')

        return state

    def euler_from_quaternion(self, quat):
        # 학습 시, 계산한 각도 함수와 동일하게 구현
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

def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()