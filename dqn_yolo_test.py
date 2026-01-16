# environment 파일 코드에서 카메라 콜백 욜로 인식과 보상체계 점검을 위한 테스트용 코드

import math
import os

import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, CompressedImage
from std_srvs.srv import Empty

from turtlebot3_msgs.srv import Dqn
from turtlebot3_msgs.srv import Goal

class RLEnvironment(Node):

    def __init__(self):
        super().__init__('rl_environment')

        self.target_distance = 0.0
        self.center_offset = 0
        self.target_detected = False

        self.main_group = ReentrantCallbackGroup()
        self.service_group = ReentrantCallbackGroup()

        self.camera_count = 0

        self.goal_pose_x = 2.0
        self.goal_pose_y = 1.8
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0

        self.action_size = 7
        self.max_step = 300

        self.done = False
        self.fail = False
        self.succeed = False

        self.goal_angle = 0.0
        self.goal_distance = 1.0
        self.init_goal_distance = 0.5
        self.scan_ranges = []
        self.front_ranges = []
        self.min_obstacle_distance = 10.0
        self.is_front_min_actual_front = False
        self.prev_front_clearance = None

        self.local_step = 0
        self.stop_cmd_vel_timer = None
        self.angular_vel = [1.5, 0.75, 0.0, -0.75, -1.5, 0.1, 0.2]
        self.normal_checkpoints = [
            [1.0, 0.0]
        ]
        self.normal_cp_idx = 0 
        self.num_checkpoints = len(self.normal_checkpoints)

        self.temp_cp_x = 0.0
        self.temp_cp_y = 0.0
        self.temp_cp_active = False
        self.temp_cp_timer = 0
        self.arrow_processed = False

        self.rotation_score = 0
        self.is_rotating = False
        self.last_box_center_x = 0
        self.box_found_in_current_frame = False

        self.last_rotation_end_step = -9999
        self.rotation_cooldown_steps = 30 

        self.local_step = 0

        self.cv_bridge = CvBridge()
        model_path = os.path.expanduser('/home/dev/turtlebot3_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/turtlebot3_dqn/best.pt')
        self.yolo_model = YOLO(model_path)
        self.detected_object_info = [0.0]

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.image_pub = self.create_publisher(CompressedImage, '/yolo_debug/compressed', 10)


        self.camera_sub = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.camera_sub_callback, qos_profile)

    def camera_sub_callback(self, msg):
        self.local_step += 1
        self.camera_count += 1
        reward = 0

        if self.camera_count % 10 != 0:
            return
        #print("DEBUG: camera_sub_callback 호출됨!")
        try:
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        results = self.yolo_model(cv_image, verbose=False, conf=0.3)

        direction_id = 0.0
        
        # 시각화용 이미지 복사 (원본 보존)
        debug_image = cv_image.copy()

        self.box_found_in_current_frame = False

        current_frame_detected = False

        for r in results:
            for box in r.boxes:
                current_frame_detected = True  # [추가] 객체가 발견되면 True로 변경
                if hasattr(self, 'not_found_counter'):
                    self.not_found_counter = 0  # 발견했으므로 카운터 리셋
                # 1. 정보 추출
                label = self.yolo_model.names[int(box.cls)]
                conf = float(box.conf)

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # 2. 박스 그리기 (녹색 선)
                cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                        # 1. 박스 면적(또는 높이) 계산
                box_width = x2 - x1
                box_height = y2 - y1
                area = box_width * box_height
                center_x = (x1 + x2) / 2

                self.get_logger().info(f"[DETECT] Label: {label}, Area: {area:.0f}")
      
                # 3. 라벨 텍스트 쓰기
                display_text = f"{label} {conf:.2f}"
                cv2.putText(debug_image, display_text, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if "blue_left" in label: direction_id = 1.0
                elif "blue_right" in label: direction_id = 2.0

                current_step = self.local_step
                cooldown_active = (
                    current_step - self.last_rotation_end_step
                ) < self.rotation_cooldown_steps

                if current_step - self.last_rotation_end_step == self.rotation_cooldown_steps:
                    self.get_logger().info("회전 쿨타임 종료")

                if not self.is_rotating and not cooldown_active  and 40000 < area < 100000 and direction_id == 1.0:
                    self.is_rotating = True
                    self.rotation_target_label = "blue_left"
                    self.rotation_score = 0 # 시작 시 점수 초기화
                    self.last_box_center_x = center_x # 기준점 초기화
                    self.get_logger().info(f"회전 시작 감지: {self.rotation_target_label}, 면적: {area:.0f}")

                elif not self.is_rotating and not cooldown_active  and 40000 < area < 100000 and direction_id == 2.0:
                    self.is_rotating = True
                    self.rotation_target_label = "blue_right"
                    self.rotation_score = 0 # 시작 시 점수 초기화
                    self.last_box_center_x = center_x # 기준점 초기화
                    self.get_logger().info(f"회전 시작 감지: {self.rotation_target_label}, 면적: {area:.0f}")

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
                        self.get_logger().info(f"[Score UP] {label}: delta={delta:.1f}, Total={self.rotation_score}")
                    elif delta < 0:
                        self.rotation_score -= 1.0
                        self.get_logger().info(f"[Score DOWN] {label}: delta={delta:.1f}, Total={self.rotation_score}")
                    
                    # 다음 프레임을 위해 현재 위치 저장
                    self.last_box_center_x = center_x

        if not current_frame_detected:
            if hasattr(self, 'not_found_counter'):
                self.not_found_counter += 1
                if self.not_found_counter > 5: # 5번 연속 안 보이면
                    self.is_rotating = False

        msg_out = self.cv_bridge.cv2_to_compressed_imgmsg(debug_image)
        msg_out.header = msg.header # 원본 이미지의 타임스탬프 유지
        self.image_pub.publish(msg_out)

        self.detected_object_info = [float(direction_id)]

        if self.is_rotating:
            if not self.box_found_in_current_frame:
                if self.rotation_score > 1: # 성공 임계값
                    reward += 100.0
                    self.get_logger().info(f"화살표 회전 성공 보너스! {reward}")                
                else:
                    reward -= 100.0
                    self.get_logger().warn(f"화살표 회전 실패 패널티 {reward}")
                self.is_rotating = False
                self.rotation_score = 0

                self.last_rotation_end_step = self.local_step
                self.get_logger().info(f"회전 쿨타임 시작: step={self.last_rotation_end_step}")
            else:
                reward += 0.5

        self.box_found_in_current_frame = False

def main(args=None):
    rclpy.init(args=args)
    rl_environment = RLEnvironment()
    
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=8)
    executor.add_node(rl_environment)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rl_environment.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()