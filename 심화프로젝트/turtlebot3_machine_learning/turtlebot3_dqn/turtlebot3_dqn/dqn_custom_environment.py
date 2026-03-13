#!/usr/bin/env python3
#################################################################################
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################
#
# Authors: Ryan Shim, Gilbert, ChanHyeong Lee

# 기존 dqn_environment 파일을 수정한 코드
import math
import os

import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge

import numpy
import rclpy

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup 
# 통신 이슈로 콜백 그룹을 분리 
# MutuallyExclusiveCallbackGroup 은 한 번에 하나씩만 실행
# ReentrantCallbackGroup 은 동시에 여러 개가 실행

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from rclpy.node import Node

from std_srvs.srv import Empty

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

from nav_msgs.msg import Odometry

from sensor_msgs.msg import LaserScan, CompressedImage

from turtlebot3_msgs.srv import Dqn
from turtlebot3_msgs.srv import Goal


ROS_DISTRO = os.environ.get('ROS_DISTRO')


class RLEnvironment(Node):

    def __init__(self):
        super().__init__('rl_environment')

        self.main_group = ReentrantCallbackGroup()
        self.service_group = ReentrantCallbackGroup()
        # 메인에는 카메라 스캔과 같은 센서콜백들이 해당하고
        # 서비스에는 rl_agent_interface_service, reset_environment_service 해당

        self.camera_count = 0 # 카메라 프레임 처리수를 줄여 메모리 사용량을 줄이기 위한 초기화 값 설정

        self.goal_pose_x = 2.0
        self.goal_pose_y = 1.8
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        # 골인 지점과 로봇의 위치, 시작점 위치를 잡아서 시작위치로 부터 로봇이 얼마나 떨어졌는지
        # 골인 지점으로 얼마나 가까워졌는지를 확인 하기 위한 초기 값

        self.action_size = 5
        self.max_step = 300
        # 행동 사이즈와 최대 스탭수 최대 스텝수를 넘으면 에피소드 종료

        self.done = False
        self.fail = False
        self.succeed = False
        # 에피소드가 완료 되었고 성공인지 실패인지 확인 하기 위한 초기 값

        self.goal_angle = 0.0
        self.goal_distance = 1.0
        self.init_goal_distance = 0.5
        # 골과의 각도와 골과의 거리를 계산하여 점수에 반영하기 위한 초기 값

        self.scan_ranges = []
        self.front_ranges = []
        # 스캔 범위 값과 전방 범위 값을 설정하기 위한 초기 값

        self.min_obstacle_distance = 10.0
        self.is_front_min_actual_front = False
        self.prev_front_clearance = None
        #장애물과의 최소 거리를 파악하기 위해 설정한 초기 값

        self.local_step = 0
        # 스텝수를 파악해서 도착하기 까지 얼마나 걸렸는지 또는 시작하고 얼마나 빨리 박았는지를
        # 파악해서 보상을 주기 위한 초기 값

        self.stop_cmd_vel_timer = None
        # 타이머 객체를 담아두어, 콜백 내부에서 타이머를 파괴하고 
        # 현재 정지 명령 타이머가 동작 중인지 확인하는 상태 플래그 역할을 하는 초기 값 

        self.angular_vel = [1.5, 0.75, 0.0, -0.75, -1.5]
        # 5개 사이즈인 액션이 회전 행동을 정하기 위해 지정한 Z값 리스트

        self.normal_checkpoints = [
            [1.0, 0.5],
            [1.3, 2.0]
        ]
        self.normal_cp_idx = 0
        self.num_checkpoints = len(self.normal_checkpoints)
        # 초반 장애물 구역을 빠져나가기 쉽게 도움을 주기 위해 지정한 체크 포인트와 
        # 체크포인트가 여러개 설정할 때를 대비한 id 초기값 

        self.rotation_score = 0
        self.is_rotating = False
        self.last_box_center_x = 0
        self.box_found_in_current_frame = False

        self.last_rotation_end_step = -9999
        self.rotation_cooldown_steps = 10   # 3~5 권장
        # 카메라로 특정 객체를 인식하면 그 객체 라베링을 따라 회전 방향을 정하도록 하고 
        # 같은 화살표를 연속으로 잡지 못하게 하기 위한 초기 값 

        self.cv_bridge = CvBridge()
        model_path = os.path.expanduser('/home/dev/turtlebot3_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/turtlebot3_dqn/best.pt')
        self.yolo_model = YOLO(model_path)
        self.detected_object_info = [0.0]
        # 욜로 모델을 불러와 인식한 라벨의 정보를 숫자화 하여 저장하기 위한 초기 값 

        qos = QoSProfile(depth=10)

        if ROS_DISTRO == 'humble':
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        else:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_sub_callback,
            qos,
            callback_group=self.main_group
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_sub_callback,
            qos_profile_sensor_data,
            callback_group=self.main_group
        )
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.camera_sub = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.camera_sub_callback, qos_profile,callback_group=self.main_group)
        # real bot case:         
        # self.img_subscriber = self.create_subscription(CompressedImage, '/image_raw/compressed', self.camera_sub_callback, qos_profile_sensor_data)
        # 실제 사용할 카메라 토픽과 가제보에서 쓰는 카메라 토픽 분리한 코드 
    
        self.clients_callback_group = MutuallyExclusiveCallbackGroup()
        # 여기에는 task_succeed_client, task_failed_client, initialize_environment_client 이런 클라이언트들이 해당

        self.task_succeed_client = self.create_client(
            Goal,
            'task_succeed',
            callback_group=self.clients_callback_group
        )
        self.task_failed_client = self.create_client(
            Goal,
            'task_failed',
            callback_group=self.clients_callback_group
        )
        self.initialize_environment_client = self.create_client(
            Goal,
            'initialize_env',
            callback_group=self.clients_callback_group
        )

        self.rl_agent_interface_service = self.create_service(
            Dqn,
            'rl_agent_interface',
            self.rl_agent_interface_callback,
            callback_group=self.service_group)

        self.make_environment_service = self.create_service(
            Empty,
            'make_environment',
            self.make_environment_callback
        )
        self.reset_environment_service = self.create_service(
            Dqn,
            'reset_environment',
            self.reset_environment_callback,
            callback_group=self.service_group)

        self.image_pub = self.create_publisher(CompressedImage, '/yolo_debug/compressed', 10)
        # rqt에서 이미지를 볼 수 있도록 발행하는 코드 

    def make_environment_callback(self, request, response):
        # 호출 되었을 때 로그를 띄우고 
        # initialize_environment_client가 응답하지 못하면 1초마다 기다림 로그를 띄우고 
        # 그 후 respose_goal 이 성공이 아니면 실패 로그를 띄우고 
        # 성공이면 성공한 좌표를 같이 띄워 성공 로그를 띄움

        self.get_logger().info('Make environment called')
        while not self.initialize_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'service for initialize the environment is not available, waiting ...'
            )

        response_goal = self.initialize_environment_client.call(Goal.Request())

        if not response_goal.success:
            self.get_logger().error('initialize environment request failed')
        else:
            self.goal_pose_x = response_goal.pose_x
            self.goal_pose_y = response_goal.pose_y
            self.get_logger().info(
                'goal initialized at [%f, %f]' % (self.goal_pose_x, self.goal_pose_y)
            )

        return response

    def reset_environment_callback(self, request, response):
        # 호출 되었을 때 ROS 버전을 파악하고 
        # ROS 버전에 맞는 정지 메시지를 발행 하면서 로그 띄움
        # state 구조에서 계산을 하기 위해 사용되거나 종료 되었음을 알리기 위한 값들을 초기화 해서 
        # 에피소드가 끝난 후 이어서 계산하지 못하게 하기 위한 값들 
        # 호출 했을때 state 값을 저장 하고 현재 목표거리를 init_goal_distance 와 
        # prev_goal_distance에 저장하고 로봇의 좌표를 시작 좌표로 저장 

        stop_msg = Twist() if ROS_DISTRO == 'humble' else TwistStamped()
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info('--- Resetting Environment for New Episode ---')

        self.local_step = 0
        self.done = False
        self.fail = False
        self.succeed = False
        self.normal_cp_idx = 0
        self.is_rotating = False
        self.rotation_score = 0
        self.detected_object_info = [0.0]
        self.prev_front_clearance = None
        self.last_rotation_end_step = -9999

        '''
        작동 문제로 주석 처리한 부분 
        retry_count = 0
        while abs(self.robot_pose_x) > 0.05 or abs(self.robot_pose_y) > 0.05:
            rclpy.spin_once(self, timeout_sec=0.1)
            retry_count += 1
            if retry_count > 20: break
        '''

        state = self.calculate_state()
        self.init_goal_distance = state[0]
        self.prev_goal_distance = self.init_goal_distance
        
        self.start_x = self.robot_pose_x
        self.start_y = self.robot_pose_y

        response.state = state
        return response

    def call_task_succeed(self):
        # 성공시 불러와지며 서비스 불러오기가 지연이 되면 로그를 띄우고
        # 리스폰스에 값이 있으면 로스폰스x,y 값을 골 좌표로 설정하고 성공 로그를 띄우고 
        # 이게 아니면 불러오기 실패 로그를 띄움 
        while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for task succeed is not available, waiting ...')

        response = self.task_succeed_client.call(Goal.Request())

        if response is not None:
            self.goal_pose_x = response.pose_x
            self.goal_pose_y = response.pose_y
            self.get_logger().info('service for task succeed finished')
        else:
            self.get_logger().error('task succeed service call failed')

    def call_task_failed(self):
        # 실패시 불러와지며 서비스 불러오기가 지연이 되면 로그를 띄우고
        # 리스폰스에 값이 있으면 로스폰스x,y 값을 골 좌표로 설정하고 실패 로그를 띄우고 
        # 이게 아니면 불러오기 실패 로그를 띄움
        while not self.task_failed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for task failed is not available, waiting ...')
            return
        
        response = self.task_failed_client.call(Goal.Request())

        if response is not None:
            self.goal_pose_x = response.pose_x
            self.goal_pose_y = response.pose_y
            self.get_logger().info('service for task failed finished')
        else:
            self.get_logger().info('task failed service call failed')

    def camera_sub_callback(self, msg):
        # 프레임 수 10번당 1번으로 제한하고 컴프레스드 이미지를 cv브릿지로 cv2로 전환
        # 욜로 모델을 가져와 특정 객체를 감지하면 박스와 라벨이름으로 객체를 표시 
        # 감지한 박스 크기와 쿨타임을 계산하고 회전중 동작 여부 , 박스 크기와 쿨타임 작동여부를 파악하여
        # 맞으면 회전 동작을 시작하고 특정 객체를 인식한 박스가 왼쪽 방향으로 가는 것이면 
        # 오른쪽으로 사라져야 하는 왼쪽 오른쪽 방향에 대한 로직이 있음
        # 객체 인식 했을때 박스 의 중앙 x 좌표를 저장하여 올바른 방향으로 회전 했으면 
        # 회전 점수가 양수로 저장되어 보상 쪽에서 양수이면 100점 음수이면 -100점을 주는 식으로 작동하고 있음 
        # 이 보상주는 것이 끝나면 현재 스탭을 쿨타임 시작 스탭으로 두고 10스탭정도 차이나면 쿨타임이 끝나는 로직 
        # rqt에서 확인하기 위해 박스와 라벨 이름을 표시하는 이미지를 /yolo_debug/compressed 라는 이름으로 발행
        # state에서 사용하기 위해 blue_left = 1.0 blue_right = 2.0 이렇게 저장한 값을 state에 전송
        
        self.camera_count += 1

        if self.camera_count % 5 != 0:
            return
        # print("DEBUG: camera_sub_callback 호출됨!")
        # 로그 뜨는지 확인하기 위한 코드 였던 것

        try:
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

            #height, width, channels = cv_image.shape
            #self.get_logger().info(f"카메라 수신 중: {width}x{height} 해상도")
            # 해상도 체크를 하기 위한 코드 였던 것 
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        results = self.yolo_model(cv_image, verbose=False, conf=0.3)

        direction_id = 0.0
        
        # 시각화용 이미지 복사 (원본 보존)
        debug_image = cv_image.copy()

        self.box_found_in_current_frame = False

        for r in results:
            for box in r.boxes:
                # 1. 정보 추출
                label = self.yolo_model.names[int(box.cls)]
                conf = float(box.conf)

                # self.get_logger().info(f"[YOLO] 객체 탐지: {label} (신뢰도: {conf:.2f})")
                # 객체를 탐지하는지 출력하는 확인용 로그 코드 였던 것 

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # 2. 박스 그리기 (녹색 선)
                cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # 1. 박스 면적(또는 높이) 계산
                box_width = x2 - x1
                box_height = y2 - y1
                area = box_width * box_height
                center_x = (x1 + x2) / 2

                #self.get_logger().info(f"[DETECT] Label: {label}, Area: {area:.0f}")
                # 박스 크기를 출력하는 확인용 로그 코드 였던 것

                # 3. 라벨 텍스트 쓰기
                display_text = f"{label} {conf:.2f}"
                cv2.putText(debug_image, display_text, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # ROI 추출 및 색상 판별 로직
                # 라벨링이 바뀌기 전 색상 판별을 위해 사용하던 색상 마스킹 코드 였던 것
                '''
                roi = cv_image[y1:y2, x1:x2]
                if roi.size == 0: continue

                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                # ... [기존 mask_red, mask_blue, mask_black 처리 로직 생략] ...

                lower_red = numpy.array([0, 100, 100]); upper_red = numpy.array([10, 255, 255])
                lower_blue = numpy.array([90, 50, 50]); upper_blue = numpy.array([130, 255, 255])

                counts = {
                    1.0: cv2.countNonZero(cv2.inRange(hsv_roi, lower_blue, upper_blue)),   # Blue
                    2.0: cv2.countNonZero(cv2.inRange(hsv_roi, lower_red, upper_red))      # Red
                }

                # 5. 가장 많이 검출된 색상 찾기 (max_color 정의)
                max_color = max(counts, key=counts.get)
                
                # 색상 판별 후 debug_image에 추가 정보 기입
                roi_area = roi.shape[0] * roi.shape[1]
                if counts[max_color] > (roi_area * 0.2): # ROI의 20% 이상이 해당 색상일 때
                    color_id = int(max_color)

                    color_name = {1.0: "Blue", 2.0: "Red"}.get(color_id, "Unknown")
                    cv2.putText(debug_image, f"Color: {color_name}", (x1, y2 + 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                '''

                if "blue_left" in label: direction_id = 1.0
                elif "blue_right" in label: direction_id = 2.0

                current_step = self.local_step
                cooldown_active = (
                    current_step - self.last_rotation_end_step
                ) < self.rotation_cooldown_steps

                if current_step - self.last_rotation_end_step == self.rotation_cooldown_steps:
                    self.get_logger().info("회전 쿨타임 종료")


                if not self.is_rotating and not cooldown_active and 30000 < area < 100000 and direction_id == 1.0:
                    self.is_rotating = True
                    self.rotation_target_label = "blue_left"
                    self.rotation_score = 0 # 시작 시 점수 초기화
                    self.last_box_center_x = center_x # 기준점 초기화
                    self.get_logger().info(f"회전 시작 감지: {self.rotation_target_label}, 면적: {area:.0f}")

                elif not self.is_rotating and not cooldown_active and 30000 < area < 100000 and direction_id == 2.0:
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
                    if delta > 0:
                        self.rotation_score += 1.0
                        self.get_logger().info(f"[Score UP] {label}: delta={delta:.1f}, Total={self.rotation_score}")
                    elif delta < 0:
                        self.rotation_score -= 1.0
                        self.get_logger().info(f"[Score DOWN] {label}: delta={delta:.1f}, Total={self.rotation_score}")
                    
                    # 다음 프레임을 위해 현재 위치 저장
                    self.last_box_center_x = center_x

        msg_out = self.cv_bridge.cv2_to_compressed_imgmsg(debug_image)
        msg_out.header = msg.header # 원본 이미지의 타임스탬프 유지
        self.image_pub.publish(msg_out)

        self.detected_object_info = [float(direction_id)]

    def scan_sub_callback(self, scan):
        # 스캔 값을 받아 스캔 범위를 정하고 전방 각도와 범위를 지정
        # 스캔 데이터의 레이저를 48개로 설정하여 전방은 180도를 24개로 나누고 inf나 nan 값을 3.5로 지정함
        # 최소 장애물 거리는 최소 라이다 값으로 하여 장애물이 일정 거리 미만일때 최소 장애물 거리를 이용하여 계산
        # 전방 범위를 state에 전송

        #self.get_logger().info("Scan 수신 중...", throttle_duration_sec=1.0)
        # 콜콘 로그 이슈로 바뀐 정보가 저장이 안되는 것을 모르고 사용했던 확인용 로그 코드 였던것 ...

        self.scan_ranges = []
        self.front_ranges = []
        self.front_angles = []

        num_of_lidar_rays = len(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        for i in range(num_of_lidar_rays):
            distance = scan.ranges[i]
            if distance == float('Inf') or numpy.isnan(distance):
                distance = 3.5
            self.scan_ranges.append(distance)

        lidar_samples = 24
        for i in range(-(lidar_samples // 2), lidar_samples // 2):
            index = i if i >= 0 else (num_of_lidar_rays + i)
            self.front_ranges.append(self.scan_ranges[index])

            self.front_angles.append(angle_min + index * angle_increment)

        self.min_obstacle_distance = min(self.scan_ranges)

        #self.front_min_obstacle_distance = min(self.front_ranges) if self.front_ranges else 10.0
        # 지금은 사용하지 않는 전방 최소 장애물 거리 저장 코드 였던 것

    def odom_sub_callback(self, msg):
        # 로봇의 x,y 좌표를 메시지로 받아 오고  theta 값을 euler_from_quaternion 이 함수에서 계산해서 정보를 받아옴
        # 골과의 거리를 절대 값으로 계산하여 저장하고 각도를 계산해서 골지점을 바라보는 각도를 저장함
        # 골과의 거리는 골인 성공여부를 따질때 사용하고 바라보는 각도는 reward 쪽에서 사용함

        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        _, _, self.robot_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        goal_distance = math.sqrt(
            (self.goal_pose_x - self.robot_pose_x) ** 2
            + (self.goal_pose_y - self.robot_pose_y) ** 2)
        path_theta = math.atan2(
            self.goal_pose_y - self.robot_pose_y,
            self.goal_pose_x - self.robot_pose_x)

        goal_angle = path_theta - self.robot_pose_theta
        if goal_angle > math.pi:
            goal_angle -= 2 * math.pi

        elif goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = goal_distance
        self.goal_angle = goal_angle

    def calculate_state(self):
        # 체크포인트의 남은 수에 따라 타겟 x,y가 정해지고 이 x,y 값을 이용해서 타겟 각도나 거리를 정함
        # 스테이트 구조는 타겟 거리와 각도, 스캔데이터의 범위를 쪼갠 (24개의 범위 값)
        # 화살표를 감지하여 회전하고 있는지 확인 하는 값 회전 했을때 받는 회전 점수와 
        # 객체를 인식했을때 나오는 라벨 값을 저장한 값으로 이루어지고 
        # if self.local_step == 1: 이 부분은 이전 에피소드가 끝나고 리셋하는 과정에서 
        # 시작시 첫 스탭 부터 성공이나 실패가 뜨는 것을 방지 하기 위해 설정
        # 성공 실패 기준으로 성공했는지 아니면 실패 했는지를 파악하고 성공, 실패와 관련된 함수에 호출 신호 보냄

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

        state = []
        state.append(float(t_dist))
        state.append(float(t_angle))

        for var in self.front_ranges:
            state.append(float(var))

        state.append(float(1.0 if self.is_rotating else 0.0))
        state.append(float(self.rotation_score))

        # state.append(self.detected_object_info[0]) # color
        state.append(self.detected_object_info[0]) # direction
        
        self.local_step += 1

        if self.local_step == 1:
            return state

        # 성공 판정
        if self.goal_distance < 0.20:
            self.get_logger().info('Goal Reached')
            self.succeed = True
            self.done = True
            self.cmd_vel_pub.publish(Twist() if ROS_DISTRO == 'humble' else TwistStamped())
            self.call_task_succeed()

        # 실패 판정 (충돌)
        if self.min_obstacle_distance < 0.17:
            self.get_logger().info('Collision happened')
            self.fail = True
            self.done = True
            self.cmd_vel_pub.publish(Twist() if ROS_DISTRO == 'humble' else TwistStamped())
            self.call_task_failed()

        # 실패 판정 (타임아웃)
        if self.local_step == self.max_step:
            self.get_logger().info('Time out!')
            self.fail = True
            self.done = True
            self.cmd_vel_pub.publish(Twist() if ROS_DISTRO == 'humble' else TwistStamped())
            self.call_task_failed()

        return state

    def compute_directional_weights(self, relative_angles, max_weight=10.0):
        # 정면일 때 값이 가장 크고 옆으로 갈수록 점점 작아지며 방향에 따른 가중치를 계산 +0.1은 측면 최소 점수
        # 계산된 값 중 가장 큰 값이 max_weight가 되도록 비유을 키움
        # 모든 가중치의 합이 1이 되도록 하여 여러 장애물 거리 값들과 곱해 평균을 낼때 안정적으로 계산
        # 이렇게 계산 한 값을 compute_weighted_obstacle_reward으로 넘김

        power = 6
        raw_weights = (numpy.cos(relative_angles))**power + 0.1
        scaled_weights = raw_weights * (max_weight / numpy.max(raw_weights))
        normalized_weights = scaled_weights / numpy.sum(scaled_weights)
        return normalized_weights

    def compute_weighted_obstacle_reward(self):
        # 전방 범위나 각도가 없으면 0.0으로 초기화
        # 0.5 미터 이내에 있으면 감지하고 없으면 0.0으로 초기화하고 전방 0.5 미터 이내에 있는 데이터만 뽑아서 저장 
        # 각도 값이 180 에서 -180으로 갑자기 변경되는 현상을 방지하고 정면 기준으로 계산하기 편하게 만듬
        # compute_directional_weights 여기서 계산한 값을 불러와 relative_angles, max_weight 값을 지정하고 저장
        # 안전 거리 0.25 미터로 지정하고 안전거리로 부터 가까워지면 위험 지수가 급격하게 상승하도록 계싼
        # compute_directional_weights를 이용한 weights값과 위험지수 decay 값을 이용해서 벌점 계산하여 reward에 반영

        if not self.front_ranges or not self.front_angles:
            return 0.0

        front_ranges = numpy.array(self.front_ranges)
        front_angles = numpy.array(self.front_angles)

        valid_mask = front_ranges <= 0.5
        if not numpy.any(valid_mask):
            return 0.0

        front_ranges = front_ranges[valid_mask]
        front_angles = front_angles[valid_mask]

        relative_angles = numpy.unwrap(front_angles)
        relative_angles[relative_angles > numpy.pi] -= 2 * numpy.pi

        weights = self.compute_directional_weights(relative_angles, max_weight=10.0)

        safe_dists = numpy.clip(front_ranges - 0.25, 1e-2, 3.5)
        decay = numpy.exp(-3.0 * safe_dists)

        weighted_decay = numpy.dot(weights, decay)

        reward = - (1.0 + 4.0 * weighted_decay)

        return reward

    def calculate_reward(self):
        # 리워드 초기화 하고 체크포인트 번호가 남은 체크포인트 수보다 낮으면 최종골 비활성화하고 높으면 최종골 활성화
        # 타겟의 좌표와 로봇의 좌표를 비교하여 현재 남은 거리를 계산 하고 
        # rl_agent_interface_callback에서 계산한 prev_goal_distance에서 current_dist값을 뺸 값을 distance_rate로 저장함
        # 체크포인트에 도달하면 100점을 받고 체크포인트 번호를 올리면서 로그를 띄우고 이전 골 거리로 현재 거리를 저장하고 reward 반영함
        # 로봇 좌표와 시작 좌표를 비교하여 절대 값 처리를 하고 50스탭 동안 시작 위치에서 멀어질 수록 점수를 받음
        # 50에서 125 스탭동안 스탭 보너스를 받고 126 이상이면 점수가 점점 떨어지는 것을 reward에 반영함
        # 스캔 콜백에서 받아온 최소 장애물 거리가 0.4 보다 작으면 obstacle_near에 저장하고 
        # obstacle_near가 아니면 계산한 distance_rate을 가지고 점수를 30을 곱할지 10을 곱할지 정함
        # obstacle_near이면 -점수가 들어가도록 해서 reward에 반영함
        # obstacle_near이고 전방 범위가 24개 이상이면 범위 중앙 8개 값만 저장
        # 저장한 front_clearance 을 prev_front_clearance값으로 저장하고 prev_front_clearance이 있으면
        # front_clearance값을 prev_front_clearance으로 빼서 저장하고 점수에 반영함
        # 스캔 콜백에서 받아온 min_obstacle_distance 값이 0.25보다 작으면 0.25에서 최소 장애물 거리를 뺀 값을 저장하여 -점수로 reward에 반영함
        # 장애물이 없으면 골 지점과의 각도에 따라 reward를 반영함 
        # 특정 객체를 인식해 회전이 활성화 되고 박스가 시야에서 사라졌다면 회전 점수에 따라 25점을 받거나 25점이 차감됨
        # 그리고 회전 점수와 회전을 비활성화 하고 쿨타임을 적용시킴 박스가 있다면 점수를 0.5점씩 추가하고 reward에 반영
        # 골인에 성공헀으면 200점을 주고 추가로 120 스탭 안에 들어왔으면 (120 - 현재 스탭) *100점을 주고 
        # 실패 했으면 -200점 추가로 50스탭 안에 실패 했으면 -300을 주어 reward에 반영함

        reward = 0.0

        #reward -= 0.1
        # 시작부터 점수를 깍으면 더 빨리 가지 않을까 하여 추가한 지속적 -점수 코드 였던 것

        if self.normal_cp_idx < self.num_checkpoints:
            target_x, target_y = self.normal_checkpoints[self.normal_cp_idx]
            is_final_goal = False

        else:
            target_x, target_y = self.goal_pose_x, self.goal_pose_y
            is_final_goal = True

        current_dist = math.sqrt(
            (target_x - self.robot_pose_x) ** 2 + (target_y - self.robot_pose_y) ** 2
        )
        distance_rate = (self.prev_goal_distance - current_dist)


        if not is_final_goal and current_dist < 0.3:
            reward += 100.0
            self.normal_cp_idx += 1
            self.get_logger().info(f'--- Checkpoint {self.normal_cp_idx} Cleared! ---')
            self.prev_goal_distance = current_dist # 거리 동기화
            return reward

        # [추가] 시작 지점으로부터의 물리적 거리 계산
        dist_from_start = math.sqrt(
            (self.robot_pose_x - self.start_x) ** 2 + 
            (self.robot_pose_y - self.start_y) ** 2
        )

        # 1. 탈출 보상: 초반 200스텝 동안 시작점에서 멀어질수록 점수 부여
        if self.local_step < 50:
            reward += dist_from_start * 5.0  

        step_bonus = 0.0
        if 50 <= self.local_step <= 125:
            step_bonus = 0.5
        elif self.local_step >= 126:
            step_bonus = -0.5

        reward += step_bonus

        obstacle_near = self.min_obstacle_distance < 0.4

        if not obstacle_near:
            # --------장애물 없음 --------
            if distance_rate > 0:
                reward += distance_rate * 30.0
            else:
                reward += distance_rate * 10.0
        else:
            # --------장애물 근접 --------
            # goal reward를 강제로 약화
            reward += distance_rate * -2.0

        if obstacle_near and len(self.front_ranges) >= 24:
            # 전방 24개 중 중앙 8개만 사용
            front_clearance = sum(self.front_ranges[8:16]) / 8.0

            if self.prev_front_clearance is not None:
                delta_clearance = front_clearance - self.prev_front_clearance
                reward += delta_clearance * 8.0  # 핵심 계수

            self.prev_front_clearance = front_clearance

        if self.min_obstacle_distance < 0.25:
            reward -= (0.25 - self.min_obstacle_distance) * 35.0

        '''
        장애물 에 대한 보상 코드 였던 것
        if self.min_obstacle_distance < 0.25:
            reward -= (0.25 - self.min_obstacle_distance) * 35.0
            reward_approach = 0.0
        else:
            # self.goal_distance 대신 위에서 계산한 distance_rate를 사용합니다.
            if distance_rate > 0:
                reward_approach = distance_rate * 40.0 # 전진 보상 강화
            else:
                reward_approach = distance_rate * 10.0 # 후퇴 감점 완화
        
        reward += reward_approach
        '''

        # 3. Yaw 보상 (각도 차이 패널티 10배 낮춤 유지)
        if not obstacle_near:
            reward_yaw = (1 - 2 * abs(self.goal_angle) / math.pi) * 0.005
            reward += reward_yaw

        if self.is_rotating:
            if not self.box_found_in_current_frame:
                if self.rotation_score >= 1: # 성공 임계값
                    reward += 25.0
                    self.get_logger().info("화살표 회전 성공 보너스!")
                
                else:
                    reward -= 25.0
                    self.get_logger().warn("화살표 회전 실패 패널티")
                
                self.is_rotating = False
                self.rotation_score = 0

                self.last_rotation_end_step = self.local_step
                self.get_logger().info(f"회전 쿨타임 시작: step={self.last_rotation_end_step}")
            else:
                reward += 0.5 

        self.box_found_in_current_frame = False

        # 4. 종료 보상
        if self.succeed:
            reward = 300.0
            self.get_logger().info(f"SUCCESS: Goal Reached (Steps: {self.local_step})")
            if self.local_step < 120:
                best_approach = 100 * (120 - self.local_step)
                reward += best_approach

        elif self.fail:
            reward = -200.0
            if self.local_step < 50:
                reward -= 300.0 
            self.get_logger().info(f"FAIL: Collision at Step {self.local_step}")

        return reward

    def rl_agent_interface_callback(self, request, response):
        # 호출 되었을 때 ROS 버전을 파악하고 버전에 맞는 주행 토픽을 메시지로 저장함
        # agent에서 state를 보고 정한 action 값에 따라 회전 값을 줌
        # 받은 회전값으로 토픽을 발행함
        # 계산된 state 와 reward를 리스폰스에 저장하고 그 값을 agent에 보낼때 사용
        # 체크포인트 번호가 체크포인트 수 보다 작으면 체크 포인트 좌표를 로봇 좌표에서 뺀 절대 값을 prev_goal_distance로 저장
        # 아니면 최종 골 좌표에서 로봇 좌표를 뺀 절대 값을 prev_goal_distance로 저장하고 완료 상태를 리스폰스에 반영함

        action = request.action
        if ROS_DISTRO == 'humble':
            msg = Twist()

            '''
            # 감속, 가속과 같은 기능을 추가하는 코드 였던것 
            if action > 4:
                msg.linear.x = self.angular_vel[action]
                msg.angular.z = 0.0
            elif action <= 4:
            '''

            msg.linear.x = 0.15
            msg.angular.z = self.angular_vel[action]
        else:
            msg = TwistStamped()
            msg.twist.linear.x = 0.2
            msg.twist.angular.z = self.angular_vel[action]

        self.cmd_vel_pub.publish(msg)

        response.state = self.calculate_state()
        response.reward = self.calculate_reward()

        if self.normal_cp_idx < self.num_checkpoints:
            # 리스트 인덱싱 방식 그대로 유지
            self.prev_goal_distance = math.sqrt(
                (self.normal_checkpoints[self.normal_cp_idx][0] - self.robot_pose_x)**2 + 
                (self.normal_checkpoints[self.normal_cp_idx][1] - self.robot_pose_y)**2
            )
        else:
            self.prev_goal_distance = math.sqrt(
                (self.goal_pose_x - self.robot_pose_x)**2 + (self.goal_pose_y - self.robot_pose_y)**2
            )

        response.done = self.done

        return response
    
    '''
    # 기존 코드 dqn 코드에서 사용하던 코드 였던 것
    def timer_callback(self):
        self.get_logger().info('Stop called')
        if ROS_DISTRO == 'humble':
            self.cmd_vel_pub.publish(Twist())
        else:
            self.cmd_vel_pub.publish(TwistStamped())
        
        if self.stop_cmd_vel_timer is not None:
            self.destroy_timer(self.stop_cmd_vel_timer)
            self.stop_cmd_vel_timer = None
    '''

    def euler_from_quaternion(self, quat):
        # 로봇의 쿼터니언(3D자세 데이터)를 추출하여 roll은 옆으로 기우는 정도 pitch는 앞뒤가 들리는 정도 
        # yaw는 왼쪽 오른쪽 회전하는 정도 와 같은 우리가 알수 있는 각도로 변환함 
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    # ROS 2 통신을 초기화하고 RLEnvironment 노드를 생성 및 init함수가 실행되며  init에 있는 설정을 완료 
    # 카메라, 스캔, 강화학습 서비스 응답등 여러 작업이 들어왔을 때 병렬로 처리하여 성능 저하를 방지
    # 노드가 종료 될 때까지 함수가 상황에 맞춰 실행하고 종료시 노드를 파괴하고 끔

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
    # 이 파일이 실행 될 경우 메인함수 호출
    main()