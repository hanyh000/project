#!/usr/bin/env python3
import gc
import collections
import datetime
import json
import math
import os
import random
import sys
import time

import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
import tensorflow
from tensorflow.keras.layers import Dense, Input
from tensorflow.keras.losses import MeanSquaredError
from tensorflow.keras.models import Sequential
from tensorflow.keras.optimizers import Adam

from turtlebot3_msgs.srv import Dqn

# GPU 사용 안 함 설정
tensorflow.config.set_visible_devices([], 'GPU')

LOGGING = True
current_time = datetime.datetime.now().strftime('[%mm%dd-%H:%M]')

class DQNMetric(tensorflow.keras.metrics.Metric):
    def __init__(self, name='dqn_metric'):
        super(DQNMetric, self).__init__(name=name)
        self.loss = self.add_weight(name='loss', initializer='zeros')
        self.episode_step = self.add_weight(name='step', initializer='zeros')

    def update_state(self, y_true, y_pred=0, sample_weight=None):
        self.loss.assign_add(y_true)
        self.episode_step.assign_add(1)

    def result(self):
        return self.loss / self.episode_step

    def reset_states(self):
        self.loss.assign(0)
        self.episode_step.assign(0)

class DQNAgent(Node):
    def __init__(self, stage_num, max_training_episodes):
        super().__init__('dqn_agent')

        # === [모드 설정 공간] ===
        # 세 개 중 하나만 True로 설정하십시오.
        self.new_train_mode = False       # 아예 처음부터 학습
        self.continue_train_mode = False   # 저장된 모델 불러와서 이어서 학습
        self.load_mode = True            # 학습 없이 주행만 (Test)
        # =======================

        self.stage = int(stage_num)
        self.state_size = 26
        self.action_size = 7
        self.max_training_episodes = int(max_training_episodes)

        # 하이퍼파라미터
        self.discount_factor = 0.99
        self.learning_rate = 0.0007
        self.epsilon_min = 0.05
        self.epsilon_decay = 20000 * self.stage
        self.batch_size = 32
        self.update_target_after = 2000
        
        # 상태 변수 초기화
        self.epsilon = 1.0
        self.step_counter = 0
        self.best_score = -999999
        self.target_update_after_counter = 0
        self.replay_memory = collections.deque(maxlen=5000)
        self.min_replay_memory_size = 2000

        # 모델 생성
        self.model = self.create_qnetwork()
        self.target_model = self.create_qnetwork()

        # 경로 설정
        self.model_dir_path = '/home/dev/turtlebot3_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/saved_model'
        # 불러올 파일 (Continue/Load 모드용)
        self.load_model_path = os.path.join(self.model_dir_path, 'stage1_best_17.h5')
        self.load_json_path = os.path.join(self.model_dir_path, 'stage1_best_17.json')

        # [모드별 초기화 로직]
        self.init_by_mode()

        # ROS 통신 설정
        if LOGGING:
            log_dir = os.path.join(os.path.expanduser('~'), 'turtlebot3_dqn_logs', 'gradient_tape', 
                                   current_time + f'_dqn_stage{self.stage}')
            self.dqn_reward_writer = tensorflow.summary.create_file_writer(log_dir)
            self.dqn_reward_metric = DQNMetric()

        self.rl_agent_interface_client = self.create_client(Dqn, 'rl_agent_interface')
        self.make_environment_client = self.create_client(Empty, 'make_environment')
        self.reset_environment_client = self.create_client(Dqn, 'reset_environment')
        self.action_pub = self.create_publisher(Float32MultiArray, '/get_action', 10)

        # 프로세스 시작
        self.process()

    def init_by_mode(self):
        """설정된 모드 플래그에 따라 모델 로드 및 학습 여부 결정"""
        self.train_enabled = self.new_train_mode or self.continue_train_mode

        if self.new_train_mode:
            print(">>> MODE: NEW TRAINING START")
            self.epsilon = 1.0
            self.step_counter = 0
            self.update_target_model()

        elif self.continue_train_mode:
            print(f">>> MODE: CONTINUE TRAINING - Loading {self.load_model_path}")
            if os.path.exists(self.load_model_path):
                self.model.load_weights(self.load_model_path)
                if os.path.exists(self.load_json_path):
                    with open(self.load_json_path, 'r') as f:
                        param = json.load(f)
                        self.best_score = param.get('score', -999999)
                        self.epsilon = param.get('epsilon', 1.0)
                        self.step_counter = param.get('step', 0)
                self.update_target_model()
                print(f"Successfully Loaded. Epsilon: {self.epsilon:.2f}, Best Score: {self.best_score:.2f}")
            else:
                print("!! Warning: Model file not found. Starting from scratch !!")

        elif self.load_mode:
            print(f">>> MODE: LOAD (TEST) - Loading {self.load_model_path}")
            if os.path.exists(self.load_model_path):
                self.model.load_weights(self.load_model_path)
                self.epsilon = 0.0  # 추론 시에는 무조건 Greedy
            else:
                print("!! Error: No model to load for Test mode !!")
                sys.exit()

    def create_qnetwork(self):
        model = Sequential([
            Input(shape=(self.state_size,)),
            Dense(512, activation='relu'),
            Dense(256, activation='relu'),
            Dense(128, activation='relu'),
            Dense(self.action_size, activation='linear')
        ])
        model.compile(loss=MeanSquaredError(), optimizer=Adam(learning_rate=self.learning_rate))
        return model

    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())
        self.target_update_after_counter = 0

    def get_action(self, state):
        # 학습 모드일 때만 Epsilon-Greedy 적용
        if self.train_enabled:
            self.step_counter += 1
            self.epsilon = self.epsilon_min + (1.0 - self.epsilon_min) * math.exp(
                -1.0 * self.step_counter / self.epsilon_decay)
            
            if random.random() < self.epsilon:
                return random.randint(0, self.action_size - 1)
        
        # Test 모드거나 확률에 걸리지 않았을 때
        return numpy.argmax(self.model.predict(state, verbose=0))

    def process(self):
        self.env_make()
        time.sleep(1.0)

        for episode in range(1, self.max_training_episodes + 1):
            state = self.reset_environment()
            score = 0
            
            while True:
                action = int(self.get_action(state))
                next_state, reward, done = self.step(action)
                score += reward

                # 퍼블리싱 (모니터링용)
                msg = Float32MultiArray()
                msg.data = [float(action), float(score), float(reward)]
                self.action_pub.publish(msg)

                if self.train_enabled:
                    self.replay_memory.append((state, action, reward, next_state, done))
                    self.train_model(done)

                state = next_state

                if done:
                    # Best Model 자동 저장 로직 (학습 모드일 때만)
                    if self.train_enabled and score > self.best_score:
                        self.best_score = score
                        save_p = os.path.join(self.model_dir_path, f'stage{self.stage}_best.h5')
                        json_p = os.path.join(self.model_dir_path, f'stage{self.stage}_best.json')
                        self.model.save(save_p)
                        with open(json_p, 'w') as f:
                            json.dump({'epsilon': self.epsilon, 'step': self.step_counter, 'score': self.best_score}, f)
                        print(f"!!! NEW BEST: {score:.2f} Saved !!!")

                    if LOGGING:
                        self.dqn_reward_metric.update_state(score)
                        with self.dqn_reward_writer.as_default():
                            tensorflow.summary.scalar('dqn_reward', self.dqn_reward_metric.result(), step=episode)
                        self.dqn_reward_metric.reset_states()

                    print(f"Episode: {episode} | Score: {score:.2f} | Epsilon: {self.epsilon:.3f}")
                    break

    def train_model(self, terminal):
        if len(self.replay_memory) < self.min_replay_memory_size:
            return

        mini_batch = random.sample(self.replay_memory, self.batch_size)
        states = numpy.vstack([x[0] for x in mini_batch])
        next_states = numpy.vstack([x[3] for x in mini_batch])
        
        q_current = self.model.predict(states, verbose=0)
        q_next = self.target_model.predict(next_states, verbose=0)

        x_batch, y_batch = [], []

        for i, (s, a, r, ns, d) in enumerate(mini_batch):
            target = q_current[i]
            if d:
                target[a] = r
            else:
                target[a] = r + self.discount_factor * numpy.max(q_next[i])
            
            x_batch.append(s)
            y_batch.append(target)

        self.model.fit(numpy.vstack(x_batch), numpy.vstack(y_batch), batch_size=self.batch_size, verbose=0)
        
        self.target_update_after_counter += 1
        if self.target_update_after_counter > self.update_target_after and terminal:
            self.update_target_model()
            print("Target Model Updated")

        if terminal:
            tensorflow.keras.backend.clear_session()
            gc.collect()

    # --- Gazebo Service Calls ---
    def env_make(self):
        self.make_environment_client.wait_for_service()
        self.make_environment_client.call_async(Empty.Request())

    def reset_environment(self):
        self.reset_environment_client.wait_for_service()
        future = self.reset_environment_client.call_async(Dqn.Request())
        rclpy.spin_until_future_complete(self, future)
        return numpy.reshape(numpy.asarray(future.result().state), [1, self.state_size])

    def step(self, action):
        req = Dqn.Request()
        req.action = action
        self.rl_agent_interface_client.wait_for_service()
        future = self.rl_agent_interface_client.call_async(req)
        
        # 타임아웃 방지를 위한 스핀
        start_t = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if future.done(): break
            if time.time() - start_t > 5.0:
                return numpy.zeros([1, self.state_size]), -100.0, True

        res = future.result()
        ns = numpy.reshape(numpy.asarray(res.state), [1, self.state_size])
        return ns, res.reward, res.done

def main(args=None):
    rclpy.init(args=args)
    stage_num = sys.argv[1] if len(sys.argv) > 1 else '1'
    max_episodes = sys.argv[2] if len(sys.argv) > 2 else '1000'
    
    dqn_agent = DQNAgent(stage_num, max_episodes)
    try:
        rclpy.spin(dqn_agent)
    except KeyboardInterrupt:
        pass
    finally:
        dqn_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()