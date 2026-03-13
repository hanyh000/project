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
import gc

import threading
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
from tensorflow.keras.layers import Dense
from tensorflow.keras.layers import Input
from tensorflow.keras.losses import MeanSquaredError
from tensorflow.keras.models import load_model
from tensorflow.keras.models import Sequential
from tensorflow.keras.optimizers import Adam

from turtlebot3_dqn.db.db_helper import DB, DB_CONFIG

from turtlebot3_msgs.srv import Dqn


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

        self.db = DB(**DB_CONFIG)


        self.stage = int(stage_num)
        self.train_mode = True
        self.state_size = 29
        self.action_size = 5
        self.max_training_episodes = int(max_training_episodes)

        self.done = False
        self.succeed = False
        self.fail = False

        self.discount_factor = 0.99
        self.learning_rate = 0.0007
        self.epsilon = 1.0
        self.step_counter = 0
        self.epsilon_decay = 10000 * self.stage
        self.epsilon_min = 0.05
        self.batch_size = 32
        self.best_score = -999999

        self.replay_memory = collections.deque(maxlen=1000000)
        self.min_replay_memory_size = 2000

        self.model = self.create_qnetwork()
        self.target_model = self.create_qnetwork()
        self.update_target_model()
        self.update_target_after = 2000
        self.target_update_after_counter = 0

        self.load_model = True
        self.load_episode = 0

        self.model_dir_path = '/home/dev/turtlebot3_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/saved_model'
        self.model_path = os.path.join(self.model_dir_path, '17.h5')
        self.json_path = os.path.join(self.model_dir_path, '17.json')

        if self.load_model and os.path.exists(self.model_path):
            print(f"--- 모델 로드 시작: {self.model_path} ---")
            try:
                self.model.load_weights(self.model_path)
                self.target_model.set_weights(self.model.get_weights())
                if os.path.exists(self.json_path):
                    with open(self.json_path) as outfile:
                        param = json.load(outfile)
                        self.best_score = param.get('score', -999999)
                        loaded_epsilon = param.get('epsilon', 1.0)
                        self.epsilon = min(loaded_epsilon + 0.2, 1.0)
                        denom = (1.0 - self.epsilon_min)
                        numer = (self.epsilon - self.epsilon_min)
                        if numer <= 0: self.step_counter = 50000
                        else: self.step_counter = int(-self.epsilon_decay * math.log(numer / denom))
                print(f"--- 로드 완료! Best Score: {self.best_score:.2f} ---")
            except Exception as e: print(f"--- 로딩 에러: {e} ---")

        if LOGGING:
            tensorboard_file_name = current_time + ' dqn_stage' + str(self.stage) + '_reward'
            home_dir = os.path.expanduser('~')
            dqn_reward_log_dir = os.path.join(
                home_dir, 'turtlebot3_dqn_logs', 'gradient_tape', tensorboard_file_name
            )
            self.dqn_reward_writer = tensorflow.summary.create_file_writer(dqn_reward_log_dir)
            self.dqn_reward_metric = DQNMetric()

        self.rl_agent_interface_client = self.create_client(Dqn, 'rl_agent_interface')
        self.make_environment_client = self.create_client(Empty, 'make_environment')
        self.reset_environment_client = self.create_client(Dqn, 'reset_environment')

        self.action_pub = self.create_publisher(Float32MultiArray, '/get_action', 10)
        self.result_pub = self.create_publisher(Float32MultiArray, 'result', 10)

        #self.process()

    def process(self):

        run_id = self.db.insert_run() 

        self.env_make()
        time.sleep(0.05)

        episode_num = self.load_episode

        for episode in range(self.load_episode + 1, self.max_training_episodes + 1):
            state = self.reset_environment()
            episode_num += 1
            local_step = 0
            score = 0
            sum_max_q = 0.0

            time.sleep(0.05)

            while True:
                local_step += 1

                q_values = self.model.predict(state, verbose=0)
                sum_max_q += float(numpy.max(q_values))

                action = int(self.get_action(state))
                next_state, reward, done = self.step(action)
                score += reward

                msg = Float32MultiArray()
                msg.data = [float(action), float(score), float(reward)]
                self.action_pub.publish(msg)

                if self.train_mode:
                    self.append_sample((state, action, reward, next_state, done))
                    self.train_model(done)

                state = next_state

                if done:
                # [Best Model 저장 로직]
                    if score > self.best_score:
                        self.best_score = score
                        # 파일명을 _best로 고정하여 다음 프로세스에서 찾기 쉽게 함
                        save_path = os.path.join(self.model_dir_path, f'stage{self.stage}_best.h5')
                        json_save_path = os.path.join(self.model_dir_path, f'stage{self.stage}_best.json')
                        
                        self.model.save(save_path)
                        # 다음 학습을 위해 현재의 epsilon과 score를 함께 저장
                        param_dict = {
                            'epsilon': self.epsilon, 
                            'step': self.step_counter, 
                            'score': self.best_score
                        }
                        with open(json_save_path, 'w') as outfile:
                            json.dump(param_dict, outfile)
                        print(f"!!! NEW BEST 경신 및 저장: {score:.2f} !!!")

                    if LOGGING:
                        self.dqn_reward_metric.update_state(score)
                        with self.dqn_reward_writer.as_default():
                            tensorflow.summary.scalar(
                                'dqn_reward', self.dqn_reward_metric.result(), step=episode_num
                            )
                        self.dqn_reward_metric.reset_states()

                    print(
                        'Episode:', episode,
                        'score:', score,
                        'memory length:', len(self.replay_memory),
                        'epsilon:', self.epsilon)

                    episode_info = {
                        'run_id' : run_id,
                        'episode_id' : episode,
                        'score' : score,
                        'memory_length' : len(self.replay_memory),
                        'epsilon' : self.epsilon,
                        'step_count' : local_step
                    }

                    if self.db.insert_episode(episode_info):
                        print('insert_episode success')
                    else :
                        print('insert_episode fail')

                    param_keys = ['epsilon', 'step']
                    param_values = [self.epsilon, self.step_counter]
                    param_dictionary = dict(zip(param_keys, param_values))
                    break

                time.sleep(0.05)

            if self.train_mode:
                if episode % 100 == 0:
                    self.model_path = os.path.join(
                        self.model_dir_path,
                        'stage' + str(self.stage) + '_episode' + str(episode) + '.h5')
                    self.model.save(self.model_path)
                    with open(
                        os.path.join(
                            self.model_dir_path,
                            'stage' + str(self.stage) + '_episode' + str(episode) + '.json'
                        ),
                        'w'
                    ) as outfile:
                        json.dump(param_dictionary, outfile)

                    model_id = 'stage' + str(self.stage) + '_episode' + str(episode)

                    model_info = {
                        'model_id' : model_id,
                        'model_ext' : 'h5',
                        'model_file_path' : self.model_dir_path,
                        'run_id': run_id,
                        'episode_id': episode
                    }
                    if self.db.insert_model(model_info):
                        print('model h5 insert success')

                    model_info = {
                        'model_id' : model_id,
                        'model_ext' : 'json',
                        'model_file_path' : self.model_dir_path,
                        'run_id': run_id,
                        'episode_id': episode
                    }
                    if self.db.insert_model(model_info):
                        print('model json insert success')

    def env_make(self):
        while not self.make_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'Environment make client failed to connect to the server, try again ...'
            )

        self.make_environment_client.call_async(Empty.Request())

    def reset_environment(self):
        if not self.reset_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Reset environment client failed to connect!')
            return numpy.zeros([1, self.state_size])

        request = Dqn.Request()
        result = self.reset_environment_client.call(request)

        if result is not None:
            state = numpy.reshape(numpy.asarray(result.state), [1, self.state_size])
            return state
        else:
            self.get_logger().error('Reset Service Failed')
            return numpy.zeros([1, self.state_size])

    def get_action(self, state):
        if self.train_mode:
            self.step_counter += 1
            self.epsilon = self.epsilon_min + (1.0 - self.epsilon_min) * math.exp(
                -1.0 * self.step_counter / self.epsilon_decay)
            lucky = random.random()
            if lucky > (1 - self.epsilon):
                result = random.randint(0, self.action_size - 1)
            else:
                result = numpy.argmax(self.model.predict(state, verbose=0))
        else:
            result = numpy.argmax(self.model.predict(state, verbose=0))

        return result

    def step(self, action):
        req = Dqn.Request()
        req.action = action

        if not self.rl_agent_interface_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('rl_agent interface service not available!')
            return numpy.zeros([1, self.state_size]), -100.0, True

        result = self.rl_agent_interface_client.call(req)

        if result is not None:
            next_state = numpy.reshape(numpy.asarray(result.state), [1, self.state_size])
            return next_state, result.reward, result.done
        else:
            self.get_logger().error('Step Service Failed')
            return numpy.zeros([1, self.state_size]), -100.0, True

    def create_qnetwork(self):
        model = Sequential()
        model.add(Input(shape=(self.state_size,)))
        model.add(Dense(512, activation='relu'))
        model.add(Dense(256, activation='relu'))
        model.add(Dense(128, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss=MeanSquaredError(), optimizer=Adam(learning_rate=self.learning_rate))
        model.summary()

        return model

    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())
        self.target_update_after_counter = 0
        print('*Target model updated*')

    def append_sample(self, transition):
        self.replay_memory.append(transition)

    def train_model(self, terminal):
        if len(self.replay_memory) < self.min_replay_memory_size:
            return
        data_in_mini_batch = random.sample(self.replay_memory, self.batch_size)

        current_states = numpy.array([transition[0] for transition in data_in_mini_batch])
        current_states = current_states.squeeze()
        current_qvalues_list = self.model.predict(current_states, verbose=0)

        next_states = numpy.array([transition[3] for transition in data_in_mini_batch])
        next_states = next_states.squeeze()
        next_qvalues_list = self.target_model.predict(next_states, verbose=0)

        x_train = []
        y_train = []

        for index, (current_state, action, reward, _, done) in enumerate(data_in_mini_batch):
            current_q_values = current_qvalues_list[index]

            if not done:
                future_reward = numpy.max(next_qvalues_list[index])
                desired_q = reward + self.discount_factor * future_reward
            else:
                desired_q = reward

            current_q_values[action] = desired_q
            x_train.append(current_state)
            y_train.append(current_q_values)

        x_train = numpy.array(x_train)
        y_train = numpy.array(y_train)
        x_train = numpy.reshape(x_train, [len(data_in_mini_batch), self.state_size])
        y_train = numpy.reshape(y_train, [len(data_in_mini_batch), self.action_size])

        self.model.fit(
            tensorflow.convert_to_tensor(x_train, tensorflow.float32),
            tensorflow.convert_to_tensor(y_train, tensorflow.float32),
            batch_size=self.batch_size, verbose=0
        )
        self.target_update_after_counter += 1

        if self.target_update_after_counter > self.update_target_after and terminal:
            self.update_target_model()
        if terminal:
        # 1. 텐서플로우 내부 세션의 임시 그래프/변수 정리
            tensorflow.keras.backend.clear_session()
        # 2. 파이썬의 가비지 컬렉터 강제 구동
            gc.collect()


def main(args=None):
    if args is None:
        args = sys.argv
    stage_num = args[1] if len(args) > 1 else '1'
    max_training_episodes = args[2] if len(args) > 2 else '1000'
    rclpy.init(args=args)


    dqn_agent = DQNAgent(stage_num, max_training_episodes)

    train_thread = threading.Thread(target=dqn_agent.process)
    train_thread.daemon = True
    train_thread.start()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(dqn_agent)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        dqn_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()