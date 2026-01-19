미래융합 교육원에서 했던 프로젝트 파일 모음 레포지터리

심화 프로젝트에서 파일을 적용하려면 turtlebot emanual 에서 머신러닝 관련 필요파일 다운 절차를 따라야 함
그 후 turtlebot3_machine_learning/turtlebot3_dqn/turtlebot3_dqn 에 있는 dqn_environment.py에 
심화 프로젝트 폴더에 있는 dqn_environment.py내용을 복사하거나 덮어씌우고 실행

dqn_yolo_test.py는 turtlebot3_machine_learning/turtlebot3_dqn 폴더에 있는 setup.py에 지정하고 
turtlebot3_machine_learning/turtlebot3_dqn/turtlebot3_dqn 폴더에 넣어서 실행

가상 맵에서 학습하려면 커스텀 맵 파일 dqn_agent.py, dqn_gazebo.py, dqn_environment.py를 실행
실행 순서는 맵 런치 파일 및 터틀봇 세팅 -> dqn_gazebo -> dqn_environment.py -> dqn_agent 순으로 실행 
agent, gazebo 수정 파일은 발표 후 현재 깃에 추가할 예정
