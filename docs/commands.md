# 실행 명령어 모음

프로젝트 진행 중 실제로 사용한 명령어를 정리한 것.

## 빌드

```bash
cd ~/ros2_project_ws
colcon build --symlink-install
# pip 사용자 패키지와 충돌 나서 빌드 오류 날 때:
PYTHONNOUSERSITE=1 colcon build --symlink-install
source install/setup.bash
```

## 맵 작성 (SLAM)

```bash
# 연습용 하우스 맵
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
# 프로젝트 공원 맵
ros2 launch my_project_pkg my_project.launch.py

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
ros2 run turtlebot3_teleop teleop_keyboard

ros2 run nav2_map_server map_saver_cli -f day_1
```

## 내비게이션

```bash
# turtlebot3 기본 런치
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_project_ws/day_7.yaml
# 커스텀 파라미터 + rviz + 순찰 노드까지 한 번에
ros2 launch clean_nav_pkg nav_launch.py use_sim_time:=True map:=$HOME/ros2_project_ws/day_7.yaml
```

## 순찰 / 스캔

```bash
# 순찰만
ros2 run my_nav2_tools nav2_patrol

# 순찰 + YOLO + 스캔 + JSON 기록 (요일 파일명 지정)
ros2 launch my_project_pkg patrol_and_scan.launch.py output_filename:=월요일.json

# 리포트 기반 우선순위 순찰
ros2 launch my_project_pkg priority_patrol.launch.py
```

## 객체 감지

```bash
ros2 run yolo_detector yolo_node                 # YOLOv5 커스텀 모델
ros2 run rqt_image_view rqt_image_view /yolo_image   # 감지 화면 확인
ros2 run yolo_detector person_counter            # 사람 수 → /person_count
ros2 run my_nav2_tools cylinder_classifier       # (초기) OpenCV 색상 기반 실린더 분류
```

## 리포트

```bash
python3 create_daily_report.py    # report/*.json → daily_trash_report.png + 최다 지점 출력
```

## Behavior Tree 편집 (Groot2)

```bash
sudo apt-get install ros-$ROS_DISTRO-groot
cd ~/Groot && ./build/Groot
```
`scan_bt.xml`을 Groot에서 열어 수정 후, `nav2_patrol.py`의 `bt_xml_path`가 가리키는 위치에 반영.
