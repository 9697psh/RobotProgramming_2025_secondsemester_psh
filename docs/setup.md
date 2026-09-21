# 환경 설정

## 개발 환경 (프로젝트 당시)

- Ubuntu 22.04.5 LTS
- ROS 2 Humble
- Gazebo Classic 11 (`gazebo_ros`)
- TurtleBot3 Waffle (`turtlebot3_gazebo`, `turtlebot3_navigation2`)
- Nav2, slam_toolbox
- YOLOv5 (`~/yolov5` 로컬 클론, `torch.hub.load(..., source='local')`)
- BehaviorTree.CPP + Groot (BT 편집)

## ~/.bashrc 설정

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
alias cb='colcon build --symlink-install'
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
source /usr/share/gazebo/setup.bash
source ~/ros2_project_ws/install/setup.bash
```

## 저장소에 포함하지 않은 파일

| 항목 | 위치(원본) | 이유 |
|---|---|---|
| Gazebo 모델 | `src/my_project_pkg/models/` (741MB) | GitHub 100MB/파일 제한. `models.zip`으로 별도 보관 |
| YOLOv5 커스텀 가중치 | `~/Desktop/best.pt` | 바이너리. 별도 보관 |
| YOLOv8 기본 가중치 | `src/yolo_detector/yolov8n.pt` | 공개 가중치, 재다운로드 가능 |
| 발표자료(영상·스크린샷) | `발표자료.tar.gz` | 용량 |

`models/`를 복원하려면 `models.zip`을 `src/my_project_pkg/models/`에 풀고 `GAZEBO_MODEL_PATH`에 추가.
