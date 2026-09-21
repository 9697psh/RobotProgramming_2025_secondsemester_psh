import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import torch
import sys
import warnings

# Suppress all FutureWarning messages
warnings.filterwarnings("ignore", category=FutureWarning)

# ✅ 내 컴퓨터 경로 유지
sys.path.append('/home/suhyeong/yolov5')


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.get_logger().info('🚀 YOLOv5 ROS node starting (Local Mode)')

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )

        # Publisher for annotated image
        self.image_publisher = self.create_publisher(
            Image,
            '/yolo_image',
            10
        )
        
        # ✅ 외부 노드엔 없지만, 내 object_scanner를 위해 유지
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/yolo_detections',
            10
        )

        self.bridge = CvBridge()

        # ✅ 내 컴퓨터 모델 경로 유지
        self.model = torch.hub.load(
            '/home/suhyeong/yolov5',
            'custom',
            path='/home/suhyeong/Desktop/best.pt',
            source='local',
            force_reload=True
        )

        # ✅ 외부 노드와 동일한 임계값 설정
        self.model.conf = 0.25
        self.model.iou = 0.5

        self.get_logger().info('✅ YOLOv5 model loaded')
        self.get_logger().info(f'📦 Model class names: {self.model.names}')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # BGR → RGB
        img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # YOLO inference
        results = self.model(img_rgb)

        if results.pred[0] is None or len(results.pred[0]) == 0:
            # self.get_logger().info('❌ No detections')
            annotated_frame = cv_image
            
            # 빈 결과라도 발행은 해야 스캐너가 멈추지 않음 (옵션)
            empty_det_array = Detection2DArray()
            empty_det_array.header = msg.header
            self.detection_publisher.publish(empty_det_array)

        else:
            h, w, _ = img_rgb.shape
            img_area = h * w
            filtered_dets = []

            # ✅ 외부 노드의 "라벨 스와핑 및 필터링 로직" 그대로 적용
            for det in results.pred[0]:
                x1, y1, x2, y2, conf, cls_id_original = det.tolist()
                cls_id_original = int(cls_id_original)
                cls_name_original = self.model.names[cls_id_original]

                box_area = (x2 - x1) * (y2 - y1)
                area_ratio = box_area / img_area

                # ❌ 화면 대부분 차지하는 허공 박스 제거 (동일)
                if area_ratio > 0.3:
                    continue

                # 🔎 필터링 및 ID 스와핑 (외부 노드 로직 복사)
                if (
                    (cls_name_original == 'can' and conf > 0.05) or
                    (cls_name_original == 'box' and conf > 0.05)
                ):
                    # 라벨 스와핑: can -> 0(box), box -> 1(can)
                    # 모델마다 ID 매핑이 다를 수 있으니 이름 기준으로 강제 할당
                    swapped_cls_id = cls_id_original
                    
                    if cls_name_original == 'can':
                        swapped_cls_id = 0  # 0번이 box라고 가정 (혹은 이름 'box'의 인덱스)
                        # 만약 model.names에서 인덱스를 확실히 하려면:
                        # swapped_cls_id = list(self.model.names.values()).index('box')
                    elif cls_name_original == 'box':
                        swapped_cls_id = 1  # 1번이 can이라고 가정
                        # swapped_cls_id = list(self.model.names.values()).index('can')

                    modified_det = det.clone()
                    modified_det[5] = float(swapped_cls_id)
                    filtered_dets.append(modified_det)

            if len(filtered_dets) == 0:
                # self.get_logger().debug('❌ No valid detections after filtering')
                annotated_frame = cv_image
                # 빈 배열 발행
                empty_det_array = Detection2DArray()
                empty_det_array.header = msg.header
                self.detection_publisher.publish(empty_det_array)

            else:
                # 🔥 Tensor 자체를 교체 (외부 노드 방식)
                results.pred[0] = torch.stack(filtered_dets)

                # 1. Detection2DArray 발행 (object_scanner용)
                detection_array = Detection2DArray()
                detection_array.header = msg.header

                for det in filtered_dets:
                    x1, y1, x2, y2, conf, cls_id = det.tolist()
                    cls_id = int(cls_id)
                    # 여기서는 이미 ID가 swap 되었으므로 그 이름을 가져옴
                    # 주의: self.model.names[0]이 'box'여야 의도대로 동작함
                    cls_name = self.model.names[cls_id]

                    detection = Detection2D()
                    detection.header = msg.header

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(cls_name)
                    hypothesis.hypothesis.score = conf
                    detection.results.append(hypothesis)

                    detection.bbox.center.position.x = float((x1 + x2) / 2)
                    detection.bbox.center.position.y = float((y1 + y2) / 2)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    detection_array.detections.append(detection)
                
                self.detection_publisher.publish(detection_array)

                # 2. 이미지 렌더링 (외부 노드 방식: 이미 tensor가 수정되었으므로 그대로 render)
                annotated_frame = results.render()[0]
                # self.get_logger().debug(f'✅ Final detections: {len(filtered_dets)}')

        # publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        out_msg.header = msg.header
        self.image_publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()