import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Int32

class PersonCounterNode(Node):
    def __init__(self):
        super().__init__('person_counter_node')
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/yolo_detections',
            self.detection_callback,
            10)
        # 사람 수를 발행할 퍼블리셔 추가
        self.count_publisher = self.create_publisher(Int32, '/person_count', 10)
        self.get_logger().info('Person counter node has been started.')

    def detection_callback(self, msg):
        person_count = 0
        for detection in msg.detections:
            # 각 detection의 첫 번째 hypothesis의 class_id를 확인
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id
                if class_id == 'person':
                    person_count += 1
        
        if person_count > 0:
            self.get_logger().info(f'Person detected! Total: {person_count} people.')

        # 사람 수를 Int32 메시지로 만들어 발행
        count_msg = Int32()
        count_msg.data = person_count
        self.count_publisher.publish(count_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonCounterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
