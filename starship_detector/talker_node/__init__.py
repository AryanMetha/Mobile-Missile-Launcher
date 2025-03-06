import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import cv2
import numpy as np

# Path to the YOLO model
MODEL_PATH = "/path/to/your_model.pt"
CONFIDENCE_THRESHOLD = 0.5  # Adjust confidence threshold as needed
NMS_THRESHOLD = 0.4  # removes overlapping detections

class YOLOBoundingBoxNode(Node):
    def __init__(self):
        super().__init__('yolo_bounding_box_node')
        
        # ROS 2 Subscriptions & Publishers
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10) #subscribes to Realsense image class
        
        self.publisher = self.create_publisher(
            Detection2DArray, '/bounding_boxes', 10) #Publisher

        self.br = CvBridge() #convert ros2 images into OpenCV 

        # Load YOLO Model using Opencv DNN module
        self.net = cv2.dnn.readNet(MODEL_PATH)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)#loads yolo model weights using DNN's module
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)  

        # Get YOLO layer names
        layer_names = self.net.getLayerNames()
        self.output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

        self.get_logger().info("Bounding Box Node has started.")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.br.imgmsg_to_cv2(msg, "rgb8")

        # Resize image to 640x640 for YOLO model
        blob = cv2.dnn.blobFromImage(cv_image, 1/255.0, (640, 640), swapRB=True, crop=False)
        self.net.setInput(blob)

        # Run YOLO inference
        outputs = self.net.forward(self.output_layers)
        boxes = self.process_detections(outputs, cv_image.shape)

        # Publish bounding boxes
        detection_array = Detection2DArray()#creates a detection2D array
        for (x, y, w, h, cls) in boxes:
            detection = Detection2D()#converts this detection array into ros2 format 
            detection.bbox.center.x = x + w / 2
            detection.bbox.center.y = y + h / 2
            detection.bbox.size_x = w
            detection.bbox.size_y = h
            detection_array.detections.append(detection)

        self.publisher.publish(detection_array) #publishes bounding boxes

    def process_detections(self, outputs, image_shape):
        height, width, _ = image_shape
        boxes = []
        confidences = []
        class_ids = []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > CONFIDENCE_THRESHOLD:
                    center_x, center_y, w, h = (detection[:4] * np.array([width, height, width, height])).astype("int")
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h, class_id])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply Non-Maximum Suppression
        indices = cv2.dnn.NMSBoxes(
            [b[:4] for b in boxes], confidences, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)

        return [boxes[i] for i in indices.flatten()] if len(indices) > 0 else []

def main(args=None):
    rclpy.init(args=args)
    node = YOLOBoundingBoxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
