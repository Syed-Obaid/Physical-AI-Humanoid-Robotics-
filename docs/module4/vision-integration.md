---
id: vision-integration
title: "Chapter 4.2: Vision Integration"
sidebar_label: "4.2 Vision Integration"
sidebar_position: 2
description: Integrating computer vision models with ROS 2 for object detection and segmentation
keywords: [YOLO, Segment Anything, object detection, semantic segmentation, ROS 2 vision]
---

# Chapter 4.2: Vision Integration

## Learning Objectives

By the end of this chapter, you will be able to:

1. Integrate YOLOv8 for real-time object detection in ROS 2
2. Use Segment Anything Model (SAM) for zero-shot segmentation
3. Implement depth estimation with MiDaS or DepthAnything
4. Process camera streams and publish vision messages
5. Transform detected objects from image space to robot coordinates

## Prerequisites

### Required Knowledge
- Python programming (PyTorch basics)
- ROS 2 topics and messages
- Computer vision fundamentals (image coordinates, bounding boxes)
- Camera calibration concepts

### Previous Chapters
- [Chapter 4.1: Embodied AI Overview](./overview.md)
- [Chapter 1.3: Nodes and Topics](../module1/nodes-topics.md)

## Content

### ROS 2 Vision Messages

ROS 2 provides standardized message types for vision data:

#### sensor_msgs/Image
Raw image data from cameras.
```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Convert ROS Image to OpenCV
bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')

# Convert OpenCV to ROS Image
ros_image = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
```

#### vision_msgs/Detection2D
Single object detection with bounding box.
```python
from vision_msgs.msg import Detection2D, BoundingBox2D, ObjectHypothesisWithPose

detection = Detection2D()
detection.bbox.center.position.x = 320.0  # Image center
detection.bbox.center.position.y = 240.0
detection.bbox.size_x = 100.0  # Bounding box width
detection.bbox.size_y = 150.0  # Bounding box height

# Object class and confidence
hypothesis = ObjectHypothesisWithPose()
hypothesis.hypothesis.class_id = "cup"
hypothesis.hypothesis.score = 0.95
detection.results.append(hypothesis)
```

#### vision_msgs/Detection2DArray
Multiple detections in single frame.
```python
from vision_msgs.msg import Detection2DArray

detections = Detection2DArray()
detections.header.stamp = node.get_clock().now().to_msg()
detections.header.frame_id = "camera_frame"
detections.detections = [detection1, detection2, detection3]
```

### YOLOv8 Integration

**YOLOv8** (You Only Look Once v8) is a state-of-the-art object detection model.

#### Installation

```bash
pip install ultralytics opencv-python
```

#### Creating a YOLO ROS 2 Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Load YOLOv8 model
        self.model = YOLO('yolov8n.pt')  # 'n' = nano (fast), 's'=small, 'm'=medium, 'l'=large

        # ROS 2 interfaces
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        self.image_pub = self.create_publisher(
            Image,
            '/detections/image',
            10
        )

        self.bridge = CvBridge()
        self.get_logger().info('YOLO detector node started')

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO inference
        results = self.model(cv_image, conf=0.5)  # conf = confidence threshold

        # Create Detection2DArray message
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        # Process each detection
        for result in results:
            boxes = result.boxes
            for box in boxes:
                detection = Detection2D()

                # Bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                # Class and confidence
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(box.cls[0]))
                hypothesis.hypothesis.score = float(box.conf[0])
                detection.results.append(hypothesis)

                detections_msg.detections.append(detection)

        # Publish detections
        self.detection_pub.publish(detections_msg)

        # Draw and publish visualization
        annotated_frame = results[0].plot()  # Draw boxes on image
        image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.image_pub.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### YOLOv8 Model Variants

| Model       | Size (MB) | mAP@50-95 | Speed (ms) | Best For |
|-------------|-----------|-----------|------------|----------|
| YOLOv8n     | 6.2       | 37.3%     | 0.99       | Embedded devices |
| YOLOv8s     | 22        | 44.9%     | 1.21       | Balanced (default) |
| YOLOv8m     | 52        | 50.2%     | 2.1        | Higher accuracy |
| YOLOv8l     | 88        | 52.9%     | 3.0        | Desktop/GPU |
| YOLOv8x     | 136       | 53.9%     | 4.5        | Max accuracy |

**Recommendation**: Use YOLOv8s for humanoid robots (good speed/accuracy trade-off).

#### Custom Object Classes

Train YOLO on custom objects (e.g., household items):

```bash
# Install Roboflow for dataset management
pip install roboflow

# Download annotated dataset
from roboflow import Roboflow
rf = Roboflow(api_key="YOUR_API_KEY")
project = rf.workspace().project("household-objects")
dataset = project.version(1).download("yolov8")

# Train custom model
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.train(data='household-objects/data.yaml', epochs=50, imgsz=640)
```

### Segment Anything Model (SAM)

**SAM** performs zero-shot segmentation—generates pixel-perfect masks for any object without training.

#### Installation

```bash
pip install segment-anything-py
pip install git+https://github.com/facebookresearch/segment-anything.git
```

#### SAM ROS 2 Node

```python
import numpy as np
from segment_anything import sam_model_registry, SamPredictor

class SamSegmentationNode(Node):
    def __init__(self):
        super().__init__('sam_segmentation')

        # Load SAM model
        sam_checkpoint = "sam_vit_h_4b8939.pth"
        model_type = "vit_h"
        sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        self.predictor = SamPredictor(sam)

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.mask_pub = self.create_publisher(Image, '/segmentation/mask', 10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Set image for SAM
        self.predictor.set_image(cv_image)

        # Prompt SAM with bounding box (from YOLO detection)
        # Example: segment object at center of image
        h, w = cv_image.shape[:2]
        input_box = np.array([w//4, h//4, 3*w//4, 3*h//4])  # [x1, y1, x2, y2]

        # Get segmentation mask
        masks, scores, logits = self.predictor.predict(
            box=input_box,
            multimask_output=False
        )

        # Convert mask to image and publish
        mask_image = (masks[0] * 255).astype(np.uint8)
        mask_msg = self.bridge.cv2_to_imgmsg(mask_image, encoding='mono8')
        self.mask_pub.publish(mask_msg)
```

**Use Case**: Precise grasp planning—segment object to compute center of mass and grasp points.

### Depth Estimation

#### Using RGB-D Cameras (Realsense, Kinect)

Direct depth from sensor:
```python
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

def depth_callback(depth_msg):
    # Read depth at pixel (u, v)
    points = list(pc2.read_points(depth_msg, field_names=("x", "y", "z"), skip_nans=True))
    x, y, z = points[0]  # Depth in meters
```

#### Monocular Depth with MiDaS

Estimate depth from single RGB camera:

```bash
pip install timm
# Download MiDaS model
wget https://github.com/isl-org/MiDaS/releases/download/v3_1/dpt_beit_large_512.pt
```

```python
import torch
from torchvision.transforms import Compose

class DepthEstimationNode(Node):
    def __init__(self):
        super().__init__('depth_estimation')

        # Load MiDaS model
        self.model = torch.hub.load("intel-isl/MiDaS", "DPT_Large")
        self.model.eval()

        self.midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = self.midas_transforms.dpt_transform

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.depth_pub = self.create_publisher(Image, '/depth/image', 10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

        # Prepare input
        input_batch = self.transform(cv_image).unsqueeze(0)

        # Predict depth
        with torch.no_grad():
            prediction = self.model(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=cv_image.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        depth_map = prediction.cpu().numpy()

        # Normalize for visualization
        depth_map = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min()) * 255
        depth_msg = self.bridge.cv2_to_imgmsg(depth_map.astype(np.uint8), 'mono8')
        self.depth_pub.publish(depth_msg)
```

### Image to Robot Coordinate Transform

Convert object position from image coordinates (pixels) to robot base frame (meters).

#### Camera Intrinsics

```python
# From camera calibration
fx = 525.0  # Focal length x
fy = 525.0  # Focal length y
cx = 320.0  # Principal point x
cy = 240.0  # Principal point y

def pixel_to_camera(u, v, depth):
    """Convert pixel (u,v) and depth to camera frame (x,y,z)."""
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth
    return x, y, z
```

#### TF2 Transform to Robot Frame

```python
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped

# In your node __init__:
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)

def transform_to_base_frame(self, x, y, z):
    """Transform point from camera frame to robot base frame."""
    point_camera = PointStamped()
    point_camera.header.frame_id = "camera_link"
    point_camera.header.stamp = self.get_clock().now().to_msg()
    point_camera.point.x = float(x)
    point_camera.point.y = float(y)
    point_camera.point.z = float(z)

    try:
        # Transform to base_link
        point_base = self.tf_buffer.transform(point_camera, "base_link", timeout=Duration(seconds=1.0))
        return point_base.point.x, point_base.point.y, point_base.point.z
    except Exception as e:
        self.get_logger().error(f'Transform failed: {e}')
        return None
```

### Practical Example: Grasp Pipeline

```python
def grasp_object_at_pixel(self, u, v):
    """
    Complete pipeline: pixel → 3D position → grasp.
    """
    # 1. Get depth at pixel
    depth = self.get_depth_at_pixel(u, v)  # From depth camera or MiDaS

    # 2. Convert to camera coordinates
    x_cam, y_cam, z_cam = pixel_to_camera(u, v, depth)

    # 3. Transform to robot base frame
    x_base, y_base, z_base = self.transform_to_base_frame(x_cam, y_cam, z_cam)

    # 4. Plan grasp with MoveIt
    from moveit_msgs.msg import MoveGroupGoal
    goal = MoveGroupGoal()
    goal.request.group_name = "arm"
    goal.request.end_effector_link = "gripper_link"
    goal.request.pose_goal.position.x = x_base
    goal.request.pose_goal.position.y = y_base
    goal.request.pose_goal.position.z = z_base + 0.05  # Approach from above
    # ... (send goal to MoveIt action server)
```

## Summary

### Key Takeaways
- **ROS 2 vision messages**: `sensor_msgs/Image`, `vision_msgs/Detection2DArray` for standardized vision data
- **YOLOv8**: Real-time object detection (YOLOv8s recommended for humanoids)
- **SAM (Segment Anything)**: Zero-shot segmentation for precise object masks
- **Depth estimation**: RGB-D cameras (direct) or MiDaS/DepthAnything (monocular)
- **Coordinate transforms**: Use camera intrinsics + TF2 to convert image pixels to robot coordinates
- **Grasp pipeline**: Detection → depth → transform → MoveIt planning

### What's Next
In Chapter 4.3, you'll integrate large language models for natural language command parsing and task decomposition.

## Exercises

See [Module 4 Exercises](./exercises.md) - Exercise 4.1 covers YOLO integration and object detection.

## References

- Jocher, G., et al. (2023). Ultralytics YOLO. Retrieved from https://github.com/ultralytics/ultralytics
- Kirillov, A., et al. (2023). Segment Anything. *ICCV 2023*. https://arxiv.org/abs/2304.02643
- Ranftl, R., et al. (2021). Vision Transformers for Dense Prediction. *ICCV 2021*. https://arxiv.org/abs/2103.13413

---

**Word Count**: ~900 words
