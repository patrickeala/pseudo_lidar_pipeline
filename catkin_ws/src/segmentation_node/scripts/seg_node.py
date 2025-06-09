#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import torch
import numpy as np
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))
from seg_constants import CITYSCAPES_COLORS, CITYSCAPES_CLASSES, GROUND_CLASSES
from fast_scnn import FastSCNN
from projection_utils import project_mask_to_3d, reproject_mask_rgb_to_depth, load_and_scale_intrinsics, convert_to_ros_coords
from img_utils import crop_top_to_div32, preprocess, decode_compressed_image, run_segmentation, prepare_depth_image, visualize_masks, get_color

import sensor_msgs.point_cloud2 as pc2

class SegNode:
    def __init__(self):
        rospy.init_node('seg_node')
        self.bridge = CvBridge()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Load and scale RGB camera intrinsics
        self.K_rgb = load_and_scale_intrinsics(
            os.path.join(os.path.dirname(__file__), '..', 'config', 'camera_info.yaml'),
            640, 360, 320, 240
        )
        # Load and scale depth camera intrinsics
        self.K_depth = load_and_scale_intrinsics(
            os.path.join(os.path.dirname(__file__), '..', 'config', 'depth_camera_info.yaml'),
            1024, 768, 320, 240
        )

        rospy.loginfo("Loading Fast-SCNN pretrained on Cityscapes from local weights...")
        # Make sure your weight are in the correct path
        # /root/pseudo_lidar_pipeline/catkin_ws/src/segmentation_node/utils/fast_scnn_repo/weights/fast_scnn_citys.pth

        weight_path = '/root/pseudo_lidar_pipeline/catkin_ws/src/segmentation_node/weights/fast_scnn_citys.pth'
        
        self.model = FastSCNN(num_classes=19).to(self.device)
        self.model.load_state_dict(torch.load(weight_path, map_location=self.device))
        self.model.eval()
        rospy.loginfo("✅ Fast-SCNN model loaded.")

        self.colors = np.array(CITYSCAPES_COLORS, dtype=np.uint8)

        self.sub = rospy.Subscriber(
            '/perception/camera_front_straight/rgb/image/compressed',
            CompressedImage, self.callback, queue_size=1
        )

        self.depth_image = None

        self.depth_sub = rospy.Subscriber(
            '/perception/camera_front_straight/depth/image',
            Image, self.depth_callback, queue_size=1
        )

        self.pc_pub = rospy.Publisher(
            '/pseudo_lidar/points', PointCloud2, queue_size=1
        )
        # Add publishers for binary and semantic masks
        self.ground_mask_pub = rospy.Publisher(
            '/pseudo_lidar/ground_mask', Image, queue_size=1
        )
        self.semantic_mask_pub = rospy.Publisher(
            '/pseudo_lidar/semantic_mask', Image, queue_size=1
        )

    def callback(self, msg):
        # 1. Decode and preprocess the incoming RGB image
        image = decode_compressed_image(msg)

        # 2. Run segmentation and get masks
        mask, ground_mask, ground_mask_full = run_segmentation(self.model, self.device, image)
        color_mask = get_color(mask, self.colors)

        # 3. Visualize masks for debugging (optional)
        # visualize_masks(image, mask, ground_mask_full, color_mask)

        # 4. Publish the ground and semantic masks
        self.publish_masks(ground_mask_full, color_mask)

        # 5. Wait for depth image if not yet received
        if self.depth_image is None:
            rospy.logwarn("Waiting for depth image...")
            return

        # 6. Prepare the depth image (convert to meters if needed)
        depth = prepare_depth_image(self.bridge, self.depth_image)

        # 7. Reproject the ground mask to depth image coordinates
        ground_mask_on_depth = reproject_mask_rgb_to_depth(ground_mask_full, depth, self.K_rgb, self.K_depth)

        # 8. Project ground mask to 3D points
        points_3d = project_mask_to_3d(ground_mask_on_depth, depth, self.K_depth)
        points_3d = np.array(points_3d)
        if points_3d.size == 0:
            return

        # 9. Convert to ROS coordinate convention and publish point cloud
        points_3d_ros = convert_to_ros_coords(points_3d)
        self.publish_pointcloud(points_3d_ros)

    def publish_masks(self, ground_mask_full, color_mask):
        """Publish the ground and semantic masks as ROS Image messages."""
        ground_mask_img = self.bridge.cv2_to_imgmsg((ground_mask_full * 255).astype(np.uint8), encoding="mono8")
        ground_mask_img.header.stamp = rospy.Time.now()
        ground_mask_img.header.frame_id = "camera_front_straight_link"
        self.ground_mask_pub.publish(ground_mask_img)
        semantic_mask_img = self.bridge.cv2_to_imgmsg(color_mask, encoding="bgr8")
        semantic_mask_img.header.stamp = rospy.Time.now()
        semantic_mask_img.header.frame_id = "camera_front_straight_link"
        self.semantic_mask_pub.publish(semantic_mask_img)

    def publish_pointcloud(self, points_3d, frame_id="camera_front_straight_link"):
        if len(points_3d) == 0:
            return
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id  # Use the correct frame
        pc_msg = pc2.create_cloud_xyz32(header, points_3d)
        self.pc_pub.publish(pc_msg)

    def depth_callback(self, msg):
        self.depth_image = msg
        # rospy.loginfo("Received depth image")
        # print("Depth image shape:", self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').shape)

if __name__ == '__main__':
    SegNode()
    rospy.spin()
