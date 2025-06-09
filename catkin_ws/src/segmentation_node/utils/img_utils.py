# img utils

import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
from seg_constants import GROUND_CLASSES

def crop_top_to_div32(image):
    """
    Crops the top rows so that the output height is the largest possible multiple of 32.
    image: np.ndarray of shape (H, W, C)
    Returns: cropped_image, crop_rows
    """
    h, w, c = image.shape
    new_h = (h // 32) * 32  # largest height divisible by 32 that's <= h
    crop_rows = h - new_h
    cropped = image[crop_rows:, :, :]  # remove crop_rows from top
    return cropped, crop_rows

def preprocess(image):
    """
    Preprocess the image for Fast-SCNN: crop, convert to tensor, normalize.
    """
    cropped, crop_rows = crop_top_to_div32(image)
    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225])
    ])
    return transform(cropped)

def decode_compressed_image(msg):
    """
    Decode a ROS CompressedImage message to a cv2 image.
    """
    np_arr = np.frombuffer(msg.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return image

def run_segmentation(model, device, image):
    """
    Run Fast-SCNN segmentation and prepare masks.
    Returns: mask, ground_mask, ground_mask_full, color_mask
    """
    from seg_constants import GROUND_CLASSES  # for standalone use
    cropped, crop_rows = crop_top_to_div32(image)
    input_tensor = preprocess(image).unsqueeze(0).to(device)
    with torch.no_grad():
        output = model(input_tensor)[0]  # shape: (1, 19, H_cropped, W)
        output = output.squeeze(0)            # shape: (19, H_cropped, W)
        mask = output.argmax(0).cpu().numpy() # shape: (H_cropped, W)
    ground_mask = np.isin(mask, GROUND_CLASSES).astype(np.uint8)  # 1 for ground, 0 for others
    # Create a full-size ground mask for visualization
    H, W = image.shape[:2]
    new_h = mask.shape[0]
    crop_rows = H - new_h
    ground_mask_full = np.zeros((H, W), dtype=np.uint8)
    ground_mask_full[crop_rows:, :] = ground_mask
    return mask, ground_mask, ground_mask_full

def prepare_depth_image(bridge, depth_msg):
    """
    Convert a ROS Image message to a cv2 depth image in meters.
    """
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    if depth.max() > 100:  # likely in mm
        depth = depth / 1000.0  # convert to meters
    return depth

def visualize_masks(image, mask, ground_mask_full, color_mask):
    """
    Visualize the RGB image, segmentation mask, and ground mask using OpenCV windows.
    """
    vis_rgb = cv2.resize(image, (image.shape[1]*2, image.shape[0]*2), interpolation=cv2.INTER_NEAREST)
    vis_mask = cv2.resize(color_mask, (color_mask.shape[1]*2, color_mask.shape[0]*2), interpolation=cv2.INTER_NEAREST)
    vis_ground = cv2.resize(ground_mask_full * 255, (ground_mask_full.shape[1]*2, ground_mask_full.shape[0]*2), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("RGB Image", vis_rgb)
    cv2.imshow("Segmentation Mask", vis_mask)
    cv2.imshow("Ground Mask", vis_ground)  # White = ground, black = not ground
    cv2.waitKey(1)

def get_color(mask, colors):
    """
    Generate a color mask from a class mask and a color palette.
    Args:
        mask: 2D numpy array of class indices (H, W)
        colors: (N, 3) array or list of RGB tuples
    Returns:
        color_mask: (H, W, 3) uint8 color image
    """
    h, w = mask.shape
    color_mask = np.zeros((h, w, 3), dtype=np.uint8)
    for idx, color in enumerate(colors):
        color_mask[mask == idx] = color
    return color_mask