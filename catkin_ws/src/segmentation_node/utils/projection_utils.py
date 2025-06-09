import numpy as np
import yaml

def load_and_scale_intrinsics(yaml_path, orig_width, orig_height, new_width, new_height):
    with open(yaml_path, 'r') as f:
        cam_cfg = yaml.safe_load(f)
    K = np.array(cam_cfg['camera_matrix']['data']).reshape(3, 3)
    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    scale_x = new_width / orig_width
    scale_y = new_height / orig_height
    fx_new = fx * scale_x
    fy_new = fy * scale_y
    cx_new = cx * scale_x
    cy_new = cy * scale_y
    K_new = np.array([[fx_new, 0, cx_new], [0, fy_new, cy_new], [0, 0, 1]])
    return K_new

def project_mask_to_3d(ground_mask, depth, K):
    K = np.array(K).reshape(3, 3)
    K_inv = np.linalg.inv(K)

    # 1. Get (v, u) indices where ground_mask is 1
    v_idx, u_idx = np.where(ground_mask > 0)
    if len(v_idx) == 0:
        return []

    # 2. Get corresponding depth values
    z = depth[v_idx, u_idx]
    valid = z > 0
    u_idx = u_idx[valid]
    v_idx = v_idx[valid]
    z = z[valid]
    if len(z) == 0:
        return []

    # 3. Stack [u, v, 1] for all ground pixels
    ones = np.ones_like(u_idx)
    pixels = np.stack([u_idx, v_idx, ones], axis=1).T  # shape: (3, N)

    # 4. Apply K_inv and scale by depth
    rays = K_inv @ pixels  # shape: (3, N)
    points_3d = rays * z  # shape: (3, N)
    points_3d = points_3d.T  # shape: (N, 3)

    return points_3d.tolist()

def reproject_mask_rgb_to_depth(rgb_mask, depth, K_rgb, K_depth):
    H, W = depth.shape
    mask_on_depth = np.zeros((H, W), dtype=np.uint8)
    fx_d, fy_d, cx_d, cy_d = K_depth[0,0], K_depth[1,1], K_depth[0,2], K_depth[1,2]
    fx_rgb, fy_rgb, cx_rgb, cy_rgb = K_rgb[0,0], K_rgb[1,1], K_rgb[0,2], K_rgb[1,2]

    # Create a grid of (u, v) coordinates
    u = np.arange(W)
    v = np.arange(H)
    uu, vv = np.meshgrid(u, v)

    # Flatten for vectorized computation
    uu_flat = uu.flatten()
    vv_flat = vv.flatten()
    z_flat = depth.flatten()

    # Only consider valid depth
    valid = z_flat > 0
    uu_valid = uu_flat[valid]
    vv_valid = vv_flat[valid]
    z_valid = z_flat[valid]

    # Project depth pixels to 3D (depth camera frame)
    X = (uu_valid - cx_d) * z_valid / fx_d
    Y = (vv_valid - cy_d) * z_valid / fy_d
    Z = z_valid

    # Project 3D points into RGB image
    u_rgb = np.round(fx_rgb * X / Z + cx_rgb).astype(int)
    v_rgb = np.round(fy_rgb * Y / Z + cy_rgb).astype(int)

    # Only keep points that fall within the RGB image
    in_bounds = (
        (u_rgb >= 0) & (u_rgb < rgb_mask.shape[1]) &
        (v_rgb >= 0) & (v_rgb < rgb_mask.shape[0])
    )

    # Map mask values
    mask_on_depth_flat = np.zeros_like(z_flat, dtype=np.uint8)
    mask_on_depth_flat[valid] = 0  # default to 0
    # mask_on_depth_flat[valid][in_bounds] = rgb_mask[v_rgb[in_bounds], u_rgb[in_bounds]]
    valid_indices = np.where(valid)[0]
    selected_indices = valid_indices[in_bounds]
    mask_on_depth_flat[selected_indices] = rgb_mask[v_rgb[in_bounds], u_rgb[in_bounds]]

    # Reshape back to (H, W)
    mask_on_depth = mask_on_depth_flat.reshape(H, W)


    return mask_on_depth


def convert_to_ros_coords(points_3d):
    """Convert points from OpenCV to ROS coordinate convention."""
    return np.stack([
        points_3d[:, 2],     # z_cv → x_ros
        -points_3d[:, 0],    # -x_cv → y_ros
        -points_3d[:, 1],    # -y_cv → z_ros
    ], axis=1)
