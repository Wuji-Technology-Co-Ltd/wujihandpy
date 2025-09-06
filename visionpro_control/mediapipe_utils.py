#!/usr/bin/env python3
"""
Minimal utility to convert Vision Pro hand matrices to MediaPipe format
"""

import numpy as np
from enum import Enum

class RetargetingConfig(Enum):
    DEX_LEFT = "/home/uji/Robben_ws/wujihandpy/config/wujihand_left_dexpilot.yaml"
    DEX_RIGHT = "/home/uji/Robben_ws/wujihandpy/config/wujihand_right_dexpilot.yaml"
    VECTOR_LEFT = "/home/uji/Robben_ws/wujihandpy/config/wujihand_left_vector.yaml"
    VECTOR_RIGHT = "/home/uji/Robben_ws/wujihandpy/config/wujihand_right_vector.yaml"

# Coordinate transformation matrices for MANO hand model
OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ]
)

OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0],
    ]
)

def convert_hand_matrices_to_mediapipe(hand_matrices):
    """
    Convert Vision Pro hand matrices (25x4x4) to MediaPipe format (21x3)
    
    Args:
        hand_matrices: numpy array of shape (25, 4, 4) - transformation matrices
        
    Returns:
        mediapipe_pose: numpy array of shape (21, 3) - MediaPipe landmarks
    """
    
    # Initialize output array
    mediapipe_pose = np.zeros((21, 3))
    
    # Vision Pro joint indices to MediaPipe landmark indices mapping
    # Based on the observed data structure from your pickle file
    joint_mapping = [
        0,   # wrist (we'll use the first matrix as wrist reference)
        1,   # thumb_cmc
        2,   # thumb_mcp
        3,   # thumb_ip
        4,   # thumb_tip
        6,   # index_mcp
        7,   # index_pip
        8,   # index_dip
        9,   # index_tip
        11,  # middle_mcp
        12,  # middle_pip
        13,  # middle_dip
        14,  # middle_tip
        16,  # ring_mcp
        17,  # ring_pip
        18,  # ring_dip
        19,  # ring_tip
        21,  # pinky_mcp
        22,  # pinky_pip
        23,  # pinky_dip
        24,  # pinky_tip
    ]
    
    # Extract positions from transformation matrices
    for mp_idx, vp_joint_idx in enumerate(joint_mapping):
        # Extract translation from 4x4 matrix (last column, first 3 rows)
        position = hand_matrices[vp_joint_idx][:3, 3]
        mediapipe_pose[mp_idx] = position
    
    #print("converted to mediapipe!!!")
    return mediapipe_pose


def estimate_frame_from_hand_points(keypoint_3d_array: np.ndarray) -> np.ndarray:
    """
    Compute the 3D coordinate frame (orientation only) from detected 3d key points
    :param keypoint_3d_array: keypoint3 detected from hand detector. Shape: (21, 3)
    :return: the coordinate frame of wrist in MANO convention
    """
    assert keypoint_3d_array.shape == (21, 3)
    points = keypoint_3d_array[[0, 5, 9], :]

    # Compute vector from palm to the first joint of middle finger
    x_vector = points[0] - points[2]

    # Normal fitting with SVD
    points = points - np.mean(points, axis=0, keepdims=True)
    u, s, v = np.linalg.svd(points)

    normal = v[2, :]

    # Gram–Schmidt Orthonormalize
    x = x_vector - np.sum(x_vector * normal) * normal
    x = x / np.linalg.norm(x)
    z = np.cross(x, normal)

    # We assume that the vector from pinky to index is similar the z axis in MANO convention
    if np.sum(z * (points[1] - points[2])) < 0:
        normal *= -1
        z *= -1
    frame = np.stack([x, normal, z], axis=1)
    return frame


def apply_mediapipe_transformations(keypoint_3d_array: np.ndarray, hand_type: str = "Right") -> np.ndarray:
    """
    Apply the same coordinate transformations as MediaPipe data processing.
    
    Args:
        keypoint_3d_array: numpy array of shape (21, 3) - hand landmarks
        hand_type: "Right" or "Left" - determines coordinate system
        
    Returns:
        transformed_joint_pos: numpy array of shape (21, 3) - transformed landmarks
    """
    # Center at wrist (make wrist the origin)
    keypoint_3d_array = keypoint_3d_array - keypoint_3d_array[0:1, :]
    
    # Estimate coordinate frame from hand geometry
    mediapipe_wrist_rot = estimate_frame_from_hand_points(keypoint_3d_array)
    
    # Apply MANO coordinate system transformation
    operator2mano = OPERATOR2MANO_RIGHT if hand_type == "Right" else OPERATOR2MANO_LEFT
    joint_pos = keypoint_3d_array @ mediapipe_wrist_rot @ operator2mano
    
    return joint_pos


def convert_vision_pro_to_mediapipe_format(hand_matrices, hand_type: str = "Right"):
    """
    Convert Vision Pro hand matrices to MediaPipe format with proper coordinate transformations.
    
    Args:
        hand_matrices: numpy array of shape (25, 4, 4) - Vision Pro transformation matrices
        hand_type: "Right" or "Left" - determines coordinate system
        
    Returns:
        joint_pos: numpy array of shape (21, 3) - transformed landmarks in MediaPipe format
    """
    # Convert Vision Pro matrices to MediaPipe format
    keypoint_3d_array = convert_hand_matrices_to_mediapipe(hand_matrices)
    
    # Apply the same transformations as MediaPipe data
    joint_pos = apply_mediapipe_transformations(keypoint_3d_array, hand_type)
    
    return joint_pos



if __name__ == "__main__":
    pass
