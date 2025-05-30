# 3D-Reconstruction-Pipeline


## Overview

This project implements a complete Structure from Motion pipeline that takes a sequence of 2D images and produces a 3D point cloud reconstruction. The implementation follows industry-standard computer vision practices and demonstrates proficiency in 3D reconstruction algorithms, feature matching, and pose estimation.

## Features

- **Feature Detection and Matching**: Utilizes SIFT feature detector with FLANN-based matcher for robust keypoint correspondence
- **Camera Pose Estimation**: Implements essential matrix decomposition and PnP RANSAC for accurate camera pose recovery
- **3D Point Triangulation**: Reconstructs 3D points from stereo correspondences using triangulation
- **Incremental Registration**: Progressively adds new images to expand the 3D reconstruction
- **Point Cloud Export**: Outputs results in standard PLY format for visualization

## Technical Implementation

### Core Components

1. **Feature Detection and Matching**
   - SIFT detector for scale-invariant feature extraction
   - Ratio test filtering (threshold: 0.35) to eliminate poor matches
   - FLANN-based matcher for efficient descriptor matching

2. **Pose Estimation**
   - Essential matrix estimation using RANSAC for outlier rejection
   - Camera pose recovery from essential matrix decomposition
   - PnP RANSAC for subsequent image registration

3. **3D Reconstruction**
   - Stereo triangulation for initial point cloud generation
   - Incremental point cloud expansion through sequential image processing
   - Global point tracking with descriptor management

## Requirements

- Python 3.7+
- OpenCV (cv2)
- NumPy

## Installation

```bash
pip install opencv-python numpy
```

## Usage

### Input Requirements

1. **Images**: Sequential images in `images/` directory named as `0000.jpg`, `0001.jpg`, etc.
2. **Camera Calibration**: `K.txt` file containing the 3x3 camera intrinsic matrix
3. **Viewer**: `showcloud.py` script for point cloud visualization (if available)

### Running the Pipeline

```bash
python main.py
```

### Output

The pipeline generates `mycloud.ply` containing the reconstructed 3D point cloud.

### Visualization

If the viewer script is available:
```bash
python showcloud.py mycloud.ply
```

## Algorithm Workflow

1. **Initialization**: Load camera calibration matrix and sequential images
2. **Initial Pair Processing**: 
   - Detect and match features between first two images
   - Estimate relative camera pose using essential matrix
   - Triangulate initial 3D points
3. **Incremental Registration**:
   - For each subsequent image, match features to existing 3D points
   - Estimate camera pose using PnP RANSAC
   - Triangulate new 3D points between consecutive images
   - Update global point cloud and feature database
4. **Export**: Save final point cloud in PLY format

## Technical Specifications

- **Feature Detector**: SIFT with default parameters
- **Matching Strategy**: FLANN-based matcher with ratio test (0.35 threshold)
- **Pose Estimation**: Essential matrix with RANSAC (confidence: 0.999, threshold: 1.0)
- **Triangulation**: OpenCV triangulatePoints with projection matrices
- **File Format**: ASCII PLY for cross-platform compatibility

## Project Structure

```
3D-Reconstruction-Pipeline/
├── main.py                 # Main SfM pipeline implementation
├── K.txt                   # Camera intrinsic matrix
├── images/                 # Input image sequence
│   ├── 0000.jpg
│   ├── 0001.jpg
│   └── ...
├── mycloud.ply            # Output point cloud
├── showcloud.py           # Point cloud viewer (if provided)
└── README.md              # This file
```

## Key Functions

- `feature_detection()`: SIFT-based feature detection and matching
- `estimate_camera_pose()`: Essential matrix-based pose estimation
- `triangulate_points()`: 3D point reconstruction from stereo pairs
- `incremental_registration()`: Sequential image processing and registration
- `write_ply()`: Point cloud export in PLY format

## Performance Considerations

- Sequential processing approach optimized for spatially ordered images
- RANSAC-based outlier rejection for robust pose estimation
- Efficient descriptor matching using FLANN indexing
- Memory-efficient point cloud accumulation

## Applications

This implementation demonstrates core concepts used in:
- Photogrammetry and 3D mapping
- Autonomous vehicle navigation
- Augmented reality applications
- Cultural heritage digitization
- Industrial inspection and measurement

## License

This project is part of academic coursework in Computer Vision and Image Processing.
