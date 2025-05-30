import numpy as np
import cv2 as cv

def feature_detection(image_one: np.ndarray, image_two: np.ndarray):
    sift = cv.SIFT_create()
    keypoints_one, descriptors_one = sift.detectAndCompute(image_one, None)
    keypoints_two, descriptors_two = sift.detectAndCompute(image_two, None)

    # Create FLANN matcher parameters
    index_params = dict(algorithm=0, trees=5)
    search_params = dict(checks=50)
    flann = cv.FlannBasedMatcher(indexParams=index_params, searchParams=search_params)

    # kNN matching (with k=2)
    matches = flann.knnMatch(descriptors_one, descriptors_two, k=2)
    good_matches = []
    for m, n in matches:
        if m.distance < 0.35 * n.distance:
            good_matches.append(m)

    # Extract 2D point coordinates for the matched keypoints
    src_pts = np.array([keypoints_one[m.queryIdx].pt for m in good_matches], dtype=np.float32)
    dst_pts = np.array([keypoints_two[m.trainIdx].pt for m in good_matches], dtype=np.float32)

    # Also extract the descriptors from image two corresponding to the good matches
    good_descriptors = np.array([descriptors_two[m.trainIdx] for m in good_matches])

    return src_pts, dst_pts, good_descriptors, keypoints_two

def estimate_camera_pose(src_pts: np.ndarray, dst_pts: np.ndarray, K: np.ndarray):
    E, mask = cv.findEssentialMat(src_pts, dst_pts, K,
                                  method=cv.RANSAC, prob=0.999, threshold=1.0)
    _, R, t, mask_pose = cv.recoverPose(E, src_pts, dst_pts, K, mask=mask)
    return R, t

def triangulate_points(K: np.ndarray, src_pts: np.ndarray, dst_pts: np.ndarray,
                       R=None, t=None):
    I = np.eye(3)
    zero = np.zeros((3, 1))

    P1 = K @ np.hstack((I, zero))  # First camera: P1 = K [I | 0]
    P2 = K @ np.hstack((R, t))  # Second camera: P2 = K [R | t]

    homogeneous_points = cv.triangulatePoints(P1, P2, src_pts.T, dst_pts.T)

    homogeneous_points /= homogeneous_points[3]
    points_3d = homogeneous_points[:3].T

    return points_3d


def write_ply(filename: str, points: np.ndarray):
    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("element vertex {}\n".format(len(points)))
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for pt in points:
            f.write("{} {} {}\n".format(pt[0], pt[1], pt[2]))

def incremental_registration(images, K, initial_pose, global_points, global_descriptors):

    sift = cv.SIFT_create()
    index_params = dict(algorithm=0, trees=5)
    search_params = dict(checks=50)
    flann = cv.FlannBasedMatcher(indexParams=index_params, searchParams=search_params)

    poses = []  # Store new poses (each is (R, t))
    for i in range(3, len(images)):
        print(f"Processing image {i}...")
        image = images[i]
        keypoints_new, descriptors_new = sift.detectAndCompute(image, None)

        # Match new image descriptors to the global descriptors (from current 3D points)
        matches = flann.knnMatch(descriptors_new, global_descriptors, k=2)
        good_matches = []
        for m, n in matches:
            if m.distance < 0.35 * n.distance:
                good_matches.append(m)
        if len(good_matches) < 6:
            print("Not enough 2D-3D matches for image", i)
            continue

        # Prepare 2D-3D correspondences for PnP
        pts2d = np.array([keypoints_new[m.queryIdx].pt for m in good_matches], dtype=np.float32)
        pts3d = np.array([global_points[m.trainIdx] for m in good_matches], dtype=np.float32)

        # PnP with RANSAC to estimate the camera pose for the new image
        success, rvec, tvec, inliers = cv.solvePnPRansac(pts3d, pts2d, K, None)
        if not success:
            print("PnP failed for image", i)
            continue
        R_new, _ = cv.Rodrigues(rvec)
        t_new = tvec
        poses.append((R_new, t_new))

        # Triangulate new points between the previous image and the current image.

        # We use feature detection between images[i-1] and images[i]
        src_pts, dst_pts, new_desc, _ = feature_detection(images[i - 1], images[i])

        # Get the pose for the previous image.
        if i - 1 == 2:
            R_prev, t_prev = initial_pose
        else:
            R_prev, t_prev = poses[-2]

        # Build projection matrices for the two images
        P_prev = K @ np.hstack((R_prev, t_prev))
        P_curr = K @ np.hstack((R_new, t_new))

        # Triangulate new points
        homogeneous_points = cv.triangulatePoints(P_prev, P_curr, src_pts.T, dst_pts.T)
        homogeneous_points /= homogeneous_points[3]
        new_points_3d = homogeneous_points[:3].T

        # Append the newly triangulated points and their descriptors to the global model
        global_points = np.vstack((global_points, new_points_3d))
        global_descriptors = np.vstack((global_descriptors, new_desc))

    return poses, global_points, global_descriptors


if __name__ == '__main__':
    # Load Camera Intrinsics
    K = np.loadtxt('K.txt', dtype=np.float32)

    # Load images
    images = [cv.imread("images/{:04d}.jpg".format(i), cv.IMREAD_GRAYSCALE) for i in range(30)]

    # Initial pair: use images[0] and images[1]
    src_pts, dst_pts, good_descriptors, _ = feature_detection(images[1], images[2])
    R, t = estimate_camera_pose(src_pts, dst_pts, K)
    initial_points = triangulate_points(K, src_pts, dst_pts, R, t)

    # Set up the global point cloud and descriptors from the initial pair.
    global_points = initial_points
    global_descriptors = good_descriptors

    # Assume image 0 has pose [I|0] and image 1 has the estimated (R, t)
    poses = [(np.eye(3), np.zeros((3, 1))), (R, t)]

    new_poses, global_points, global_descriptors = incremental_registration(
        images, K, (R, t), global_points, global_descriptors)
    poses.extend(new_poses)

    write_ply("mycloud.ply", global_points)