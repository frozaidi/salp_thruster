import cv2
import numpy as np
import csv
import os

# -------------------------
# CONFIGURATION
# -------------------------
ground_ids = [0,1,2]   # IDs of ground markers (we only use the first as reference)
robot_id = 4             # ID of the robot's ArUco marker
# marker_length = 0.135    # Marker size in meters (diffdrive)
marker_length = 0.085    # Marker size in meters (salpchain)
calibration_file = 'calibration_chessboard.yaml'

# Input/Output paths
input_dir = "chain_videos"  # <-- set your actual input folder here
output_csv_dir = "processed_csv"
output_video_dir = "processed_videos"

os.makedirs(output_csv_dir, exist_ok=True)
os.makedirs(output_video_dir, exist_ok=True)

file_list = os.listdir(input_dir)  # process all videos in folder
# file_list = ["test1.MOV"]  # uncomment to process just one

# -------------------------
# HELPER FUNCTIONS
# -------------------------
def build_transform(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T

def transform_to_ref(rvec, tvec, T_ref_inv):
    T = build_transform(rvec, tvec)
    T_rel = T_ref_inv @ T
    return T_rel[:3, :3], T_rel[:3, 3]

def project_point_to_plane(point, plane_point, plane_normal):
    plane_normal = plane_normal / np.linalg.norm(plane_normal)
    v = point - plane_point
    distance = np.dot(v, plane_normal)
    return point - distance * plane_normal

def rotation_matrix_to_rpy(R):
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy < 1e-6:  # singular
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])

# -------------------------
# PROCESS VIDEOS
# -------------------------
for videofile in file_list:
    print(f"Processing {videofile}...")

    video_path = os.path.join(input_dir, videofile)
    video = cv2.VideoCapture(video_path)

    if not video.isOpened():
        print(f"Could not open video: {video_path}")
        continue

    width  = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps    = video.get(cv2.CAP_PROP_FPS) or 30.0

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    output_video_path = os.path.join(output_video_dir, f"processed_{videofile}")
    output_video = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

    # ArUco setup
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()

    # Camera intrinsics
    cv_file = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode('K').mat()
    dst = cv_file.getNode('D').mat()
    cv_file.release()

    csv_filename = os.path.join(output_csv_dir, videofile[:-4] + ".csv")
    trajectory_points = []  # store robot trajectory in image coordinates
    last_frame = None       # store last video frame

    with open(csv_filename, mode="w", newline="") as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["Frame", "Robot_X", "Robot_Y", "Robot_Z",
                             "Roll_deg", "Pitch_deg", "Yaw_deg"])

        while video.isOpened():
            ret, frame = video.read()
            if not ret:
                break

            frame_number = int(video.get(cv2.CAP_PROP_POS_FRAMES))

            # --- Detect markers ---
            corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

            # If detected, update pose and append the newest trajectory point
            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dst)
                marker_dict = {id_[0]: (rvec, tvec, c)
                               for id_, rvec, tvec, c in zip(ids, rvecs, tvecs, corners)}

                if all(gid in marker_dict for gid in ground_ids) and robot_id in marker_dict:
                    rvec_ref, tvec_ref, _ = marker_dict[ground_ids[0]]
                    rvec2, tvec2, _ = marker_dict[ground_ids[1]]
                    rvec3, tvec3, _ = marker_dict[ground_ids[2]]
                    rvec_robot, tvec_robot, robot_corners = marker_dict[robot_id]

                    T_ref = build_transform(rvec_ref, tvec_ref)
                    T_ref_inv = np.linalg.inv(T_ref)

                    _, p2 = transform_to_ref(rvec2, tvec2, T_ref_inv)
                    _, p3 = transform_to_ref(rvec3, tvec3, T_ref_inv)
                    p1 = np.array([0, 0, 0])

                    R_robot_rel, robot_pos_rel = transform_to_ref(rvec_robot, tvec_robot, T_ref_inv)

                    v1, v2 = p2 - p1, p3 - p1
                    plane_normal = np.cross(v1, v2)
                    plane_normal /= np.linalg.norm(plane_normal)
                    robot_pos_proj = project_point_to_plane(robot_pos_rel, p1, plane_normal)

                    roll, pitch, yaw = rotation_matrix_to_rpy(R_robot_rel)

                    csv_writer.writerow([frame_number] + list(robot_pos_proj) +
                                        [roll, pitch, yaw])

                    # Append new pixel center for trajectory (only when detected)
                    u = int(np.mean(robot_corners[0][:, 0]))
                    v = int(np.mean(robot_corners[0][:, 1]))
                    trajectory_points.append((u, v))

                    # Draw current axes only when detected (video only)
                    cv2.drawFrameAxes(frame, mtx, dst, rvec_robot, tvec_robot, 0.1)

            # --- Draw the trajectory on EVERY frame (persist overlay) ---
            if len(trajectory_points) > 1:
                for i in range(1, len(trajectory_points)):
                    # BGR = (9, 63, 215) ≈ orange; thickness 6
                    cv2.line(frame, trajectory_points[i-1], trajectory_points[i], (9, 63, 215), 6)

                # Mark the last known robot position
                cv2.circle(frame, trajectory_points[-1], 5, (255, 0, 0), -1)

            # Save for final snapshot (last processed frame with background visible)
            last_frame = frame.copy()

            output_video.write(frame)
            # Optional live preview (press 'q' to quit)
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    video.release()
    output_video.release()
    cv2.destroyAllWindows()

    # Save final trajectory image (last frame + orange trail, no axes)
    if last_frame is not None and len(trajectory_points) > 1:
        final_img = last_frame.copy()
        for i in range(1, len(trajectory_points)):
            cv2.line(final_img, trajectory_points[i-1], trajectory_points[i], (9, 63, 215), 6)
        cv2.circle(final_img, trajectory_points[-1], 5, (255, 0, 0), -1)  # last robot position

        final_img_name = os.path.join(output_video_dir, "trajectory_" + videofile[:-4] + ".png")
        cv2.imwrite(final_img_name, final_img)
        print(f"Saved final trajectory image: {final_img_name}")

    print(f"Finished: {videofile}, results saved to {csv_filename}\n")
