import numpy as np
from dataclasses import dataclass
import gtsam
import open3d as o3d
import csv

@dataclass
class Traj:
    t: np.array
    xyz: np.array
    qxyzw: np.array

# Parameters
downsampling = 20
traj_est_file = '/home/minwoo/Research/Seminar/eval_tools/Example/point_lio_helipr_lidar.txt'

# Load trajectory data
traj_est = np.loadtxt(traj_est_file, delimiter=' ', dtype=str)

# Convert data while keeping timestamps as strings
timestamps = traj_est[:, 0]  # Keep as strings
xyz = traj_est[:, 1:4].astype(float)
qxyzw = traj_est[:, 4:].astype(float)

traj_est = Traj(t=timestamps, xyz=xyz, qxyzw=qxyzw)

traj_est_poses = []
traj_est_poses_mesh = []

# Convert trajectory data to GTSAM poses
for pose_idx in range(traj_est.t.shape[0]):
    trans = gtsam.Point3(traj_est.xyz[pose_idx, :])
    qxyzw = traj_est.qxyzw[pose_idx, :]
    R = gtsam.Rot3.Quaternion(qxyzw[-1], qxyzw[0], qxyzw[1], qxyzw[2])  # GTSAM uses the order of w, x, y, z
    pose = gtsam.Pose3(R, trans)
    traj_est_poses.append(pose)

    # Visualization mesh
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(5)
    mesh_t = mesh.transform(pose.matrix())
    if pose_idx % downsampling == 0:
        traj_est_poses_mesh.append(mesh_t)

# Transformation matrices for coordinate conversions
_source_to_target = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]

source_to_target_rot = gtsam.Rot3(_source_to_target[0], _source_to_target[1], _source_to_target[2],
                                  _source_to_target[4], _source_to_target[5], _source_to_target[6],
                                  _source_to_target[8], _source_to_target[9], _source_to_target[10])
source_to_target = gtsam.Pose3(source_to_target_rot, gtsam.Point3(_source_to_target[3], _source_to_target[7], _source_to_target[11]))
target_to_source = source_to_target.inverse()

# for i in range(len(traj_est_poses)):
#     # add random noise to the poses only for translation
#     noise = np.random.normal(0, 0.3, 3)
#     noise_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(noise))
#     traj_est_poses[i] = traj_est_poses[i].compose(noise_pose)

# Parse reoriented poses (init as origin)
traj_est_poses_reorigin = []
for ii, curr_pose_est in enumerate(traj_est_poses):
    if ii == 0:
        prev_pose_est = curr_pose_est
        curr_pose_recal = curr_pose_est.compose(target_to_source)
        traj_est_poses_reorigin.append(curr_pose_recal)
        continue

    rel_pose = prev_pose_est.between(curr_pose_est)

    # Hand-eye calibration
    curr_pose_recalc = traj_est_poses_reorigin[-1].compose(target_to_source).compose(rel_pose).compose(source_to_target)
    traj_est_poses_reorigin.append(curr_pose_recalc)

    prev_pose_est = curr_pose_est

# Visualization of reoriented trajectory
traj_est_poses_mesh = []
for ii, pose in enumerate(traj_est_poses_reorigin):
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(10)
    mesh_t = mesh.transform(pose.matrix())
    if ii % downsampling == 0:
        traj_est_poses_mesh.append(mesh_t)

# Rewrite text file with maintained timestamp format
poses_lines = []
for ii, pose in enumerate(traj_est_poses_reorigin):
    timestamp = traj_est.t[ii]  # Keep timestamp as string
    xyz = pose.translation()
    xyz = [xyz[0], xyz[1], xyz[2]]
    qwxyz = pose.rotation().quaternion()
    qxyzw = [qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]]
    
    # Prepare the line with all elements
    line = [timestamp] + [f'{coord:.9f}' for coord in xyz] + [f'{quat:.9f}' for quat in qxyzw]
    poses_lines.append(line)

# Save the reoriented pose file using csv.writer to maintain timestamp format
poses_lines_filename = "/home/minwoo/Research/Seminar/eval_tools/Example/point_lio_helipr_lidar_reorigin1.txt"
with open(poses_lines_filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=' ')
    writer.writerows(poses_lines)

print(f'Reoriented pose file is saved. See the file {poses_lines_filename}')

# Visualize the trajectory
o3d.visualization.draw_geometries(traj_est_poses_mesh)
