import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R

def kitti_pose_to_tum_format(kitti_pose_file, tum_output_file):
    """
    Converts a KITTI pose file (with time in the first column and the 3x4 transform next)
    into a TUM-style file with columns: [timestamp tx ty tz qx qy qz qw].

    Parameters:
    kitti_pose_file : str
        Path to the KITTI poses file.

    tum_output_file : str
        Path to the output TUM formatted file.
    """
    with open(kitti_pose_file, 'r') as f:
        lines = f.readlines()

    with open(tum_output_file, 'w') as f:
        for line in lines:
            data = line.strip().split()
            data = [float(val) for val in data]
            
            # Time plus 3x4 transformation matrix
            time_sec = data[0]
            transformation = np.array(data[1:]).reshape(3, 4)
            rot = transformation[:, :3]
            trans = transformation[:, 3]

            # Convert rotation matrix to a quaternion (x, y, z, w)
            r = R.from_matrix(rot)
            qx, qy, qz, qw = r.as_quat()
            
            # Write in TUM format: time tx ty tz qx qy qz qw
            f.write(f"{time_sec:.6f} {trans[0]:.6f} {trans[1]:.6f} {trans[2]:.6f} "
                    f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert KITTI pose file to TUM format")
    parser.add_argument("--kitti_pose_file", type=str, required=True, 
                        help="Path to the KITTI pose file (time + 3x4 matrix)")
    parser.add_argument("--tum_output_file", type=str, required=True, 
                        help="Path for the output TUM format file")
    args = parser.parse_args()

    kitti_pose_to_tum_format(args.kitti_pose_file, args.tum_output_file)
