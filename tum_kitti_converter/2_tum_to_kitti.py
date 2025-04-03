import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R

def tum_to_kitti_format(tum_file, kitti_output_file):
    """
    Converts TUM format file to KITTI pose format with a time value in each line.

    Parameters:
    tum_file : str
        Path to the TUM formatted file (time tx ty tz qx qy qz qw).
    kitti_output_file : str
        Path to the output KITTI formatted file.
    """
    with open(tum_file, 'r') as f_in:
        lines = f_in.readlines()

    with open(kitti_output_file, 'w') as f_out:
        for line in lines:
            data = line.strip().split()
            data = [float(val) for val in data]
            
            # TUM format: [time tx ty tz qx qy qz qw]
            time_sec = data[0]
            tx, ty, tz = data[1:4]
            qx, qy, qz, qw = data[4:8]

            # Convert quaternion to rotation matrix (3x3)
            r = R.from_quat([qx, qy, qz, qw])
            rot = r.as_matrix()
            
            # Build the 3x4 transformation matrix: [r11 r12 r13 tx; r21 r22 r23 ty; r31 r32 r33 tz]
            transformation = np.hstack((rot, np.array([[tx], [ty], [tz]])))

            # Flatten 3x4 matrix into 12 values, plus time
            transformation_values = " ".join(f"{val:.6f}" for val in transformation.flatten())
            f_out.write(f"{time_sec:.6f} {transformation_values}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert TUM format file to KITTI pose format.")
    parser.add_argument("--tum_file", type=str, required=True, 
                        help="Path to the input TUM file (time tx ty tz qx qy qz qw).")
    parser.add_argument("--kitti_output_file", type=str, required=True, 
                        help="Path to the output KITTI pose file.")
    args = parser.parse_args()

    tum_to_kitti_format(args.tum_file, args.kitti_output_file)
