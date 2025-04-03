# Odometry Evaluation Tools

A collection of Python scripts and example data designed to **pre-process** odometry results—converting them between TUM and KITTI formats, applying frame transformations, and generating outputs suitable for subsequent evaluation with **EVO** or the **RPG Trajectory Evaluator**.

## Repository Layout

```
├── Example/
│   ├── point_lio_helipr_lidar.txt
│   ├── Ouster_gt_time.txt
│
├── frame_changer/
│   └── drawer_est_lidarcoord.py
│
├── tum_kitti_converter/
│   ├── 1_kitti_to_tum.py
│   └── 2_tum_to_kitti.py
│
└── README.md
```

### 1. Example/
Contains sample odometry text files with timestamps, poses, etc. You can test the converter scripts on these.

### 2. frame_changer/
- **drawer_est_lidarcoord.py**  
  Demonstrates:
  - Loading odometry text files into GTSAM’s `Pose3`.
  - Applying coordinate transformations or a calibration offset (e.g., changing the origin or frame).
  - Saving re-oriented results to a new file.
  - Visualizing in 3D using Open3D.

### 3. tum_kitti_converter/
- **1_kitti_to_tum.py** and **2_tum_to_kitti.py**  
  Convert between KITTI and TUM pose files.  
  - *kitti → tum:* each line of `kitti_pose_file` has `[time r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz]`, and it outputs TUM format `[time tx ty tz qx qy qz qw]`.  
  - *tum → kitti:* the script takes `[time tx ty tz qx qy qz qw]` and writes a 3×4 transformation flattened to a single line plus the timestamp.

---

## Installation & Dependencies

1. **Python 3.7+**  
2. **Core Python Libraries**: NumPy, SciPy, CSV, etc.  
   - `pip install numpy scipy`
3. **Open3D** for 3D visualization (optional, required by `drawer_est_lidarcoord.py`):  
   - `pip install open3d`
4. **GTSAM** for advanced pose transformations (optional).  
   - If you skip GTSAM, remove or comment the relevant lines in `drawer_est_lidarcoord.py`.
5. **EVO** for evaluation & plotting:  
   - `pip install evo --upgrade --no-binary evo`

(Optional) The [RPG evaluator](https://github.com/uzh-rpg/rpg_trajectory_evaluation) can also be used if you prefer a different workflow or want additional metrics.

---

## How to Use

### A. Convert Between TUM & KITTI

1. **KITTI → TUM**  
   ```
   cd tum_kitti_converter/
   python 1_kitti_to_tum.py --kitti_pose_file input_kitti.txt --tum_output_file output_tum.txt
   ```
2. **TUM → KITTI**  
   ```
   cd tum_kitti_converter/
   python 2_tum_to_kitti.py --tum_file input_tum.txt --kitti_output_file output_kitti.txt
   ```

### B. Change or Re-origin a Trajectory

```
cd frame_changer/
python drawer_est_lidarcoord.py
```
- Edits the input file path in `traj_est_file`. 
- Adjust transformations in `_source_to_target` if you have a calibration offset or want to shift axes.
- Outputs a new “reorigin” text file.

---

## Odometry Evaluation Tips (with EVO)

After you have an **estimate** (e.g., `est.tum`) and a **reference** (ground truth, e.g., `reference.tum`), you can run **EVO** to compute absolute or relative pose errors:

1. **APE (Absolute Pose Error)**  
   ```
   evo_ape tum reference.txt estimate.txt --align --plot_mode xy --plot
   evo_ape tum reference.txt estimate.txt --align --plot_mode xy --plot --pose_relation angle_deg
   ```
   - `--align` tries to align the estimate to reference (SE(3) alignment).
   - `--plot_mode xy` plots in 2D overhead view.
   - `--pose_relation angle_deg` can measure orientation errors (in degrees).

2. **RPE (Relative Pose Error)**  
   ```
   evo_rpe tum reference.txt estimate.txt --align --plot_mode xy --plot
   evo_rpe tum reference.txt estimate.txt --align --plot_mode xy --plot --pose_relation angle_deg
   ```
   - For relative/segment errors, you can specify distance or angle metrics.

3. **Multiple Trajectories**  
   If you have multiple estimates:
   ```
   evo_traj tum est1.txt est2.txt est3.txt --ref reference.txt --align --plot_mode xy --plot
   ```
   - The `--t_max_diff` flag helps data association if your timestamps differ slightly.

### Important Details

- **Alignment**: Even for translational error, ensure you align orientation first. If your sensor has a known roll/pitch offset or a calibration file, apply that before running EVO.  
- **Matplotlib Font**: Some conferences (like IEEE) might disallow default fonts. For EVO’s plots, you may want to override the default font. One approach is editing your local matplotlibrc file:  
**Find your `matplotlibrc` file**:
   ```python
   import matplotlib
   print(matplotlib.matplotlib_fname())
   ```
   Then open and edit that file. For example:
   ```
   font.family         : sans-serif
   font.sans-serif     : Arial
   ```
   Restart Python so the changes take effect.
   
## Contributions
- Contributions are welcome—please open an issue or pull request! Maybe, there are many things we should do before the evaluation

---

**Happy Evaluating!** Feel free to customize or extend these scripts for your specific workflow, and remember to carefully handle calibration and alignment steps prior to measuring errors.
