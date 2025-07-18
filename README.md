
```md
# 📦 Plane Segmentation Pipeline for Sub-Millimeter Pose Estimation

A robust and modular plane segmentation pipeline built using **PCL (Point Cloud Library)**, designed for robotic manipulation tasks requiring **sub-millimeter accuracy**. Originally developed for a **warehouse pick-and-place robot**, the pipeline extracts clean, smooth planes from noisy point cloud data — ideal for pose estimation of deformable or untagged items.

---

## 🔧 Key Features

- ✅ **Two-stage RANSAC** for robust plane extraction  
- ✅ **Voxel downsampling** for efficient processing  
- ✅ **Statistical Outlier Removal (SOR)** for denoising  
- ✅ **Euclidean Clustering** for ROI selection  
- ✅ **MLS smoothing** to flatten deformable surfaces  
- ✅ **Config-driven** tuning of all parameters  
- ✅ **CLI interface** — no ROS needed  
- ✅ **PCD output + pcl_viewer ready**

---

## 📁 Repository Structure

```

.
├── src/              # Main implementation (main.cpp, PlaneSegmenter.cpp)
├── include/          # Header (PlaneSegmenter.hpp)
├── config/           # JSON config file with all tunables
├── data/             # Sample input PCDs
├── output/           # Segmented & cleaned PCDs
├── CMakeLists.txt    # Build setup
└── README.md

````

---

## ⚙️ Configurable Parameters

All parameters are stored in `config/config.json`:

| Parameter               | Description                                  |
|------------------------|----------------------------------------------|
| `voxel_leaf_size`      | Downsampling resolution (e.g. 0.01)          |
| `sor_mean_k`           | Neighbors used in outlier removal            |
| `sor_std_dev_mul`      | Threshold for SOR filtering                  |
| `ransac_dist_thresh`   | RANSAC inlier distance                       |
| `cluster_tolerance`    | Clustering distance threshold                |
| `min_cluster_size`     | Minimum cluster size                         |
| `mls_search_radius`    | Radius for MLS smoothing                     |

---

## 🚀 How to Build

```bash
# Clone and go into the repo
git clone https://github.com/yourusername/plane_segmentation_tool.git
cd plane_segmentation_tool

# Create build directory
mkdir build && cd build

# Build
cmake ..
make -j8
````

---

## 🧪 How to Use

```bash
./plane_segmenter ../data/input.pcd ../config/config.json
```

After running, your cleaned plane will be stored in `output/final_output.pcd`.

You can view it using:

```bash
pcl_viewer output/final_output.pcd
```

---

## 🔍 Use Case

This was built for a warehouse robot that had to **detect the pose of boxes** without any fiducials. Why?

* Barcodes and ArUco markers **get damaged**
* Human setup **doesn’t scale**
* Boxes **shift position** in chaotic environments

This pipeline provides a robust, scalable solution for **pose estimation**, **pick-and-place**, and **inspection tasks** — all without requiring manual tagging.

---

## ⚙️ Engineering Considerations

* ⚡ **Speed**: With focused ROI input, the pipeline is fast enough for real-time use
* 🧱 **Modular**: Separated into `src/`, `include/`, `config/`, `data/`, and `output/`
* 🖥️ **Viewer-Ready**: Output PCDs can be visualized using `pcl_viewer`
* 🛠️ **Tunable**: Config-driven parameters for adapting to different environments
* 🧪 **Tested**: Built for and tested with deformable, noisy warehouse point clouds

---

## 🤝 Contributing

Pull requests are welcome! This project is useful for anyone working in:

* Robotics & manipulation
* Bin picking
* Industrial inspection
* 3D vision & AR/VR
* Point cloud preprocessing

If you find it helpful or adapt it for your workflow, please consider contributing or sharing your use case.

---

## 📜 License

This project is licensed under the MIT License.

---

## ☁️ How to Push This to GitHub

> **Note:** Replace `yourusername` with your actual GitHub username.

```bash
# Inside your local repo
git init
git add .
git commit -m "Initial commit: robust plane segmentation pipeline"

# Add remote
git remote add origin https://github.com/yourusername/plane_segmentation_tool.git

# Push to GitHub
git branch -M main
git push -u origin main
```

---

## 🧠 Author's Note

This tool was built from a real-world need — where fiducials failed, boxes deformed, and sub-mm pose accuracy was non-negotiable. If you're solving a similar problem, feel free to reach out or fork the repo to make it your own.

---

```

