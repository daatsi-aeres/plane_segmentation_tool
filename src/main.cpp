#include "PlaneSegmenter.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./plane_segmenter <input_pcd_file> [output_plane_file]" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    std::string output_file = (argc >= 3) ? argv[2] : "segmented_plane.pcd";

    // Load input point cloud
    PlaneSegmenter::CloudT::Ptr input_cloud(new PlaneSegmenter::CloudT);
    if (pcl::io::loadPCDFile(input_file, *input_cloud) == -1) {
        std::cerr << "Failed to load " << input_file << std::endl;
        return -1;
    }

    std::cout << "[INFO] Loaded point cloud: " << input_cloud->size() << " points" << std::endl;

    // Setup parameters
    PlaneSegmenter::Params params;
    params.ransac_distance_threshold_init = 0.01;
    params.ransac_distance_threshold_final = 0.005;
    params.cluster_tolerance = 0.02;
    params.min_cluster_size = 100;
    params.sor_mean_k = 50;
    params.sor_stddev_mul_thresh = 1.0;
    params.mls_radius = 0.03;

    PlaneSegmenter segmenter(params, true);

    pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
    PlaneSegmenter::CloudT::Ptr plane_cloud(new PlaneSegmenter::CloudT);
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);

    segmenter.run(input_cloud, plane_coeff, plane_cloud, plane_inliers);

    if (plane_cloud->empty()) {
        std::cerr << "Plane segmentation failed. No plane extracted." << std::endl;
        return -1;
    }
    	#include <sys/stat.h>  // For mkdir
	#include <pcl/io/pcd_io.h>  // Ensure this is included

	// Create output directory if it doesn't exist
	struct stat info;
	if (stat("../output", &info) != 0) {
	    mkdir("../output", 0777);
	}

	// Save result to PCD
	std::string output_path = "../output/segmented_plane.pcd";
	pcl::io::savePCDFileBinary(output_path, *plane_cloud);

	std::cout << "\n✅ Output point cloud saved to: " << output_path << std::endl;
	std::cout << "👀 You can view it using:\n   pcl_viewer " << output_path << "\n" << std::endl;
    // Save segmented plane
    //pcl::io::savePCDFileBinary(output_file, *plane_cloud);
    //std::cout << "[SUCCESS] Segmented plane saved to: " << output_file << std::endl;

    return 0;
}

