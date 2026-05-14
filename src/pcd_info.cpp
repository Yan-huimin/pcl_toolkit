#include <iostream>
#include <cmath>
#include <algorithm>


#include "common.hpp"


float neg_inf = -INFINITY;

void
pcl_info_help(char * program_name){
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << "cloud_filename.[pcd]" << std::endl;
    std::cout << "-h: Show this help" << std::endl;
}


int
main(int argc, char** argv){

    // 当用户输入-h 或者 --help时，显示帮助
    if(pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")){
        pcl_info_help(argv[0]);
        return 0;
    }

    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

    // 如果输入文件个数不为1，显示帮助并退出
    if(filenames.size() != 1){
        pcl_info_help(argv[0]);
        return -1;
    }

    // 加载文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0){
        std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
        pcl_info_help(argv[0]);
        return -1;
    }

    // 检查范围
    if (!source_cloud->is_dense){
        std::cout << "[WARNING]: "
                  << "There are some NAN valude!"
                  << std::endl 
                  << std::endl;
        pcl_info_help(argv[0]);
        return -1;
    }

    // 点云尺寸
    std::cout << std::endl
              << argv[filenames[0]]
              << " [State]  ->  load successfully\n"
              << "WIDTH: " << source_cloud -> width << std::endl
              << "HEIGHT: " << source_cloud -> height << std::endl;

    // float min_x = neg_inf, max_x = -neg_inf;
    // float min_y = neg_inf, max_y = -neg_inf;
    // float min_z = neg_inf, max_z = -neg_inf;
    // for(const auto& point: *source_cloud){
    //     min_x = std::min(point.x, min_x);    max_x = std::max(point.x, max_x);
    //     min_y = std::min(point.y, min_y);    max_y = std::max(point.y, max_y);
    //     min_z = std::min(point.z, min_z);    max_z = std::max(point.z, max_z);
    // }

    pcl::PointXYZ min_pt;   pcl::PointXYZ max_pt;
    pcl::getMinMax3D(*source_cloud, min_pt, max_pt);


    // 坐标范围
    std::cout << std::endl 
              << "---------------INFO---------------" << std::endl
              << "Total Points: " << source_cloud -> size() << std::endl
              << "X: " << min_pt.x << " -> " << max_pt.x << std::endl
              << "Y: " << min_pt.y << " -> " << max_pt.y << std::endl
              << "Z: " << min_pt.z << " -> " << max_pt.z << std::endl;

    return 0;
}