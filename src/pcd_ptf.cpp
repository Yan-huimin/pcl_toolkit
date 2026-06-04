#include "common.hpp"



// 显示帮助信息
void showHelp(char *program_name) {
    std::cout << "Usage: " << program_name << " <point_cloud.pcd> <axis> <min> <max>\n\n"
              << "Crop the point cloud using a PassThrough filter, keeping points within [min, max] along the given axis.\n"
              << "Axis options: x, y, z\n"
              << "Example: " << program_name << " ./data.pcd z 0.0 2.0\n"
              << "Output will be saved with suffix '_cropped' added to the original filename.\n"
              << "-h, --help : Show this help" << std::endl;
}

int main(int argc, char **argv) {

    // 检查帮助选项
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")) {
        showHelp(argv[0]);
        return 0;
    }

    // 参数数量检查
    if (argc != 5) {
        std::cerr << "错误: 参数数量不正确。\n";
        showHelp(argv[0]);
        return -1;
    }

    // 解析命令行参数
    std::string input_file = argv[1];        // 输入点云文件
    std::string axis_str = argv[2];          // 裁剪轴：x, y, z
    double min_range = std::stod(argv[3]);   // 最小边界
    double max_range = std::stod(argv[4]);   // 最大边界

    // 校验轴参数
    if (axis_str != "x" && axis_str != "y" && axis_str != "z") {
        std::cerr << "Error: The axis parameter must be x, y, or z.\n";
        showHelp(argv[0]);
        return -1;
    }

    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1) {
        std::cerr << "错误: 无法打开点云文件 " << input_file << std::endl;
        return -1;
    }
    std::cout << "原始点云点数: " << cloud->points.size() << std::endl;

    // 创建直通滤波器
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    // 设置裁剪轴
    pass.setFilterFieldName(axis_str);
    // 设置裁剪范围
    pass.setFilterLimits(min_range, max_range);
    // 可选：是否保留裁剪区域之外的点，默认为 false（保留范围内的点）
    // pass.setFilterLimitsNegative(false);

    // 执行滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*cloud_filtered);

    std::cout << "裁剪后点云点数: " << cloud_filtered->points.size() << std::endl;

    // 生成输出文件名：在原文件名后添加 "_cropped" 后缀
    std::string output_file;
    size_t pos = input_file.find_last_of('.');
    if (pos != std::string::npos) {
        output_file = input_file.substr(0, pos) + "_cropped" + input_file.substr(pos);
    } else {
        // 如果没有扩展名，直接加后缀
        output_file = input_file + "_cropped.pcd";
    }

    // 保存裁剪后的点云
    if (pcl::io::savePCDFileASCII(output_file, *cloud_filtered) == -1) {
        std::cerr << "Error: Unable to save point cloud file " << output_file << std::endl;
        return -1;
    }
    std::cout << "Crop result saved to: " << output_file << std::endl;

    return 0;
}