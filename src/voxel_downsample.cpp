#include "common.hpp"

// 帮助显示
void 
showHelp(char * program_name){

    std::cout << std::endl
              << "Usage-1: " << program_name << " --same " << "--leaf " << "leaf_size " << "inputfilename.pcd " << "outputfilename.pcd" << std::endl
              << "Usage-2: " << program_name << " --leaf_x " << " leaf_size_1" << " --leaf_y " << " leaf_size_2" << " --leaf_z " << " leaf_size_3" << " inputfilename.pcd" << "outputfilename.pcd" << std::endl
              << "-h: Show this help" << std::endl;

}

int
main(int argc, char** argv){

    bool is_same = false;
    float leaf_1 = 0.5, leaf_2 = 0.5, leaf_3 = 0.5;

    std::string input, output;
    pcl::PCDReader reader;

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filter (new pcl::PCLPointCloud2());

    // 如果输入中包含-h || --help 就输出帮助
    if(pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")){
        showHelp(argv[0]);
        return 0;
    }

    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

    if(filenames.size() != 2){
        showHelp(argv[0]);
        return -1;
    }

    input = argv[filenames[0]];
    output = argv[filenames[1]];

    is_same = pcl::console::find_switch(argc, argv, "--same");

    if(is_same){
        float cur = 0.5;    pcl::console::parse(argc, argv, "--leaf", cur);
        leaf_1 = leaf_2 = leaf_3 = cur;
    }else{
        pcl::console::parse(argc, argv, "leaf_x", leaf_1);
        pcl::console::parse(argc, argv, "leaf_y", leaf_2);
        pcl::console::parse(argc, argv, "leaf_z", leaf_3);
    }

    std::cerr << "Start downsampling......" << std::endl
              << "leaf_size: " << leaf_1 << " " << leaf_2 << " " << leaf_3 << std::endl;

    //开始降采样
    reader.read(input, *cloud);
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
              << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(leaf_1, leaf_2, leaf_3);
    filter.filter(*cloud_filter);

    std::cerr << "PointCloud after filtering: " << cloud_filter->width * cloud_filter->height 
              << " data points (" << pcl::getFieldsList (*cloud_filter) << ")." << std::endl;


    pcl::PCDWriter writer;
    writer.write(output, *cloud_filter, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

    std::cout << "File saved successfully: " << std::endl
              << "      " << output << std::endl;
    
    return 0;
}
