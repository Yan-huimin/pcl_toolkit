#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "common.hpp"

void sor_help(char * program_name){
    std::cout << std::endl
              << "Usage: " << program_name << " cloud_name.pcd" << " output.pcd" << " meank(int)" << " sigma-mult(double)" << std::endl;
    std::cout << "-h: Show this help" << std::endl;
}

int
main(int argc, char** argv){

    std::string input, output;
    
    int meank = 50;  double sigma_mult = 1.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    if(argc != 5){
        sor_help(argv[0]);
        return -1;
    }

    if(pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")){
        sor_help(argv[0]);
        return 0;
    }

    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

    if(filenames.size() != 2){
        sor_help(argv[0]);
        return -1;
    }

    input = argv[filenames[0]];
    output = argv[filenames[1]];

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ> (input, *cloud);

    // start sor
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meank);
    sor.setStddevMulThresh(sigma_mult);
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;


    //save file
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (output, *cloud_filtered, false);

    return 0;
}