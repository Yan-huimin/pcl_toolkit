cmake_minimum_required(VERSION 3.16)
project(pcl_toolkit)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED COMPONENTS common io filters)

function(add_pcl_tool tool_name source_file)
    add_executable(${tool_name} ${source_file})

    target_include_directories(${tool_name} PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
    )

    target_link_libraries(${tool_name} PRIVATE
        ${PCL_LIBRARIES}
    )

    target_compile_definitions(${tool_name} PRIVATE
        ${PCL_DEFINITIONS}
    )
endfunction()

add_pcl_tool(pcd_info src/pcd_info.cpp)
add_pcl_tool(pcd_voxel_downsample src/voxel_downsample.cpp)
add_pcl_tool(pcd_sor src/pcd_sor.cpp)