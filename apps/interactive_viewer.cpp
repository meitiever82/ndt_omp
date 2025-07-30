#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>

using namespace std::chrono_literals;
using namespace pcl::console;

// Global state for interactive features
std::atomic<bool> show_source{true};
std::atomic<bool> show_target{true};
std::atomic<bool> show_grid{true};
std::atomic<int> point_size{2};
std::atomic<int> color_mode{0};
std::mutex viewer_mutex;

// Point picking callback
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* /*cookie*/)
{
    int idx = event.getPointIndex();
    if (idx == -1)
        return;

    pcl::PointXYZ picked_pt;
    event.getPoint(picked_pt.x, picked_pt.y, picked_pt.z);
    std::cout << "Point picked: [" << picked_pt.x << ", " << picked_pt.y << ", " << picked_pt.z << "]" << std::endl;
}

// Keyboard callback
void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void* viewer_void)
{
    auto* viewer = reinterpret_cast<pcl::visualization::PCLVisualizer*>(viewer_void);
    
    if (event.keyDown())
    {
        std::lock_guard<std::mutex> lock(viewer_mutex);
        
        if (event.getKeySym() == "space")
        {
            show_source = !show_source;
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 
                                                   show_source ? 1.0 : 0.0, "source");
            std::cout << "Source cloud: " << (show_source ? "ON" : "OFF") << std::endl;
        }
        else if (event.getKeySym() == "t")
        {
            show_target = !show_target;
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 
                                                   show_target ? 1.0 : 0.0, "target");
            std::cout << "Target cloud: " << (show_target ? "ON" : "OFF") << std::endl;
        }
        else if (event.getKeySym() == "g")
        {
            show_grid = !show_grid;
            if (show_grid) {
                viewer->addCoordinateSystem(2.0, "coord_system");
            } else {
                viewer->removeCoordinateSystem("coord_system");
            }
            std::cout << "Coordinate system: " << (show_grid ? "ON" : "OFF") << std::endl;
        }
        else if (event.getKeySym() == "plus" || event.getKeySym() == "equal")
        {
            point_size = std::min(point_size.load() + 1, 10);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
                                                   point_size.load(), "source");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
                                                   point_size.load(), "target");
            std::cout << "Point size: " << point_size.load() << std::endl;
        }
        else if (event.getKeySym() == "minus")
        {
            point_size = std::max(point_size.load() - 1, 1);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
                                                   point_size.load(), "source");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
                                                   point_size.load(), "target");
            std::cout << "Point size: " << point_size.load() << std::endl;
        }
        else if (event.getKeySym() == "c")
        {
            color_mode = (color_mode + 1) % 3;
            std::cout << "Color cycling not implemented in this version" << std::endl;
        }
        else if (event.getKeySym() == "r")
        {
            viewer->resetCamera();
            std::cout << "Camera reset" << std::endl;
        }
        else if (event.getKeySym() == "h")
        {
            std::cout << "\n=== Interactive Controls ===" << std::endl;
            std::cout << "SPACE    - Toggle source cloud" << std::endl;
            std::cout << "T        - Toggle target cloud" << std::endl;
            std::cout << "G        - Toggle coordinate system" << std::endl;
            std::cout << "+/-      - Increase/decrease point size" << std::endl;
            std::cout << "C        - Cycle colors" << std::endl;
            std::cout << "R        - Reset camera" << std::endl;
            std::cout << "H        - Show this help" << std::endl;
            std::cout << "Q        - Quit" << std::endl;
        }
    }
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        print_info("Usage: %s <source.pcd> [target.pcd] [options]\n", argv[0]);
        print_info("Options:\n");
        print_info("  --leaf_size <value>    Voxel filter leaf size (default: 0.1)\n");
        print_info("  -ps <size>             Point size (default: 2)\n");
        print_info("  -bc r,g,b              Background color (default: 0.1,0.1,0.1)\n");
        return 1;
    }

    // Parse arguments
    float leaf_size = 0.1f;
    pcl::console::parse_argument(argc, argv, "--leaf_size", leaf_size);
    
    int ps = 2;
    pcl::console::parse_argument(argc, argv, "-ps", ps);
    point_size = ps;
    
    double bcolor[3] = {0.1, 0.1, 0.1};
    pcl::console::parse_3x_arguments(argc, argv, "-bc", bcolor[0], bcolor[1], bcolor[2]);

    // Load point clouds using PCL's robust loading
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::console::TicToc tt;
    tt.tic();
    print_highlight("Loading source: %s ", argv[1]);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *source) == -1) {
        print_error("Failed to load source file\n");
        return -1;
    }
    print_info("[done, %g ms : %zu points]\n", tt.toc(), source->size());

    bool has_target = false;
    if (argc > 2) {
        tt.tic();
        print_highlight("Loading target: %s ", argv[2]);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *target) == -1) {
            print_error("Failed to load target file\n");
            return -1;
        }
        print_info("[done, %g ms : %zu points]\n", tt.toc(), target->size());
        has_target = true;
    }

    // Downsample
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    
    tt.tic();
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.setInputCloud(source);
    voxel_filter.filter(*source_filtered);
    print_info("Source filtered: %zu points ", source_filtered->size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (has_target) {
        voxel_filter.setInputCloud(target);
        voxel_filter.filter(*target_filtered);
        print_info("| Target filtered: %zu points", target_filtered->size());
    }
    print_info(" [%g ms]\n", tt.toc());

    // Create PCLVisualizer - use the same pattern as official viewer
    pcl::visualization::PCLVisualizer::Ptr p(new pcl::visualization::PCLVisualizer(argc, argv, "Interactive PCL Viewer"));
    
    // Set background
    p->setBackgroundColor(bcolor[0], bcolor[1], bcolor[2]);
    
    // Register callbacks
    p->registerKeyboardCallback(keyboard_callback, (void*)p.get());
    p->registerPointPickingCallback(pp_callback, nullptr);
    
    // Add coordinate system initially
    p->addCoordinateSystem(2.0, "coord_system");
    
    // Add point clouds using PCL's robust method
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source_filtered, 255, 0, 0);
    p->addPointCloud<pcl::PointXYZ>(source_filtered, source_color, "source");
    p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size.load(), "source");

    if (has_target) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_filtered, 0, 255, 0);
        p->addPointCloud<pcl::PointXYZ>(target_filtered, target_color, "target");
        p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size.load(), "target");
    }

    // Add text overlay
    std::string info_text = "Source: " + std::to_string(source_filtered->size()) + " points";
    if (has_target) {
        info_text += " | Target: " + std::to_string(target_filtered->size()) + " points";
    }
    p->addText(info_text, 10, 10, 14, 1.0, 1.0, 1.0, "info");
    p->addText("Press 'H' for help", 10, 30, 12, 0.8, 0.8, 0.8, "help");

    // Reset camera to show all data
    p->resetCameraViewpoint("source");
    p->resetCamera();

    print_info("\n=== Interactive Point Cloud Viewer Started ===\n");
    print_info("Press 'H' in the viewer window for help\n");
    print_info("Basic controls: SPACE=toggle source, T=toggle target, +/-=point size, Q=quit\n");

    // Main loop - use the same pattern as official viewer
    p->spin();

    return 0;
}