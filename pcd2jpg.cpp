#include <iostream>
#include <fstream>
#include <filesystem>
#include <ctime>
#include <cstring>   
#include <sys/io.h>
#include <dirent.h>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
namespace fs = std::filesystem;
std::vector <std::string> filedir_name_list;
struct RGB{
    int r;
    int g;
    int b;
    //rgb(int r_,int g_,int b_):r(r_),g(g_),b(b_){}
};
int find_rgblist(struct RGB rgb_list[],int len,RGB rgb_){
    for(int i=0;i<len;i++){
        if(rgb_.r==rgb_list[i].r&&rgb_.g==rgb_list[i].g&&rgb_.b==rgb_list[i].b)
            return i;
    }
    return -1;
}
void listDirectories(const char* dir) {
    DIR *dp;
    struct dirent *ep;
    dp = opendir (dir);

    if (dp != NULL) {
        while ((ep = readdir (dp))) {
            std::string fileName = ep->d_name;
            if (fileName == "." || fileName == "..") {
                continue;
            }
            if (ep->d_type == DT_DIR) {
                //std::cout<< fileName<< std::endl;
                filedir_name_list.push_back(fileName);
            }
        }
        (void) closedir (dp);
    } else {
        std::cerr << "Couldn't open the directory!"<< std::endl;
    }
    std::sort(filedir_name_list.begin(),filedir_name_list.end());
    return;
}
int main() {
    double start_time=clock();
    const char* visual_dir = "/home/suayu/SurroundOcc/visual_dir/";
    listDirectories(visual_dir);
    float voxsize=1.0;
    std::cout<<"Please input barrier edge restricted area size:"<<std::endl;
    std::cin>>voxsize;
    char red_flag;
    std::cout<<"Display a red dot to mark the center position of EGO CAR? [y/n]"<<std::endl;
    std::cin>>red_flag;
    // 读取PCD点云地图
    //fs::path currentPath = fs::current_path();
    int count=0;
    for(auto file_path:filedir_name_list){
        //std::cout<<file_path<<std::endl;
        std::string pcd_file=visual_dir+file_path;

        //std::string pcd_file = currentPath;
        PointCloud::Ptr cloud(new PointCloud);
        if (pcl::io::loadPCDFile<PointT>(pcd_file+"/pred.pcd", *cloud) == -1) {
            std::cerr << "Unable to read PCD file." << std::endl;
            return -1;
        }

        std::ofstream file("transData.txt",std::ios::trunc);

        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.1f,0.1f,0.1f);

        PointCloud::Ptr filtered_cloud (new PointCloud);
        voxel_grid.filter(*filtered_cloud);

        // 计算点云的投影尺寸
        float max_x = -std::numeric_limits<float>::max();
        float min_x = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        for (const auto& point : cloud->points) {
            max_x = std::max(max_x, point.x);
            min_x = std::min(min_x, point.x);
            max_y = std::max(max_y, point.y);
            min_y = std::min(min_y, point.y);
        }
        int width = std::ceil(max_x - min_x) + 1;
        int height = std::ceil(max_y - min_y) + 1;
        file<<min_x<<"  "<<min_y<<"  "<<max_x<<" "<<max_y<<std::endl;
        file.close();
        // 创建高分辨率的图像
        //cv::Mat image(height * 10, width * 10, CV_8UC3, cv::Scalar(0,0,0));
        //std::cout<<height * 10<<" "<<width * 10<<std::endl;
        float imgsize=200*voxsize;
        cv::Mat image(imgsize,imgsize, CV_8UC3, cv::Scalar(0,0,0));
        cv::Vec3b color = cv::Vec3b(0, 0, 255); 

        PointCloud::Ptr cloud_driveable(new PointCloud);
        PointCloud::Ptr cloud_undriveable(new PointCloud);
        PointCloud::Ptr cloud_ego_car(new PointCloud);

        // 投影点云到二维平面，并根据深度信息设置点的颜色
        //Warning!The keyword CONST is not used here,which may corrupt the point cloud data have read.
        struct RGB rgb_list[21];
        int len=0;
        for (auto& point : cloud->points) {
            std::uint32_t rgb = *reinterpret_cast<int*>(&point.rgb);
            std::uint8_t r=(rgb>>16)&0x0000ff;
            std::uint8_t g=(rgb>>8)&0x0000ff;
            std::uint8_t b=(rgb)&0x0000ff;
            RGB rgb_={r,g,b};
            if(find_rgblist(rgb_list,len,rgb_)==-1){
                rgb_list[len]=rgb_;
                len++;

                //std::cout<<(int)r<<" "<<(int)g<<" "<<(int)b<<std::endl;
            }

            if(r==255&&g==0&&b==255){
                cloud_driveable->push_back(point);
            }
            else if(r==127&&g==255&&b==0){
                cloud_ego_car->push_back(point);
            }
            else{
                cloud_undriveable->push_back(point);
                
            }
        }

        for (auto& point : cloud_driveable->points) {
            // pointIndex ++;
            float x = (point.x - min_x) * 10;
            float y = (point.y - min_y) * 10;
            float depth = point.z;
            /*
            std::cout<<"X:"<<x<<" Y:"<<y<<" Z:"<<depth<<std::endl;
            std::cout<<point<<std::endl;
            std::uint32_t rgb = *reinterpret_cast<int*>(&point.rgb);
            std::uint8_t r=(rgb>>16)&0x0000ff;
            std::uint8_t g=(rgb>>8)&0x0000ff;
            std::uint8_t b=(rgb)&0x0000ff;
            */
            // 设置点的颜色
            //color = cv::Vec3b(r, g, b);
            color = cv::Vec3b(255,255,255); 
            // 在图像上绘制点
            //std::cout<<x - voxsize*1.0/2<<" "<<height * 10 - y - voxsize*1.0/2<<std::endl;
            //std::cout<<point.x<<" "<<point.y<<std::endl;
            //cv::rectangle(image, cv::Rect(x - voxsize*1.0/2, height * 10 - y - voxsize*1.0/2, voxsize, voxsize), color, cv::FILLED);
            cv::rectangle(image,cv::Rect(imgsize/2+point.x*voxsize-voxsize/2,imgsize/2-point.y*voxsize+voxsize/2,voxsize,voxsize),color,cv::FILLED);
        }
        for (auto& point : cloud_undriveable->points) {
            float x = (point.x - min_x) * 10;
            float y = (point.y - min_y) * 10;
            float depth = point.z;
            // 设置点的颜色
            color = cv::Vec3b(0, 0, 0); 
            // 在图像上绘制点
            //cv::rectangle(image, cv::Rect(x - voxsize*1.0/2, height * 10 - y - voxsize*1.0/2, voxsize, voxsize), color, cv::FILLED);
            cv::rectangle(image,cv::Rect(imgsize/2+point.x*voxsize-voxsize/2,imgsize/2-point.y*voxsize+voxsize/2,voxsize,voxsize),color,cv::FILLED);
        }
        //In fact,there are no EGO CAR in the prediction point cloud
        /*
        for (auto& point : cloud_ego_car->points) {
            float x = (point.x - min_x) * 10;
            float y = (point.y - min_y) * 10;
            float depth = point.z;
            // 设置点的颜色
            color = cv::Vec3b(127, 255, 0); 
            // 在图像上绘制点
            cv::rectangle(image, cv::Rect(x - voxsize*1.0/2, height * 10 - y - voxsize*1.0/2, voxsize, voxsize), color, cv::FILLED);
        }
        */
        if(red_flag=='y')
            cv::rectangle(image, cv::Rect(imgsize/2-voxsize/2,imgsize/2-voxsize/2,voxsize,voxsize), cv::Vec3b(0, 0,255), cv::FILLED);//draw a red point
        // 保存结果为.png文件
        std::string output_file = "output_image/image_"+std::to_string(count)+".png";
        cv::imwrite(output_file, image);
        std::cout << "Saved image: " << output_file << std::endl;
        count++;

    }
    
    double end_time=clock();
    double duration=start_time-end_time;
    std::cout<<start_time<<" "<<end_time<<" Duration Time: "<<duration<<" ms"<<std::endl;
    return 0;
}
