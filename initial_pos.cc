#include <filesystem> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <cmath>
#include <fstream>
#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
std::string file_path="/home/wdd/6_localization_fast/gen_submap/result/";
int ExtractSide=25;
//读取pcd点云
//根据x,y边界，获取子图坐标
//根据坐标，提取子图，按索引保存坐标信息到.csv文件，按索引保存点云数据到bin文件8
void gen_submap(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//原始点云
    //pcl::io::loadPCDFile(file_path+"global_map.pcd", *cloud);
    pcl::io::loadPCDFile(file_path+"noground.pcd", *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sub_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::VoxelGrid<pcl::PointXYZ> vg;
    // vg.setInputCloud(cloud);
    // vg.setLeafSize(0.3, 0.3, 0.3);
    // vg.filter(*cloud);
    // pcl::io::savePCDFileBinary(file_path+"output_cloud_binary.pcd", *cloud);

    double x_min = -7.5;
    double x_max = 4.5;
    double y_min = -0.5;
    double y_max = 16.5;
    double step = 0.5;
    ofstream pos_file("/home/wdd/6_localization_fast/gen_submap/result/grid_data.csv"); 

    int index=0;
    std::cout<<index<<std::endl;
    for (double x = x_min; x <= x_max; x += step) {
        for (double y = y_min; y <= y_max; y += step) {
            index++;
            std::cout<<index<<std::endl;
            if(pos_file.is_open()){
                pos_file << setw(6) << setfill('0') << index << ",";
                pos_file << x << "," << y << endl;
            }else{
                cout << "pos打开失败" << endl;
            }


            stringstream ss;
            ss << "/home/wdd/6_localization_fast/gen_submap/result/sub_clouds/";  
            ss << setw(6) << setfill('0') << index;
            string filename = ss.str() + ".bin";
            //生成子图
            for(int i=0;i<cloud->points.size();i++){
                pcl::PointXYZ point=cloud->points[i];
                point.x+=x;
                point.y+=y;
                if( point.x>ExtractSide || point.y>ExtractSide ||  point.x<-ExtractSide || point.y<-ExtractSide) {
                    continue;
                }else{
                    sub_cloud->points.push_back(point);
                }
            }
            ofstream sub_point(filename, ios::out | ios::binary);
            if (sub_point.is_open()) {
                sub_point.write(reinterpret_cast<const char*>(sub_cloud->points.data()), sub_cloud->points.size() * sizeof(pcl::PointXYZ));
                sub_point.close();
            }else{
                cout << "bin打开失败" << endl;
            }
        }
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "run_initial_align");
    ros::NodeHandle nh;
    gen_submap();
    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}



