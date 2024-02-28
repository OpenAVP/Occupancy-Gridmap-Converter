# Occupancy-Gridmap-Converter
C++实现pcd点云转换为二维栅格地图
# Usage

读取SurroundOcc的占用点云预测结果，然后将转换后的地图输出在output_image文件夹中

环境需求：
pcl opencv

构建：
cmake -B .

编译：
make

运行：
./pcd2jpg

使用方法：
1.在pcd2jpg.cpp中，修改visual_dir的值，改为自己电脑上SurroundOcc可视化预测结果的输出目录，如"/home/suayu/SurroundOcc/visual_dir/"
2.重新编译代码并生成可执行文件pcd2jpg
3.运行可执行文件

# 执行过程
程序显示“Please input barrier edge restricted area size：”，要求输入栅格大小。
SurroundOcc生成的预测尺寸默认为200×200，虽然实际生成的点云尺寸往往小于这个尺寸。
栅格大小即为点云尺寸→栅格地图尺寸的映射比例。若输入值为1，则生成的栅格地图尺寸为200×200像素，若输入值为5，则生成的栅格地图尺寸为1000×1000像素。

程序显示“Display a red dot to mark the center position of EGO CAR? [y/n]”，要求输入y或者n。
输入y会在生成的每一帧栅格图像中间，即原点云坐标(0,0)处绘制一个边长为一栅格长度的红点，标示自车中心位置。

另：此版本中时间统计功能并不可用，时间统计功能也不是主要功能，相关显示信息忽略即可。





