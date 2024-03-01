# Occupancy-Gridmap-Converter
C++ implementation for converting PCD point clouds to 2D grid maps
# Usage

Read SurroundOcc's occupancy point cloud prediction results and output the converted map to the `output_image` folder.

Environment requirements:
pcl opencv

Build:
cmake -B .

Compile:
make

Run:
./pcd2jpg

Instructions:
1. In `pcd2jpg.cpp`, modify the value of `visual_dir` to the output directory of SurroundOcc's visualization prediction results on your computer, such as "/home/suayu/SurroundOcc/visual_dir/".
2. Recompile the code and generate the executable file `pcd2jpg`.
3. Run the executable file.

# Execution Process
The program displays "Please input barrier edge restricted area size:" and asks for the grid size.
The default prediction size generated by SurroundOcc is 200×200, although the actual generated point cloud size is often smaller than this size.
The grid size is the mapping ratio from the point cloud size to the grid map size. If the input value is 1, the generated grid map size is 200×200 pixels; if the input value is 5, the generated grid map size is 1000×1000 pixels.

The program displays `Display a red dot to mark the center position of EGO CAR? [y/n]` and asks for input `y` or `n`.
Entering `y` will draw a red dot with a side length equal to one grid cell at the center of each frame of the generated grid map, marking the center position of the ego car.

Note: The time statistics function is not available in this version, and it is not a primary feature. You can ignore the related display information.





