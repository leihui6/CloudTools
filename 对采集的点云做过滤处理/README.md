## 注：
.pcd点云文件源于 KinectV2

>> 处理内容如下：
>>
>> 1）使用statisticalOutlierRemoval滤波器移除离群点
>>
>> 2）使用VoxelGrid滤波器对点云进行下采样
>>
>> 1）将原始点云以及处理结果做ply转换，以便在MeshLab中查看

处理程序均使用pcl已经编译好的可执行文件


