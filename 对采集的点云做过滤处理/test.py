'''
作者：callmeplus@github.com
时间：2018年5月5日
注：将当前目录下的所有pcd点云文件进行过滤处理，处理方式如下：
    1）使用statisticalOutlierRemoval滤波器移除离群点
    2）使用VoxelGrid滤波器对点云进行下采样
    1）将原始点云以及处理结果做ply转换，以便在MeshLab中查看
    处理程序均使用pcl已经编译好的可执行文件（见callmeplus@github.com的
'''
import os
import subprocess

log_file = open('log','w+')

def file_name(file_dir,suffix_name):
    L=[]
    for root, dirs, files in os.walk(file_dir):
        #print(files) #当前路径下所有非目录子文件
        for file in files:
            if os.path.splitext(file)[1] == suffix_name:
                L.append(file)
        break
    return L

def pcl_outlier_removal(filename,count):
    real_filename = os.path.splitext(filename)[0]
    suffix_filename = os.path.splitext(filename)[1]

    if not os.path.exists(real_filename):
        os.mkdir(real_filename)

    dest_name = real_filename+'_outlier'+suffix_filename
    command = \
        'pcl_outlier_removal_release\
        -method statistical -mean_k 50 -std_dev_mul 0.5\
        %s %s' % (filename,real_filename+'\\'+dest_name)
    print(command)
    subprocess.call(command,stderr=log_file,stdout=log_file)
    print('[%d]正在将点云(%s)转换为ply格式...' % (count,dest_name))
    pcl_pcd2ply(real_filename,real_filename,real_filename+'_outlier'+suffix_filename);

def pcl_voxel_grid(filename,count):
    real_filename = os.path.splitext(filename)[0]
    suffix_filename = os.path.splitext(filename)[1]

    if not os.path.exists(real_filename):
        os.mkdir(real_filename)

    src_name = real_filename+'\\'+real_filename+'_outlier'+suffix_filename
    dest_name = real_filename +'_grid'+suffix_filename
    command = \
        'pcl_voxel_grid_release\
        -leaf 0.01f\
        %s %s' % (src_name, real_filename + '\\' + dest_name)
    print(command)
    subprocess.call(command,stderr=log_file,stdout=log_file)
    print('[%d]正在将点云(%s)转换为ply格式...' % (count,dest_name))
    pcl_pcd2ply(real_filename,real_filename,real_filename+'_grid'+suffix_filename);

def pcl_pcd2ply(src_dirname,dst_dirname,filename):
    real_filename = os.path.splitext(filename)[0]

    src_name = src_dirname +'\\'+filename
    dst_name = dst_dirname +'\\'+real_filename+'.ply'

    command = \
        'pcl_pcd2ply_release\
        %s %s' % (src_name, dst_name)
    print(command)
    subprocess.call(command,stderr=log_file,stdout=log_file)

if __name__ == '__main__':
    # 查找当前目录下的pcd文件
    files = file_name('./','.pcd')
    if files == []:
        print('error:当前目录下没有.pcd文件')
        exit(-1)
    # 创建文件夹
    for file in files:
        real_filename = os.path.splitext(file)[0]
        if not os.path.exists(real_filename):
            os.mkdir(real_filename)

    # 开始处理点云
    count = 0
    for file in files:
        real_filename = os.path.splitext(file)[0]
        # 将原始点云转换为ply，以便在MeshLab中查看
        print('[%d]正在将原始点云(%s)转换为ply格式...'%(count,file))
        pcl_pcd2ply('.',real_filename,file)
        # 处理pcd点云，使用statisticalOutlierRemoval滤波器移除离群点
        print('[%d]正在将原始点云(%s)移除离群点...'%(count,file))
        pcl_outlier_removal(file,count)
        # 使用VoxelGrid滤波器对点云进行下采样
        print('[%d]正在将原始点云(%s)进行下采样...' % (count,file))
        pcl_voxel_grid(file,count)
        count = count +1

    log_file.close()