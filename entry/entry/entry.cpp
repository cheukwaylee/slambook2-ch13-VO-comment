

///在include和src文件夹中，我们已经定义并实现了各种类及函数，现在我们只需要在本文件中为整个视觉里程计VO提供一个入口即可

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

// gflags库的使用说明参考： https://www.jianshu.com/p/2179938a818d
//同上：https://blog.csdn.net/NMG_CJS/article/details/104436079?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-3.control&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-3.control

DEFINE_string(config_file, "../../config/default.yaml", "config file path"); //第三个参数是说明信息

int main(int argc, char **argv)
{
    // 用来解析命令行参数，默认argv是程序运行时的当前路径，因此此处应该是保证能够寻找到配置文件
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 创建vo类，断言初始化成功，运行
    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(FLAGS_config_file));
    // std::shared_ptr<VisualOdometry> vo(new myslam::VisualOdometry(FLAGS_config_file))
    // 调用的是std::shared_ptr的构造函数

    //构造一个VO类对象，这里其实FLAGS_config_file也可以用argv[]里面的内容来替代。
    assert(vo->Init() == true); //这里虽然是一个判断语句，但是在该过程中已然完成了VO类对象vo的初始化
    vo->Run();                  //把视觉里程计跑起来吧！

    return 0;
}

/*
TODO
    智能指针
    线程锁
    工厂模式

*/

/*
参数类：config.h dataset.h camera.h
特征类：frame.h feature.h mappoint.h map.h
系统类：frontend.h backend.h viewer.h
启动类：visual_odometry.h

common_include.h 预先定义了可能用到的数据类型，在每个头文件中都会引用它，避免重复定义。

config.h 采用键值对的形式灵活获取配置文件中的内容，单例模式构造对象。

dataset.h 初始化相机内外参，根据数据集路径来获取每一帧的图像。

camera.h 定义像素的、相机和世界三个坐标系下点的变换。

frame.h 创建新帧，设置关键帧。

feature.h 检测到的2D特征点。

mappoint.h 2D特征点对应的地图/路标点。

map.h 地图类，增加关键帧并控制地图/路标点的产生和剔除，将其交给前后端进行相应处理。

algorithm.h 三角化2D特征点得到地图/路标点。

g2o_types.h 预先定义优化中需要使用的数据类型。

frontend.h 初始化地图点和关键帧，采用光流法追踪相邻帧间特征点，采用g2o优化得到当前帧位姿Tcw，插入关键帧交给后端优化。

backend.h 优化关键帧的位姿和路标点的位置。

viewer.h 使用pangolin进行可视化展示。

visual_odometry.h 加载配置和数据，初始化各个模块，系统启动。

*/