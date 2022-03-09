#include <chrono>
#include "myslam/config.h"

#include "myslam/visual_odometry.h"

namespace myslam
{

    VisualOdometry::VisualOdometry(std::string &config_path)
        : config_file_path_(config_path) {}

    bool VisualOdometry::Init()
    {
        // 配置文件打开成功
        // 把VO类的成员变量config_path写入到 config_->file_
        if (Config::SetParameterFile(config_file_path_) == false) // "../config/default.yaml"
        {
            //判断一下这个config_file_path_是否存在，同时将配置文件赋给Config类中的cv::FileStorage file_,便于对文件操作
            return false;
        }

        //数据集类初始化，读相机参数
        dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
        // dataset_dir 是yaml文件里面的字段设置的数据集路径
        // 以Config::Get<std::string>("dataset_dir") 为参数 即：std::string(Config::config_->file_["dataset_dir"])
        // 调用Dataset的构造：接受一个const std::string &dataset_path 写入自己的成员变量dataset_path_
        // dataset_.dataset_path_ = std::string(Config::config_->file_["dataset_dir"]) /home/cw/code/slam_learning/data_odometry_gray_ch13/05
        // 把config_->file_里面的["dataset_dir"]字段写入到dataset_.dataset_path_
        // 1.模板函数自动类型推导调用Config::Get("dataset_dir")；
        // 2.模板函数具体类型显示调用Config::Get<std::string>("dataset_dir")；

        // 功能类似assert断言，断言数据集初始化成功，但不受DEBUG模式控制即非DEBUG模式也生效
        // 读取/home/cw/code/slam_learning/data_odometry_gray_ch13/05/calib.txt!里面四台相机的参数
        // 调用Camera类的构造将相机参数写入，然后把相机类对象的地址push_back到dataset的成员变量，由它持有
        CHECK_EQ(dataset_->Init(), true);

        // 创建各个模块，并构建它们之间的联系或依赖关系
        // 接下来按照逻辑关系一层层的确立联系，一个完整的VO包含前端,后端,地图,可视化器等模块，因此有下述创建代码
        frontend_ = Frontend::Ptr(new Frontend);
        // 后端初始化时生成backend_指针，并且通过Backendloop函数创建线程并启动，但随后便进入map_update_.wait(lock)，被阻塞。
        // TODO std::shared_ptr<Backend>(new Backend) 可以直接new然后赋值吗？前面的是类型转换？ new出来应该是Backend* 要转为智能指针
        backend_ = Backend::Ptr(new Backend);

        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);

        //在一个VO中，前端需对应后端,地图,可视化器,相机类等,这在frontend的类定义中有清楚显示，所以将它们连接起来
        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

        //后端类的定义中用到了相机类和地图类，所以要将后端类与相机类和地图类连接起来
        backend_->SetMap(map_);
        backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

        //对于可视化器来说，只要有地图就可以，它只是将地图可视化，所以不需要其它模块，只需将其与地图模块连接在一起
        viewer_->SetMap(map_);

        return true;
    };

    void VisualOdometry::Run()
    {
        while (1) // 循环执行步进函数
        {
            LOG(INFO) << "VO is running";
            if (Step() == false)
            {
                //这里的主过程执行在这条if语句中,每次做条件判断都需要执行Step()，
                //即步进操作，如果步进出问题，则跳出死循环while(1)
                break;
            }
        }

        // 步进失败了退出死循环，运行出错，则停止后端，关闭可视化器，
        // 同时因为跳出死循环，所以不再执行step(),所以前端也不再对图像序列进行步进跟踪
        backend_->Stop();
        viewer_->Close();

        LOG(INFO) << "VO exit";
    };

    bool VisualOdometry::Step() // 读取数据集，并向前端添加帧
    {
        Frame::Ptr new_frame = dataset_->NextFrame(); //从数据集中读出下一帧
        if (new_frame == nullptr)
            return false; //如果读到的下一帧为空，也就是说没有读到下一帧，则无法继续跟踪，报错

        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->AddFrame(new_frame); //将新的一帧加入到前端中，进行跟踪处理,帧间位姿估计
        // TODO 验证第一针初始化是否成功
        // if (!success)
        //     return false;
        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";

        return true;
    }
}