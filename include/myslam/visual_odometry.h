#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/common_include.h"

#include "myslam/frontend.h"
#include "myslam/backend.h"

#include "myslam/dataset.h"

#include "myslam/viewer.h"

namespace myslam
{

    class VisualOdometry
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;

        // 构造函数，参数是配置文件路径
        /// constructor with config file
        VisualOdometry(std::string &config_path);

        //初始化
        bool Init();

        //在特定数据集上启动VO
        void Run(); // 运行VO

        //在数据集图像序列上步进
        bool Step(); // 步进到下一帧

        //获取前端状态
        // TODO 没用过？
        FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

    private:
        // TODO 没用过？
        bool inited_ = false;

        // 构造赋值 "../config/default.yaml"
        std::string config_file_path_;

        // init赋值
        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;
        Dataset::Ptr dataset_ = nullptr;
    };
}

#endif // MYSLAM_VISUAL_ODOMETRY_H