#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "myslam/common_include.h"

#include "myslam/basicStruct/frame.h"

#include "myslam/map.h"
#include "myslam/camera.h"
#include "myslam/viewer.h"

// 基本操作单元：某一帧Frame

// 选定前端策略：
// 初始化：左右目的照片光流法匹配，并三角化。（注意到普通三角化得到的坐标是左图的相机坐标系坐标）
// 追踪：只使用左目，光流法匹配，然后估计位姿
// 追踪到的点很少，说明两帧之间差异较大，判定当前帧为关键帧，对于关键帧：
//  ① 提取新特征点。 ② 计算这些新特征点的空间坐标（路标点）：利用右图三角化。
//  ③ 将新的关键帧和路标点加入地图，触发一次后端优化。
// 如果追踪失败，重置前端系统，重新初始化。

namespace myslam
{
    enum class FrontendStatus
    {
        INITING,
        TRACKING_GOOD,
        TRACKING_BAD,
        LOST
    };

    class Frontend
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frontend> Ptr;

        // VO初始化时frontend的类成员配置，被VisualOdometry::Init()调用
        // 构造函数：OpenCV检测器等特征数目的成员变量（根据Config读取yaml的字段）初始化
        Frontend();
        // 其他frontend类成员赋值
        void SetMap(Map::Ptr map) { map_ = map; }
        void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }
        void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }
        void SetCameras(Camera::Ptr left, Camera::Ptr right) //设定相机参数
        {
            camera_left_ = left;
            camera_right_ = right;
        }

        // 将VO初始化后持有的dataset_->NextFrame()新的一帧加入到前端中，被VisualOdometry::Step()调用
        // 根据目前的状况选择不同的处理函数frontend的private方法： StereoInit() / Track() / Reset()
        bool AddFrame(Frame::Ptr frame);

        // TODO 没用过？
        FrontendStatus GetStatus() const { return status_; }

    private:
        // VO初始化的时候，调用frontend公有接口配置的类成员变量
        // TODO 为什么不能写成一样的形式？？
        // std::shared_ptr<Backend> backend_ = nullptr;
        // std::shared_ptr<Viewer> viewer_ = nullptr;
        Map::Ptr map_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;
        Camera::Ptr camera_left_ = nullptr;  // 左侧相机，参数情况
        Camera::Ptr camera_right_ = nullptr; // 右侧相机，参数情况

        // frontend的构造赋值OpenCV特征相关的参数
        // feature detector in opencv
        cv::Ptr<cv::GFTTDetector> gftt_;
        // TODO 另法
        // cv::Ptr<cv::FastFeatureDetector> gftt_;

        // params
        int num_features_ = 200;
        int num_features_init_ = 100;
        int num_features_tracking_ = 50;
        int num_features_tracking_bad_ = 20;
        int num_features_needed_for_keyframe_ = 80;
        int tracking_inliers_ = 0; // inliers, used for testing new keyframes

        // frontend运行过程中访问的类成员变量
        FrontendStatus status_ = FrontendStatus::INITING; // 前端构造完之后默认为INITING状态
        SE3 relative_motion_;                             // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

        // Frontend::AddFrame调用
        Frame::Ptr current_frame_ = nullptr; // 当前帧
        Frame::Ptr last_frame_ = nullptr;    // 上一帧

        /**
         * Try init the frontend with stereo images saved in current_frame_ (FrontendStatus::INITING)
         * 提取左目特征
         * 根据左目特征在右目找对应
         * 对应数目足够
         *      建立初始地图
         *      写入前端TRACKING_GOOD
         *      当前帧加入可视化
         *      更新可视化内容
         * @return true if success
         */
        bool StereoInit();

        /**
         * Track in normal mode (FrontendStatus::TRACKING_GOOD + TRACKING_BAD)
         * 上一帧存在
         *      估计当前帧的位置初值
         * 光流法匹配上一帧与当前帧
         * 优化当前帧的精确位置，返回追踪到的内点数目
         * 内点数目分状态(写入前端TRACKING_GOOD BAD LOST)
         * 插入关键帧
         * 计算当前帧与上一帧的相对运动（给下一个循环用）
         * 当前帧加入可视化
         * @return true if success
         */
        bool Track();

        /**
         * Reset when lost (FrontendStatus::LOST)
         * 写入前端TRACKING_INITING
         * @return true if success
         */
        bool Reset();

        /**
         * Detect features in left image in current_frame_
         * keypoints will be saved in current_frame_
         * @return
         */
        int DetectFeatures(); // current_frame_->features_left_

        /**
         * Find the corresponding features in right image of current_frame_
         * @return num of features found
         */
        int FindFeaturesInRight(); // current_frame_->features_right_

        /**
         * Build the initial map with single image
         * @return true if succeed
         */
        bool BuildInitMap();

        /**
         * Track with last frame
         * @return num of tracked points
         */
        int TrackLastFrame();

        /**
         * estimate current frame's pose
         * @return num of inliers
         */
        int EstimateCurrentPose();

        /**
         * set current frame as a keyframe and insert it into backend
         * @return true if success
         */
        bool InsertKeyframe();

        /**
         * Triangulate the 2D points in current frame
         * @return num of triangulated points
         */
        int TriangulateNewPoints();

        /**
         * Set the features in keyframe as new observation of the map points
         */
        void SetObservationsForKeyFrame();
    };
}

#endif