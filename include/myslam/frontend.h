#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/camera.h"

namespace myslam{
    enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

    class Frontend{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frontend> Ptr;

        Frontend();

        bool AddFrame(Frame::Ptr frame);//依次在源文件中实现各个成员函数，addframe是第一个

        void SetMap(Map::Ptr map) { map_ = map; }

        void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

        void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

        FrontendStatus GetStatus() const { return status_; }

        void SetCameras(Camera::Ptr left, Camera::Ptr right) {   //这里应该是为了设定相机参数
            camera_left_ = left;
            camera_right_ = right;
        }

    private:
        FrontendStatus status_ = FrontendStatus::INITING;
        Frame::Ptr current_frame_ = nullptr;  // 当前帧
        Frame::Ptr last_frame_ = nullptr;     // 上一帧
        Camera::Ptr camera_left_ = nullptr;   // 左侧相机，参数情况
        Camera::Ptr camera_right_ = nullptr;  // 右侧相机，参数情况

        Map::Ptr map_ = nullptr;

        std::shared_ptr<Backend> backend_ = nullptr;
        std::shared_ptr<Viewer> viewer_ = nullptr;

        SE3 relative_motion_;  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

        // params
        int num_features_ = 200;
        int num_features_init_ = 100;
        int num_features_tracking_ = 50;
        int num_features_tracking_bad_ = 20;
        int num_features_needed_for_keyframe_ = 80;
        int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

        cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv
        cv::Ptr<cv::FeatureDetector> orb_detector = ORB::create();


        /**
     * Track in normal mode
     * @return true if success
     */
        bool Track();

        /**
         * Reset when lost
         * @return true if success
         */
        bool Reset();

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
         * Try init the frontend with stereo images saved in current_frame_
         * @return true if success
         */
        bool StereoInit();

        /**
         * Detect features in left image in current_frame_
         * keypoints will be saved in current_frame_
         * @return
         */
        int DetectFeatures();

        /**
         * Find the corresponding features in right image of current_frame_
         * @return num of features found
         */
        int FindFeaturesInRight();

        /**
         * Build the initial map with single image
         * @return true if succeed
         */
        bool BuildInitMap();

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