#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam
{

    struct Frame; // TODO 这里为什么要声明Frame？应该是可有可无的？
    struct Feature;

    /**
     * 路标点类
     * 特征点在三角化之后形成路标点
     */

    struct MapPoint
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<MapPoint> Ptr;

        unsigned long id_ = 0;    // 路标点id
        bool is_outlier_ = false; // 是否是异常点
        std::mutex data_mutex_;
        int observed_times_ = 0; // 记录这个路标点被关键帧观察（提取）到的次数// being observed by feature matching algo.

        Vec3 pos_ = Vec3::Zero();                        // 路标点世界坐标，初始值为0,0,0
        std::list<std::weak_ptr<Feature>> observations_; // 路标点观测到的feature的vector

        MapPoint() {}

        MapPoint(long id, Vec3 position);

        // 取、改路标点，前后端都访问：需要上锁
        Vec3 Pos()
        {
            /*
             * 在unique-lock对象的声明周期内，它所管理的锁对象会一直保持上锁的状态，
             * 而unique-lock的声明周期结束后，他所管理的对象会被解锁。
             * 因此由unqie-lock托管的互斥锁就不必考虑它的解锁操作了
             */
            std::unique_lock<std::mutex> lck(data_mutex_);
            return pos_;
        }

        void SetPos(const Vec3 &pos)
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            pos_ = pos;
        };

        // 增加新的特征点到这个路标点，并且特征点数量+1
        void AddObservation(std::shared_ptr<Feature> feature)
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            observations_.push_back(feature);
            observed_times_++;
        }

        // 取出特征点存储的链表
        std::list<std::weak_ptr<Feature>> GetObs()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return observations_;
        }

        // 工厂模式构建路标点 // factory function
        //静态函数不需要对象就可以直接调用
        static MapPoint::Ptr CreateNewMappoint();

        // 可能是异常点，也可能将要删除某个关键帧，所以要移除某个特征点，并且特征点数量-1
        void RemoveObservation(std::shared_ptr<Feature> feat);
    };
}

#endif // MYSLAM_MAPPOINT_H