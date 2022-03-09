///对map.h中声明的各类函数进行定义实现

#include "myslam/map.h"
#include "myslam/feature.h"

namespace myslam
{
    void Map::RemoveOldKeyframe()
    {
        if (current_frame_ == nullptr)
            return;
        // 寻找与当前帧最近与最远的两个关键帧
        // distance
        double max_dis = 0, min_dis = 9999;
        // 注意这里指的是id
        double max_kf_id = 0, min_kf_id = 0;
        auto Twc = current_frame_->Pose().inverse(); // 当前帧位姿的逆矩阵
        for (auto &kf : active_keyframes_)           // 找到了与当前帧位姿相差最大与最小的点
        {
            if (kf.second == current_frame_)
                continue;
            auto dis = (kf.second->Pose() * Twc).log().norm();
            if (dis > max_dis)
            {
                max_dis = dis;
                max_kf_id = kf.first;
            }
            if (dis < min_dis)
            {
                min_dis = dis;
                min_kf_id = kf.first;
            }
        }

        const double min_dis_th = 0.2; // 最近阈值
        Frame::Ptr frame_to_remove = nullptr;
        if (min_dis < min_dis_th)
        {
            // 如果存在很近的帧，优先删掉最近的
            frame_to_remove = keyframes_.at(min_kf_id);
        }
        else
        {
            // 删掉最远的
            frame_to_remove = keyframes_.at(max_kf_id);
        }
        // 把移除的关键帧记录到日志
        LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;

        // remove keyframe and landmark observation
        // 激活关键帧容器中移除这个关键帧
        active_keyframes_.erase(frame_to_remove->keyframe_id_);

        // 在这个关键帧中得到的所有特征也要移除，这样能够保持BA中H矩阵的稀疏性
        // 为了避免shared_ptr指针的争用引起计数问题，使用指针自带的lock()函数拷贝指针。
        // 参考shared_ptr使用注意事项
        for (auto feat : frame_to_remove->features_left_)
        {
            auto mp = feat->map_point_.lock();
            if (mp)
            {
                mp->RemoveObservation(feat);
            }
        }
        for (auto feat : frame_to_remove->features_right_)
        {
            if (feat == nullptr)
                continue;
            auto mp = feat->map_point_.lock();
            if (mp)
            {
                mp->RemoveObservation(feat);
            }
        }

        CleanMap();
    }

    // 在移除了关键帧及其包含的特征点之后，可能会存在一些路标点没有被观测过的，
    // 但是仍然留存在激活路标点容器中，需要将它们删除
    void Map::CleanMap()
    { // RemoveOldKeyframe操作过后，有些帧被舍弃了，里面的特征观测也全部被丢弃，这样就可能造成有些landmark没有观测了
        int cnt_landmark_removed = 0;
        for (auto iter = active_landmarks_.begin();
             iter != active_landmarks_.end();)
        {
            if (iter->second->observed_times_ == 0)
            { //这里的意思是哪个landmark的观测次数等于0,则将这个landmark从active_landmarks_中移除
                iter = active_landmarks_.erase(iter);
                // 删除后计数，但不要进入下次循环，因为删除之后这个指针指向的已经是下一个元素
                cnt_landmark_removed++;
            }
            else
            {
                ++iter;
            }
        }
        LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
    }

    void Map::InsertKeyFrame(Frame::Ptr frame)
    {
        // typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;
        //根据Map类中定义的这个unordered_map容器，我们可知插入一个关键帧需要分配给其一个容器内的索引值

        // map和viewer两个类里都有一个叫做current_frame_的私有成员变量，这里要注意和Frontend里面的区别
        current_frame_ = frame;

        // map容器中没有找到这个关键帧的id，则插入它，并激活
        if (keyframes_.find(frame->keyframe_id_) == keyframes_.end())
        {
            //如果key存在，则find返回key对应的迭代器，如果key不存在，则find返回unordered_map::end
            //说明这个要插入的关键帧在容器内原先不存在，需插入
            // std::unordered_map<unsigned long, Frame::Ptr> 的方法insert
            keyframes_.insert(make_pair(frame->keyframe_id_, frame));

            //插入原先不存在的一个关键帧就是在时间上插入了一个最新的关键帧，因此这个关键帧应该放入active_keyframes中
            active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        }
        // 容器中有了这个关键帧：在关键帧和激活关键帧的容器中更新帧
        else
        {
            //找到了这个关键帧编号，则以当前要插入的关键帧覆盖掉原有内容
            keyframes_[frame->keyframe_id_] = frame;
            active_keyframes_[frame->keyframe_id_] = frame;
        }

        // 最后如果发现激活关键帧太多了，移除旧关键帧
        if (active_keyframes_.size() > num_active_keyframes_)
        {
            RemoveOldKeyframe(); // （成员函数互为友元，不必在意顺序）
        }
    }

    // 插入地图点，无则增加并激活，有则更新
    void Map::InsertMapPoint(MapPoint::Ptr map_point)
    { //与插入关键帧同理
        if (landmarks_.find(map_point->id_) == landmarks_.end())
        {
            landmarks_.insert(make_pair(map_point->id_, map_point));
            active_landmarks_.insert(make_pair(map_point->id_, map_point));
        }
        else
        {
            landmarks_[map_point->id_] = map_point;
            active_landmarks_[map_point->id_] = map_point;
        }
    }
}