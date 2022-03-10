#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"

#include "myslam/basicStruct/frame.h"

#include "myslam/map.h"
#include "myslam/camera.h"

/*
多线程编程中，还有另一种十分常见的行为：线程同步。线程同步是指线程间需要按照预定的先后次序顺序进行的行为。
C++ 11对这种行为也提供了有力的支持，这就是条件变量。条件变量位于头文件condition_variable下。

    条件变量提供了两类操作：wait和notify。这两类操作构成了多线程同步的基础。

    wait：wait是线程的等待动作，直到其它线程将其唤醒后，才会继续往下执行。
    notify：唤醒wait在该条件变量上的线程。
    notify_one：notify_one 唤醒等待的一个线程。
    notify_all 唤醒所有等待的线程。

    我的理解：获取锁之后，保持持有锁，但是不继续往下执行，等待被唤醒时再继续。
*/

/**
 * 后端
 * 有单独优化线程，在Map更新时启动优化
 * Map更新由前端触发
 */

namespace myslam
{
    // TODO 为什么要声明？
    // class Map;

    class Backend
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Backend> Ptr;

        /// 构造函数中启动优化线程并挂起
        Backend();

        // 设置左右目的相机，用于获得内外参
        void SetCameras(Camera::Ptr left, Camera::Ptr right)
        {
            cam_left_ = left;
            cam_right_ = right;
        }

        /// 设置地图，让backend自己的地图指针指向当前的地图，而不是对当前地图进行修改，不需要锁
        void SetMap(std::shared_ptr<Map> map) { map_ = map; }

        // 触发地图更新，启动优化（notify），主要应该由前端触发，当追踪点少时，添加关键帧并触发更新地图
        void UpdateMap();

        /// 关闭后端线程
        void Stop();

    private:
        /// 后端线程
        void BackendLoop();

        /// 对给定关键帧和路标点进行优化
        void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

        std::shared_ptr<Map> map_;
        std::thread backend_thread_;
        std::mutex data_mutex_;

        // 条件变量
        std::condition_variable map_update_;

        // std::atomic 是模板类，一个模板类型为 T 的原子对象中封装了一个类型为 T 的值。
        // 原子类型对象不同线程同时访问不会产生数据竞争。
        // 原子类型变量用store写入，用load读取
        std::atomic<bool> backend_running_;

        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
    };
}

#endif // MYSLAM_BACKEND_H

/*
主要是因为我个人感觉后端的代码框架并不复杂，
仍然是一个两类节点（相机位姿和路标节点）+多条二元边的优化问题（前文第九讲 后端优化(1)中详细分析过），
并在此基础上增加了一步异常数据的筛选，代码复杂一点的部分都是在所构建的数据结构内读取相应信息的过程。
整个过程使用滑动窗口法控制BA规模，都是前文有详细讲过的内容。
*/