#include "myslam/backend.h"

#include "myslam/basicStruct/feature.h"
#include "myslam/basicStruct/mappoint.h"

#include "myslam/g2o_types.h"
#include "myslam/algorithm.h"
#include "myslam/map.h"

// 一般编程中都需要先检查一个条件才进入等待环节，因此在中间有一个检查时段，检查条件的时候是不安全的，需要lock
namespace myslam
{
    // 后端的构造函数：指定后端线程的回调函数，然后挂起（wait）
    Backend::Backend()
    {
        // 原子类型变量用store写入，用load读取
        backend_running_.store(true);

        // 创建一个线程，线程执行的函数是BackendLoop
        // 类成员函数需要绑定该类的指针：相当于 Backend::BackendLoop(this)
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
        /* 在c++中，如果回调函数是一个类的成员函数，这时想把成员函数设置给一个回调函数指针往往是不行的
         * 因为类的成员函数，多了一个隐含的参数this，
         * 所以直接赋值给函数指针肯定会引起编译报错，绑了之后能被外面用
         * bind函数的用法和详细参考： https://www.cnblogs.com/jialin0x7c9/p/12219239.html
         * this指针的用法和详解参考： http://c.biancheng.net/view/2226.html
         */
    }

    void Backend::UpdateMap()
    {
        // 没有defer_lock的话创建就会自动上锁了
        std::unique_lock<std::mutex> lock(data_mutex_);
        // std::unique_lock:  https://murphypei.github.io/blog/2019/04/cpp-concurrent-2.html
        // std::unique_lock:  https://cloud.tencent.com/developer/article/1583807

        // 唤醒一个正在等待的线程
        map_update_.notify_one(); //随机唤醒一个wait的线程
    }

    void Backend::Stop()
    {
        // backend_running标志为false，唤醒一个wait的线程，等待后端线程结束
        // replace the contained value with "parameter" 这里的parameter就是false
        backend_running_.store(false);

        // 后端结束时最后一次更新地图
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::BackendLoop()
    {
        // 原子类型变量用store写入，用load读取
        // 实际上当后端在运行时，这是一个死循环函数 while(1)，但是会等待前端的激活
        // 即：前端激活一次，就运行此函数，进行一次后端优化
        while (backend_running_.load()) // Read contained value
        {
            std::unique_lock<std::mutex> lock(data_mutex_);

            // wait()：一般编程中都需要先检查一个条件才进入等待环节，
            // 因此在中间有一个检查时段，检查条件的时候是不安全的，需要lock
            // 被notify_one唤醒后，wait()函数也会自动调用data_mutex_.lock()，使得data_mutex_恢复到上锁状态
            map_update_.wait(lock);

            /// 后端仅优化激活的Frames和Landmarks
            Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();       // R6
            Map::LandmarksType active_landmarks = map_->GetActiveMapPoints(); // R3
            Optimize(active_kfs, active_landmarks);
        }
    }

    // keyframes是map类中的哈希表，左边编号id，右边关键帧Frame::Ptr
    // lanmarks同理，           id   路标点MapPoint::Ptr
    void Backend::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks)
    {
        // 设置g2o求解器
        // 优化器构造可以参照： https://www.cnblogs.com/CV-life/p/10286037.html
        // 对于二元边来说，这里的6,3是两个顶点的维度
        // 具体的先后顺序是库内写死的，第一个是关键帧相机位置pose 第二个是路标点point
        // g2o::BlockSolver_6_3 可以整体代替g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> block; // 等同于前端的BlockSolverType

        // TODO 前端用的是g2o::LinearSolverDense 是数据规模的问题吗？
        typedef g2o::LinearSolverCSparse<block::PoseMatrixType> LinearSolverType;

        // TODO 这里可以用auto吧？有new的话类型很容易看出来（前端用了auto）
        // g2o::OptimizationAlgorithmLevenberg *solver =
        //     new g2o::OptimizationAlgorithmLevenberg(
        //         g2o::make_unique<block>(g2o::make_unique<LinearSolverType>()));
        auto solver =
            new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<block>(g2o::make_unique<LinearSolverType>()));

        //创建稀疏优化器
        g2o::SparseOptimizer optimizer;

        //使用定义的求解器求解
        optimizer.setAlgorithm(solver);

        //打开调试输出
        optimizer.setVerbose(true);

        //
        //待优化的变量：关键帧的位姿SE3（R6） + 路标点位置x,y,z
        //

        // 相机pose顶点（待优化的变量）的容器：id，关键帧的位姿R6
        std::map<unsigned long, VertexPose *> vertices;

        // g2o添加位姿顶点
        unsigned long max_kf_id = 0; // 最大的关键帧id
        for (auto &keyframe : keyframes)
        {
            auto kf = keyframe.second; // Frame::Ptr

            VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose()); // 设定优化初值
            optimizer.addVertex(vertex_pose);

            if (kf->keyframe_id_ > max_kf_id)
            {
                max_kf_id = kf->keyframe_id_;
            }

            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        // 路标点x,y,z顶点（待优化的变量）的容器：id，路标点的位置R3
        std::map<unsigned long, VertexXYZ *> vertices_landmarks;

        // K内参 和左右外参
        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();

        // edges
        int index = 1;
        double chi2_th = 5.991; // robust kernel 阈值
        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

        // g2o添加路标点作为顶点
        for (auto &landmark : landmarks)
        {
            // landmark.second的类型是MapPoint::Ptr

            // 路标点、特征点异常值跳过
            if (landmark.second->is_outlier_)
                continue;

            unsigned long landmark_id = landmark.second->id_;

            // 路标点关联的feature的vector std::list<std::weak_ptr<Feature>>
            auto observations = landmark.second->GetObs();

            // 内层循环：遍历当前路标点关联到的特征点
            for (auto &obs : observations)
            {
                // obs的类型是 std::weak_ptr<Feature>

                if (obs.lock() == nullptr) // 读weak_ptr的方法
                    continue;

                auto feat = obs.lock(); // 类型：Feature

                // 特征点是外点 or
                // 持有该feature的frame是空的 std::weak_ptr<Frame> frame_;
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr)
                    continue;

                auto frame = feat->frame_.lock(); // 类型：Frame

                // 创建边
                EdgeProjection *edge = nullptr;
                // 根据观测到特征点的左右目情况
                if (feat->is_on_left_image_)
                {
                    edge = new EdgeProjection(K, left_ext);
                }
                else
                {
                    edge = new EdgeProjection(K, right_ext);
                }

                // 如果landmark还没有被加入优化，则新加一个顶点
                // 是有可能在上一循环已经被加入了，因为内层循环遍历的是特征点，特征点有好有坏，外点被跳过没有添加
                if (vertices_landmarks.find(landmark_id) == vertices_landmarks.end())
                {
                    // TODO 为什么要括号？
                    // VertexXYZ *v = new VertexXYZ();
                    VertexXYZ *v = new VertexXYZ;

                    v->setEstimate(landmark.second->Pos());

                    v->setId(landmark_id + max_kf_id + 1); //这里就看出max_kf_id有啥用了

                    // TODO ？？是否边缘化,以便稀疏化求解
                    v->setMarginalized(true);

                    vertices_landmarks.insert({landmark_id, v});

                    optimizer.addVertex(v);
                }

                // TODO ？？连接顶点（相机位姿与路标点），
                // 测量值为关键点坐标，信息矩阵设为单位阵，huber核

                edge->setId(index);

                // 相机pose
                // dynamic_cast<VertexPose *> ( optimizer.vertex ( frame->keyframe_id_) )
                edge->setVertex(0, vertices.at(frame->keyframe_id_));

                // 路标点landmark
                // dynamic_cast<VertexXYZ *> ( optimizer.vertex ( landmark_id + max_kf_id + 1) )
                edge->setVertex(1, vertices_landmarks.at(landmark_id));

                edge->setMeasurement(toVec2(feat->position_.pt));

                // e转置*信息矩阵*e,所以由此可以看出误差向量为n×1,则信息矩阵为n×n
                edge->setInformation(Mat22::Identity());

                //设置鲁棒核函数，之所以要设置鲁棒核函数是为了平衡误差，不让二范数的误差增加的过快。
                // g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber();
                auto rk = new g2o::RobustKernelHuber();

                // 鲁棒核函数里要自己设置delta值，
                // 这个delta值是，当误差的绝对值小于等于它的时候，误差函数不变。否则误差函数根据相应的鲁棒核函数发生变化。
                rk->setDelta(chi2_th);

                edge->setRobustKernel(rk);

                // 存入<边，特征点>关联容器map，添加边
                edges_and_features.insert({edge, feat});

                optimizer.addEdge(edge);

                index++;
            }
        }

        // 开始优化
        optimizer.initializeOptimization();
        optimizer.optimize(10); // 每次循环迭代10次

        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5)
        {
            cnt_outlier = 0;
            cnt_inlier = 0;

            // 记录正常点与异常点数量以及比例
            // 遍历std::map<EdgeProjection *, Feature::Ptr>
            for (auto &ef : edges_and_features)
            {
                // TODO 这里是误差大于阈值的意思吗？
                // ef.first 的类型是 EdgeProjection *
                if (ef.first->chi2() > chi2_th)
                {
                    cnt_outlier++;
                }
                else
                {
                    cnt_inlier++;
                }
            }

            // 不断调整阈值，迭代寻找一个适合淘汰异常点的代价函数阈值
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);

            if (inlier_ratio > 0.5)
            {
                // 内点率过半 停止优化
                break;
            }
            else
            {
                // 否则（内点太少了） 鲁棒核函数的delta值变大
                chi2_th *= 2;
                iteration++;
            }
        }

        // 用最终的阈值门限，设置异常特征点 并去掉路标点的错误观测
        for (auto &ef : edges_and_features)
        {
            if (ef.first->chi2() > chi2_th)
            {
                // ef.second的类型是Feature::Ptr
                ef.second->is_outlier_ = true;

                // remove the observation
                // 在外点特征点对应的路标点 移除关联的当前特征：
                // 一个特征点关联了路标点
                // 当这个特征点被判定为外点的时候
                // 就要在 这个特征点关联的路标点 上移除对这个特征点的观测
                ef.second->map_point_.lock()->RemoveObservation(ef.second);
            }
            else
            {
                ef.second->is_outlier_ = false;
            }
        }

        // 记录最终的异常值比重
        LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
                  << cnt_inlier;

        // Set pose and lanrmark position
        // 优化结果输出 写入！！
        // 修正相机位姿和路标点坐标
        // std::map<unsigned long, VertexPose *> vertices;
        for (auto &v : vertices)
        {
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        // std::map<unsigned long, VertexXYZ *> vertices_landmarks;
        for (auto &v : vertices_landmarks)
        {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }
    }

}