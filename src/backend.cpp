#include "myslam/algorithm.h"
#include "myslam/feature.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"

#include "myslam/backend.h"

// 一般编程中都需要先检查一个条件才进入等待环节，因此在中间有一个检查时段，检查条件的时候是不安全的，需要lock
namespace myslam
{

    Backend::Backend()
    {
        // 原子类型变量用store写入，用load读取
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this)); //类成员函数需要绑定该类的指针
        // bind函数的用法和详细参考： https://www.cnblogs.com/jialin0x7c9/p/12219239.html
        // this指针的用法和详解参考： http://c.biancheng.net/view/2226.html
    }

    void Backend::UpdateMap()
    {
        std::unique_lock<std::mutex> lock(data_mutex_); //没有defer_lock的话创建就会自动上锁了
        // std::unique_lock:  https://murphypei.github.io/blog/2019/04/cpp-concurrent-2.html
        // std::unique_lock:  https://cloud.tencent.com/developer/article/1583807
        // 唤醒一个正在等待的线程
        map_update_.notify_one(); //随机唤醒一个wait的线程
    }

    void Backend::Stop()
    {
        // backend_running标志为false，唤醒一个wait的线程，等待后端线程结束
        backend_running_.store(false); // replace the contained value with "parameter" 这里的parameter就是false
        // 后端结束时最后一次更新地图
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::BackendLoop()
    {
        // load读取backend_running的值
        // 实际上当后端在运行时，这是一个死循环函数，但是会等待前端的激活
        // 即前端激活一次，就运行此函数，进行一次后端优化
        while (backend_running_.load()) // load()   Read contained value
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            // wait():一般编程中都需要先检查一个条件才进入等待环节，因此在中间有一个检查时段，检查条件的时候是不安全的，需要lock
            // 被notify_one唤醒后，wait() 函数也会自动调用 data_mutex_.lock()，使得data_mutex_恢复到上锁状态
            map_update_.wait(lock);

            /// 后端仅优化激活的Frames和Landmarks
            Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
            Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
            Optimize(active_kfs, active_landmarks);
        }
    }

    // keyframes是map类中的哈希表，左边编号右边关键帧，lanmarks同理
    void Backend::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks)
    {
        // 设置g2o求解器
        //优化器构造可以参照： https://www.cnblogs.com/CV-life/p/10286037.html
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> block; //对于二元边来说，这里的6,3是两个顶点的维度
        //具体的先后顺序是库内写死的，第一个是pose 第二个是point
        // g2o::BlockSolver_6_3 可以整体代替g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>
        typedef g2o::LinearSolverCSparse<block::PoseMatrixType> LinearSolverType;
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<block>(g2o::make_unique<LinearSolverType>()));
        //创建稀疏优化器
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true); //打开调试输出

        // pose 顶点，使用Keyframe id
        std::map<unsigned long, VertexPose *> vertices;

        // g2o添加位姿顶点
        unsigned long max_kf_id = 0;
        for (auto &keyframe : keyframes)
        {
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());
            optimizer.addVertex(vertex_pose);

            if (kf->keyframe_id_ > max_kf_id)
            {
                max_kf_id = kf->keyframe_id_;
            }

            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        // 路标顶点，使用路标id索引
        std::map<unsigned long, VertexXYZ *> vertices_landmarks;

        // K 和左右外参
        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();

        // edges
        int index = 1;
        double chi2_th = 5.991; // robust kernel 阈值
        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

        for (auto &landmark : landmarks)
        {
            if (landmark.second->is_outlier_) // 路标点、特征点异常值跳过
                continue;
            unsigned long landmark_id = landmark.second->id_;

            // g2o添加路标点顶点
            auto observations = landmark.second->GetObs();
            for (auto &obs : observations)
            {
                if (obs.lock() == nullptr)
                    continue;
                auto feat = obs.lock();
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr)
                    continue;

                auto frame = feat->frame_.lock();
                EdgeProjection *edge = nullptr;

                if (feat->is_on_left_image_)
                {
                    edge = new EdgeProjection(K, left_ext);
                }
                else
                {
                    edge = new EdgeProjection(K, right_ext);
                }

                // 如果landmark还没有被加入优化，则新加一个顶点
                if (vertices_landmarks.find(landmark_id) == vertices_landmarks.end())
                {
                    VertexXYZ *v = new VertexXYZ();
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1); //这里就看出max_kf_id有啥用了
                    v->setMarginalized(true);              //是否边缘化,以便稀疏化求解
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }

                // ??连接顶点（相机位姿与路标点），测量值为关键点坐标，信息矩阵设为单位阵，huber核

                edge->setId(index);
                edge->setVertex(0, vertices.at(frame->keyframe_id_)); // pose
                // dynamic_cast<VertexPose *> ( optimizer.vertex ( frame->keyframe_id_) )
                edge->setVertex(1, vertices_landmarks.at(landmark_id)); // landmark
                // dynamic_cast<VertexXYZ *> ( optimizer.vertex ( landmark_id + max_kf_id + 1) )
                edge->setMeasurement(toVec2(feat->position_.pt));
                edge->setInformation(Mat22::Identity()); // e转置*信息矩阵*e,所以由此可以看出误差向量为n×1,则信息矩阵为n×n
                auto rk = new g2o::RobustKernelHuber();
                // g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                //设置鲁棒核函数，之所以要设置鲁棒核函数是为了平衡误差，不让二范数的误差增加的过快。
                // 鲁棒核函数里要自己设置delta值，
                // 这个delta值是，当误差的绝对值小于等于它的时候，误差函数不变。否则误差函数根据相应的鲁棒核函数发生变化。
                edge->setRobustKernel(rk);

                // 存入<边，特征点>关联容器map，添加边
                edges_and_features.insert({edge, feat});

                optimizer.addEdge(edge);

                index++;
            }
        }

        // 开始优化
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5)
        {
            cnt_outlier = 0;
            cnt_inlier = 0;

            // 记录正常点与异常点数量以及比例
            for (auto &ef : edges_and_features)
            {
                if (ef.first->chi2() > chi2_th)
                { //这里是误差大于阈值的意思吗？
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
                break;
            }
            else
            {
                chi2_th *= 2;
                iteration++;
            }
        }

        // 用最终的阈值门限，设置异常特征点并去掉路标点的错误观测
        for (auto &ef : edges_and_features)
        {
            if (ef.first->chi2() > chi2_th)
            {
                ef.second->is_outlier_ = true;
                // remove the observation
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

        // 修正相机位姿和路标点坐标
        // Set pose and lanrmark position
        for (auto &v : vertices)
        {
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        for (auto &v : vertices_landmarks)
        {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }
    }

}