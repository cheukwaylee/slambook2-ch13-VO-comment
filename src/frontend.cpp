#include "myslam/frontend.h"

#include <opencv2/opencv.hpp>

#include "myslam/basicStruct/feature.h"

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

namespace myslam
{

    Frontend::Frontend()
    {
        // 构造函数给OpenCV的feature detector赋值
        // 最大特征点数量 num_features
        // 角点可以接受的最小特征值 检测到的角点的质量等级，角点特征值小于qualityLevel*最大特征值的点将被舍弃 0.01
        // 角点最小距离 20
        gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20); // 150
        // TODO 另法
        // gftt_ = cv::FastFeatureDetector::create(Config::Get<int>("num_feature"));

        num_features_init_ = Config::Get<int>("num_features_init"); // 50
        num_features_ = Config::Get<int>("num_features");           // 50

        // TODO 在这里强调构造完之后前端处于initing状态？
        status_ = FrontendStatus::INITING;
    }

    //在增加某一帧时，根据目前的状况选择不同的处理函数（frontend的private方法）
    bool Frontend::AddFrame(myslam::Frame::Ptr frame)
    {
        // 更新当前帧
        current_frame_ = frame;
        switch (status_) // status_是前端Frontend的类成员，所以这里判断的是整个前端的状态
        {
        case FrontendStatus::INITING:
            StereoInit(); //这里的StereoInit应该是一个bool函数
            break;
            // TODO
            // if (StereoInit())
            //     break;
            // else
            //     return false;
        // 追踪正常时也需要track，因此没有break，顺序向下执行
        // 不加break就不会跳出switch结构，不管后面case的条件是否符合都将会执行，
        // 直到遇到第一个break才会跳出switch结构
        // 此处的TRACKING_GOOD和TRACKING_BAD并非跟踪失败或者跟踪成功的含义，这里的good和bad只是跟踪时条件好坏的区别
        // 当跟踪到的特征点数目充足时就是good，当特征点数目不足时就是bad，但不论good还是bad，都只是一个条件恶劣与否的问题，并不涉及失败
        // 当特征点少到不行的时候，就已经不是good和bad的问题了，太少的时候我们就认为跟踪丢了，设置为LOST，这个lost可以理解为跟踪失败
        // 所以，lost之后需要reset系统
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
        }
        last_frame_ = current_frame_;
        return true;
    }

    //根据上面的Addframe函数，我们应当在后续实现StereoInit，Track和Reset各个函数

    //先来StereoInit
    bool Frontend::StereoInit()
    {
        //提取左目特征（通常在左目提取特征时 特征点数量是一定能保证的）
        //一个frame其实就是一个时间点，里面同时含有左，右目的图像。
        int num_features_left = DetectFeatures();

        //根据左目特征在右目中找对应
        //虽然左目提取时特征点数量能够保证，但匹配过程则无法确保能够在右目图像中为所有的左目特征都找到对应
        //所以这一步最后找到的对应特征数目不一定满足初始化条件
        int num_coor_features = FindFeaturesInRight();

        //对应数目不足，无法初始化
        if (num_coor_features < num_features_init_) // yaml中设定：50
        {
            return false;
        }

        // 相当于else：左右目匹配数目足够多：前端初始化成功，则开始建立初始的地图
        bool build_map_success = BuildInitMap();
        if (build_map_success)
        {
            //初始地图建立成功
            // TODO 为什么要地图建立成功才改状态？不是应该左右目匹配足够就可以吗？
            status_ = FrontendStatus::TRACKING_GOOD; //前端状态由INITING转变为TRACKING_GOOD

            //可视化器是否开启
            if (viewer_)
            {
                //将当前帧加入可视化器
                viewer_->AddCurrentFrame(current_frame_);
                //更新可视化内容以显示当前帧,
                viewer_->UpdateMap();

                //为什么Track()函数里面只有AddCurrentFrame(current_frame_)，没有UpdateMap()呢？
                //还需后续读一读AddCurrentFrame(current_frame_)和UpdateMap()的具体源码实现。
            }
            return true;
        }

        //如果初始地图没有建立成功，则会跳过上述if段落，则还是没能完成初始化工作，返回false
        return false;
    }

    //假如初始化成功后，此时前端状态已经变为FrontendStatus::TRACKING_GOOD 再来一帧之后，则会执行Track()
    // 在执行Track之前，需要明白，Track究竟在做一件什么事情
    // Track是当前帧和上一帧之间进行的匹配，
    // 而初始化是某一帧左右目（双目）之间进行的匹配，这个要分清楚
    bool Frontend::Track()
    {
        //判断last_frame_是不是正常存在，如果是空指针，说明这个指针没有指到正确的上一帧所在的内存地址，则不执行后续语句，等同于false
        if (last_frame_)
        {
            // 当前帧pose等于上一帧pose加上一个帧间pose：刚体变换
            // world----current_frame = world----last_frame * last_frame----current_frame
            // 以【上一帧位姿】乘以【上上帧与上一帧之间的相对运动】作为这次位姿的估计初值
            // TODO 初始状态relative_motion_ 是SE3的默认值 可能是eye(4) ?
            current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
        }

        //光流法匹配上一帧与当前帧，返回光流法匹配到的点的数量
        // num_track_last可理解为从上一帧跟踪过来到当前帧 跟踪到的特征数目
        // TODO 没用到？
        // TrackLastFrame();
        int num_track_last = TrackLastFrame();

        //修正当前帧的位姿估计（根据重投影误差对当前帧pose的精化），
        // 返回追踪成功inliers点数量（在光流匹配成功基础上，更信任的点）
        // （作为前端状态 and 是否插入关键帧的判据）
        // 因为前面已经知道了一个current_frame_pose，所以可以根据这个pose初值判断inliers和outliers
        tracking_inliers_ = EstimateCurrentPose(); // 优化得到精确的current_frame_Pose

        //接下来根据跟踪到的内点的匹配数目，可以分类进行后续操作
        //(------lost--------)<num_features_tracking_bad_<(-------bad---------)<num_features_tracking_<(--------good--------)
        // 改变状态，为下次的添加帧做准备
        if (tracking_inliers_ > num_features_tracking_) // 50
        {
            // tracking good
            status_ = FrontendStatus::TRACKING_GOOD;
        }
        else if (tracking_inliers_ > num_features_tracking_bad_) // 20
        {
            // tracking bad
            status_ = FrontendStatus::TRACKING_BAD;
        }
        else
        {
            // lost
            status_ = FrontendStatus::LOST;
        }

        // InsertKeyframe还会判定追踪点数，当追踪点数足够多时，会自动放弃添加关键帧
        //根据当前的tracking_inliers_判断其是否为关键帧,
        // 当少于特定数目的点时，则可认为是一个关键帧，否则的话就不是，返回false
        //（但我个人理解这个判断是否为关键帧的阈值不能太少，否则就会lost）
        InsertKeyframe();

        // 计算当前帧与上一帧的相对运动（给下一个循环用）
        // TODO 为什么不能直接访问current_frame_的成员变量pose_呢？通过接口是为了线程锁？没有private为什么不能访问？
        // relative_motion_ = current_frame_->pose_ * last_frame_->Pose().inverse();
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();
        //看到这里可能会有些困惑，怎么又出来一个relative_motion呢？下面我作以详细介绍
        /*
         * 假设现在有3个连续帧A，B，C其中在对C进行Track()函数操作时，
         * 首先选用的relative_motion是A和B之间精准的relative_motion,
         * 为什么可以这么做呢，因为我们可以认为A，B，C这三帧之间的采样间隔是比较短的，
         * （或者说A-B之间的相对运动和B-C之间的相对运动之间隔的时间很短）
         * 基于这种小运动假设，我们可以认为A-B和B-C的相对运动其实差不多，
         * 可以拿A-B的相对运动作为B-C相对运动的一个初值来做inliers和outliers的判别，
         * 所以一开始求出的那个current_frame_pose其实是一个基于假设的粗略初值，
         * 只能拿来做一个outliers筛查，这也解释了为什么后面又求了一个relative_motion，
         * 这个时候的relative_motion由于前面EstimateCurrentPose()函数的存在，已经是精准的B-C相对运动了。
         */

        if (viewer_)
            viewer_->AddCurrentFrame(current_frame_); //可视化
        return true;
    }

    //针对前面的status_,已经将INITING，GOOD，BAD三种情况下对应的Stereoinit和Track函数都说明了，接下来说明Reset函数
    bool Frontend::Reset()
    {
        // 在需要插入日志的地方调用LOG(TYPE)<<”yourinfo”;即可。your info表示你要输入到日志文件中的信息。
        LOG(INFO) << "Reset is not implemented. ";

        //前端状态重置
        status_ = FrontendStatus::INITING;

        //高博在这里并没有做Reset的实现，我们可以在后续读完程序后给他补上
        return true;
    }

    //三个上层级函数已经实现，接下来对stereoInit，Track，Reset三个函数中的一些细节函数再作以补充实现。
    //首先对StereoInit函数中的DetectFeatures()，FindFeaturesInRight()，BuildInitMap()三个函数做实现，可视化模块放在后面统一实现

    //检测当前帧的左目的特征点，并放入feature的vector容器中
    int Frontend::DetectFeatures()
    {
        //掩膜，灰度图，同时可以看出，DetectFeatures是对左目图像的操作
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);

        // TODO current_frame_->features_left_不是空的吗？？？数据集读入的current_frame_不包含左目特征啊
        for (auto &feat : current_frame_->features_left_)
        {
            //在已有的特征附近一个矩形区域内将掩膜值设为0
            //即在这个矩形区域中不提取特征了，保持均匀性，并避免重复
            cv::rectangle(mask,
                          feat->position_.pt - cv::Point2f(10, 10),
                          feat->position_.pt + cv::Point2f(10, 10),
                          0, CV_FILLED);
        }
        // TODO 为什么这里要改原来current_frame_->features_left_里面的值？
        /* auto语法讲解：auto可以根据初始化值进行自动类型推断，在这里auto&定义了feat，
         * 初始化值就是后面的current_frame_->features_left_
         * auto&在自动类型推断完成定义的同时构成了引用类型，
         * 也就是说feat的改变将同步影响current_frame_->features_left_中的元素
         * 如果单单用一个auto就不会有这种同步效果
         */

        std::vector<cv::KeyPoint> keypoints; //关键点容器 检测之后的keypoints放在这里

        // detect函数，第三个参数是用来指定特征点选取区域的，一个和原图像同尺寸的掩膜，
        // 其中非0区域代表detect函数感兴趣的提取区域，相当于为detect函数明确了提取的大致位置
        // intput: src, output: keypoints, parameters: mask
        gftt_->detect(current_frame_->left_img_, keypoints, mask);

        int cnt_detected = 0; //检测到的特征计数
        for (auto &kp : keypoints)
        {
            //
            current_frame_->features_left_.push_back(
                Feature::Ptr(new Feature(current_frame_, kp)));
            /*这一句牵涉到两个知识点，第一个时new的用法，第二个是智能指针初始化赋值方法
             * new Feature(.....)这个整体应当是一个地址或者说指针含义，我们将这个地址赋值给一个Feature类型的智能指针，
             * 假如我们有一个指针int *p 那么p这个变量就是一个指针，
             * （单纯的使用p实则就是用它内部存储的地址，所以用p的时候可以说是用指针，也可以说是用地址）
             * Feature::Ptr(....)是一种初始化方法，就像个构造函数 只不过这里没有声明具体的存放变量（赋值给谁），
             * 而是构造出来后直接push_back到容器里面，所以可以理解为vector里面的一个单元就是具体的存放变量。
             */
            cnt_detected++;
        }

        LOG(INFO) << "Detect " << cnt_detected << " new features";
        return cnt_detected;
    }

    // TODO 两种实现对比一下
    /*
    int DetectFeatures(){
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);//掩膜，同时由这句可以看出，DetectFeatures是对左目图像的操作

        std::vector<cv::KeyPoint> keypoints; //关键点容器
        gftt_->detect(current_frame_->left_img_, keypoints);

        int cnt_detected = 0;//检测到的特征计数
        for (auto &kp : keypoints) {
            current_frame_->features_left_.push_back(
                    Feature::Ptr(new Feature(current_frame_, kp)));
            cnt_detected++;
        }

        for (auto &feat : current_frame_->features_left_){
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                          feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
        }//感觉这里就是为了画个矩形把点标注出来而已。

        LOG(INFO) << "Detect " << cnt_detected << " new features";
        return cnt_detected;
    }*/

    //找到左目图像的feature之后，就在右目里面找对应值(特征点)
    // use LK flow to estimate corresponding points in the right image
    int Frontend::FindFeaturesInRight()
    {
        //定义两个存储特征点pixel坐标的vector容器
        std::vector<cv::Point2f> kps_left, kps_right;

        // 遍历左目特征特征点：为了给光流跟踪提供一个右目初始值
        // TODO 这里没有需要改动current_frame_->features_left_的地方 为什么要auto& 为的节省内存吗？
        for (auto &kp : current_frame_->features_left_) // kp是 std::shared_ptr<Feature>& 当前帧的左目单个特征的引用
        {
            // feature类中的keypoint对应的point2f
            kps_left.push_back(kp->position_.pt); // 左目特征点位置

            // feature类中的mappoint
            //通过weak_ptr的lock()函数实现对地图点shared_ptr智能指针的复制，并赋予mp
            auto mp = kp->map_point_.lock(); // 左目特征点持有的地图路标

            //如果mp是一个非空的智能指针（左目特征点持有了地图路标）则执行if语段：
            // 右侧相机匹配点坐标的初始值为mappoint转换
            if (mp)
            {
                // use projected points as initial guess
                // camera_right_ 是frontend持有的Camera指针类对象（保存相机的参数信息）
                // 调用Camera的公有类方法： Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);
                // 根据左目特征点持有的地图路标的位置（wrt world） 和 当前帧的位置（相机位置）（wrt world）
                // 投影到右目pixel：推测这个左目特征点持有的地图路标在右目的pixel位置（作为初始值）
                auto px =
                    camera_right_->world2pixel(mp->pos_, current_frame_->Pose());

                // 把这个推测的初始值放入容器
                kps_right.push_back(cv::Point2f(px[0], px[1]));
            }
            //如果指针为空（左目特征点没持有地图路标）则执行else语段：
            // 初始值直接为左侧相机特征点坐标
            else
            {
                // use same pixel in left iamge
                kps_right.push_back(kp->position_.pt);
            }
        }

        //光流跟踪成功与否的状态向量（无符号字符），成功则为1,否则为0
        std::vector<uchar> status;
        Mat error;

        //进行光流跟踪，从这条opencv光流跟踪语句我们就可以知道：前面遍历左目特征关键点是为了给光流跟踪提供一个右目初始值
        // TODO 搞懂OpenCV自带的光流追踪（至少是用法）
        /* OpenCV光流法函数解析
         * calcOpticalFlowPyrLK(
         *              匹配图1，匹配图2，
         *              图1关键点，图2关键点存放容器（可以有初值），
         *              匹配情况，匹配误差，
         *              金字塔窗口大小，金字塔层数，
         *              终止条件（迭代次数+最小步长），
         *              使用图2关键点存放容器中的初值)
         */
        // OPTFLOW_USE_INITIAL_FLOW：
        //使用初始估计，存储在nextPts中；如果未设置标志，则将prevPts复制到nextPts并将其视为初始估计。
        cv::calcOpticalFlowPyrLK(
            current_frame_->left_img_, current_frame_->right_img_,
            kps_left, kps_right,
            status, error,
            cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        // 根据左目特征点在右目成功通过光流跟踪的点的数量
        int num_good_pts = 0;

        /* size_t的一些说明，（感觉这里用size_t意义不大）
         * 它是一种“整型”类型，里面保存的是一个整数，就像int, long那样。
         * 这种整数用来记录一个大小：size_t的全称应该是size type，就是说“一种用来记录大小的数据类型”。
         * 通常我们用sizeof(XXX)操作，这个操作所得到的结果就是size_t类型。
         * 因为size_t类型的数据其实是保存了一个整数，所以它也可以做加减乘除，也可以转化为int并赋值给int类型的变量。
         */
        // 遍历所有尝试过光流追踪的状态
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i]) // 追踪成功
            {
                // KeyPoint构造函数：7代表着关键点直径
                // 关键点代表的区域直径大小为7
                // TODO 搞清楚
                cv::KeyPoint kp(kps_right[i], 7);

                // 有一个特征点，就要有一个特征类
                Feature::Ptr feat(new Feature(current_frame_, kp));

                //指明是右侧相机feature
                feat->is_on_left_image_ = false;

                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            }
            else //左右目匹配失败
            {
                //光流跟踪没找到对应的特征，就在features_right_里面填空指针
                current_frame_->features_right_.push_back(nullptr);
            }
        }

        LOG(INFO) << "Find " << num_good_pts << " in the right image.";
        return num_good_pts;
    }

    //现在左目图像的特征提取出来了，并根据左目图像的特征对右目图像做了特征的光流跟踪，找到了对应值，
    //当对应数目满足阈值条件时，我们可以开始建立初始地图
    bool Frontend::BuildInitMap()
    {
        //构造一个存储SE3的vector，初始化放两个pose：一个左目pose，一个右目pose，
        //看到这里应该记得，Frame也有一个pose：Frame里面的pose描述了某一帧间的位姿变化，wrt固定坐标系（世界坐标系）
        //这个固定坐标系可能是第一帧的左目，可能是右目，也可能是左右目中间。
        // TODO 初始化之后左右目相机的pose是初始化后SE3的状态 应该是eye(4)吧？用这个来三角化吗？ANS：第一帧的位置就是世界坐标的原点
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};

        size_t cnt_init_landmarks = 0; //初始化的路标数目

        //遍历左目的feature
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            //对于左目的每一个feature，我们必须在右目找到对应的feature，才能继续三角化，不然就跳过
            //对于右目feature容器，如果是成功跟踪到了就是一个指向Feature的指针，否则就是个空指针，
            //我们需要跳过跟踪失败的空指针
            if (current_frame_->features_right_[i] == nullptr)
                //右目没有与左目对应的特征点
                continue;

            // 对于左右目配对成功的点，三角化它
            // create map point from triangulation

            //左目右目的像素坐标都转换到相机（归一化）坐标，存到points这个vector里面，
            //下一次循环中将重置一个含有两个元素的新vector
            std::vector<Vec3> points{
                //将配对点中的左目像素坐标转换到相机（归一化）坐标 返回Vec3
                camera_left_->pixel2camera(
                    Vec2(
                        current_frame_->features_left_[i]->position_.pt.x,
                        current_frame_->features_left_[i]->position_.pt.y)),
                //将配对点中的右目像素坐标转换到相机（归一化）坐标 返回Vec3
                camera_right_->pixel2camera(
                    Vec2(
                        current_frame_->features_right_[i]->position_.pt.x,
                        current_frame_->features_right_[i]->position_.pt.y))
                // Vec3 pixel2camera(const Vec2 &p_p, double depth = 1); 有默认参数
            };

            //建立一个存储3D世界坐标的VEC3，3D向量 x,y,z
            Vec3 pworld = Vec3::Zero();

            //正式三角化
            // triangulation()函数：相机位姿 某个feature左右目的坐标 三角化后的坐标保存
            // bool triangulation(const std::vector<SE3> &poses, const std::vector<Vec3> points, Vec3 &pt_world)
            // TODO pworld没有传入地址，为什么他的值可以被改变？在triangulation()的定义里面为什么有时候要引用，有时候不用？
            // TODO pworld[2]是刚刚初始化的吧？不就是0,0,0吗应该？ANS：if括号里面的按顺序执行
            //根据前面存放的左右目相机pose 和 对应点相机坐标points 进行三角化，得到对应路标点的深度，构造出路标点pworld
            //需要对pworld进行判断，看其深度是否大于0, pworld[2]即是其深度。
            if (triangulation(poses, points, pworld) && pworld[2] > 0)
            {
                //创建一个MapPoint类对象用来承载三角化出的世界坐标pworld
                //工厂模式创建一个新的地图点 静态函数不需要对象就可以直接调用
                auto new_map_point = MapPoint::CreateNewMappoint(); // MapPoint::Ptr

                // mappoint类主要的数据成员：路标点世界坐标（pos_） 和 观测到的feature的vector（observations_）
                new_map_point->SetPos(pworld);

                //为路标点添加特征点：landmark持有feature
                //这个路标点对应到了当前帧（应有帧ID）的 左目图像特征中的第i个 以及 右目图像特征中的第i个
                new_map_point->AddObservation(current_frame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_right_[i]);

                //为特征类Feature的对象添加路标点成员：feature持有landmark
                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;

                //初始化成功的landmark或者说路标点数目加1
                cnt_init_landmarks++;

                //对Map类对象来说，地图里面应当多了一个路标点，所以要将这个路标点加到地图中去
                map_->InsertMapPoint(new_map_point);
            }
        }

        //当前帧能够进入初始化（能进入当前函数）说明已经满足了初始化所需的帧特征数量
        //作为初始化帧，可看做开始的第一帧，所以应当是一个关键帧
        current_frame_->SetKeyFrame();

        //对Map类对象来说，地图里面应当多了一个关键帧，所以要将这个关键帧加到地图中去
        map_->InsertKeyFrame(current_frame_);

        //关键帧插入，后端需要对新纳入的关键帧进行优化处理
        // TODO 日后再看
        backend_->UpdateMap();

        //向日志输入消息
        LOG(INFO) << "Initial map created with " << cnt_init_landmarks
                  << " map points";

        return true;
    }

    //接下来对Track函数中的TrackLastFrame()，EstimateCurrentPose()，InsertKeyframe()三个函数做实现，可视化模块放在后面统一实现

    //该函数的实现非常像，FindFeaturesInRight()：同一帧左右目之间找
    // TrackLastFrame()：从上一帧的左目跟踪到当前帧的左目
    // use LK flow to estimate points in the current image wrt last image
    int Frontend::TrackLastFrame()
    {
        //定义两个存储上一/当前帧左目特征点坐标的vector容器
        std::vector<cv::Point2f> kps_last, kps_current;

        //遍历上一帧中左目特征
        for (auto &kp : last_frame_->features_left_) // kp是 std::shared_ptr<Feature>& 上一帧的左目单个特征的引用
        {
            // feature类中的keypoint对应的point2f
            kps_last.push_back(kp->position_.pt); // 上一帧左目特征点位置

            // feature类中的mappoint
            //通过weak_ptr的lock()函数实现对地图点shared_ptr智能指针的复制，并赋予mp
            auto mp = kp->map_point_.lock(); // 上一帧的左目特征点持有的地图路标

            /* TODO
             * 这里需要注意，对于左目图像来说，我们可以将其用于估计相机pose（能定位），
             * 但是不一定左目图像中的每一个点都有mappoint：
             * mappoint的形成是需要左目和同一帧的右目中构成对应关系才可以（能够三角化出世界坐标，能建图），
             * 有些左目中的feature在右目中没有配对，就没有Mappoint，但是没有Mappoint却不代表这个点是一个outlier
             */

            // 如果这个特征点有对应的路标点，则当前帧的光流法初值为：路标点对当前帧位姿（初值）的投影：
            //判断该特征有没有构建出相应的地图点：
            if (mp)
            {
                //对于建立了Mappoint的特征点而言：use projected points as initial guess
                // kps_current的初值是通过world2pixel转换得到的：
                // 投影到当前帧pixel：推测上一帧特征点持有的地图路标在当前帧的pixel位置（作为初始值）
                auto px =
                    camera_left_->world2pixel(mp->pos_, current_frame_->Pose());

                // kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(px[0], px[1]));
            }
            // 否则当前帧的光流法初值就是上一帧的特征点坐标
            else
            {
                //没有MapPoint就没有初始化猜测值，那么光流搜索的起点就是上一帧点的像素位置
                kps_current.push_back(kp->position_.pt);
            }
        }

        //光流跟踪成功与否的状态向量（无符号字符），成功则为1,否则为0
        std::vector<uchar> status;
        Mat error;

        /* OpenCV光流法函数解析
         * calcOpticalFlowPyrLK(
         *              匹配图1，匹配图2，
         *              图1关键点，图2关键点存放容器（可以有初值），
         *              匹配情况，匹配误差，
         *              金字塔窗口大小，金字塔层数，
         *              终止条件（迭代次数+最小步长），
         *              使用图2关键点存放容器中的初值)
         */
        // OPTFLOW_USE_INITIAL_FLOW：
        //使用初始估计，存储在nextPts中；如果未设置标志，则将prevPts复制到nextPts并将其视为初始估计。
        cv::calcOpticalFlowPyrLK(
            last_frame_->left_img_, current_frame_->left_img_,
            kps_last, kps_current,
            status, error,
            cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        // 根据上一帧特征点在当前帧成功通过光流跟踪的点的数量
        int num_good_pts = 0;

        for (size_t i = 0; i < status.size(); ++i)
        {
            // status[i]=true则说明跟踪成功有对应点，false则跟踪失败没找到对应点
            if (status[i]) // 匹配成功
            {
                // KeyPoint构造函数：7代表着关键点直径
                // 关键点代表的区域直径大小为7
                // TODO 搞清楚
                cv::KeyPoint kp(kps_current[i], 7);

                // 有一个特征点，就要有一个特征类
                Feature::Ptr feature(new Feature(current_frame_, kp));

                // 特征点对应的地图点：就是上一帧的点对应的地图点
                // 世界坐标上的同一个点在相邻的帧被观测到（被光流追踪到）
                feature->map_point_ = last_frame_->features_left_[i]->map_point_;
                /* TODO
                 * 这个时候这里面的map_point_有可能之前没在右目中找到对应点，
                 * 所以其对应的map_point_未曾被初始化或者赋值，也可以这么写吗？
                 * 针对于这种情况，应该只是保证了feature->map_point_和last_frame_->features_left_[i]->map_point_之间的相同性，
                 * 但却并没有关注last_frame_->features_left_[i]->map_point_是否被初始化或者赋值。
                 */

                // 填充当前帧的左图特征点容器
                current_frame_->features_left_.push_back(feature);

                // 匹配成功点计数
                num_good_pts++;
            }
        }

        // 匹配成功的点数量记录到日志，并返回
        LOG(INFO) << "Find " << num_good_pts << " in the last image.";
        return num_good_pts;
    }

    //跟踪成功之后可以求解精准的Current——Pose
    // EstimateCurrentPose()求解精准的CurrentPose
    int Frontend::EstimateCurrentPose()
    {
        // setup g2o
        //利用g2o来进行优化求解，先进行优化求解器配置
        //双边：一个顶点为6维T，一个为3维point
        // p,l分别表示pose和landmark点维度：其中p指的是流行空间的最小维度6,分别代表平移和旋转，不管旋转用不用四元数
        typedef g2o::BlockSolver_6_3 BlockSolverType;

        // TODO 后端用的是g2o::LinearSolverCSparse
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
            LinearSolverType;

        //定义总solver求解器，选择优化方法
        auto solver =
            new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

        //建立稀疏矩阵优化器
        g2o::SparseOptimizer optimizer;

        //使用定义的求解器求解
        optimizer.setAlgorithm(solver);

        /* 明确一下这个过程中我们要优化的变量是什么，
         * 这里我们认为地图路标点的3D位置是不用再去优化的（想优化也可以）
         * 所以这个过程中只需要优化R，t：//TODO 要优化的是当前帧的位姿current_frame_->Pose() wrt world
         * 而R，t可以统一为一个变量SE3，所以这个两帧间的SE3就是我们的优化对象，也就是g2o中的一个顶点（一个变量）
         */

        // 设定顶点 vertex：优化变量 decision variable
        VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
        vertex_pose->setId(0);

        //设定初值：根据 当前帧pose等于上一帧pose加上一个帧间pose 估计
        vertex_pose->setEstimate(current_frame_->Pose());

        //添加顶点
        optimizer.addVertex(vertex_pose);

        // K：左目内参矩阵
        Mat33 K = camera_left_->K();

        //  设定边 edges：误差项 cost function
        int index = 1;                               //建立索引
        std::vector<EdgeProjectionPoseOnly *> edges; //建立边的容器，边类型为EdgeProjectionPoseOnly*
        std::vector<Feature::Ptr> features;          //建立一个特征容器

        // 优化变量decision variable：// TODO 两帧间的SE3就是我们的优化对象 ？？？
        //                     当前帧的位姿current_frame_->Pose() wrt world ？？？竹曼认为是这个
        // 代价函数cost function：重投影误差 ？？？

        // 建立并添加边
        // 遍历当前帧的左目特征：  因为一帧有多个投影点（多个特征点在帧间重投影）构成多个方程，
        //              所以有多个边：一对特征就有一个重投影误差项，就是一条边（误差）
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            //这里就涉及到前面在TrackLastFrame()函数里面提到的：有些特征虽然被跟踪到了，
            //但是并没有受到三角化，即没有map_point_，这里便对feature有没有map_point_进行判断，
            //有则可以往下进行重投影，没有则不行，因为重投影需要点的3D位置
            auto mp = current_frame_->features_left_[i]->map_point_.lock();
            // 遍历到的当前帧的左目特征持有路标点（被三角化过）
            if (mp)
            {
                EdgeProjectionPoseOnly *edge =
                    new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);

                // 顶点：待优化的变量
                // 一个顶点（相机位置）对应多条边（各个特征点的重投影误差）
                edge->setVertex(0, vertex_pose);

                // 特征点的位置 x,y
                edge->setMeasurement(
                    toVec2(current_frame_->features_left_[i]->position_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());

                //鲁棒核函数
                edge->setRobustKernel(new g2o::RobustKernelHuber);

                // features 和 edges 的索引号都一样的，都对应第i号左目当前帧的特征点
                features.push_back(current_frame_->features_left_[i]);
                edges.push_back(edge);

                optimizer.addEdge(edge);
                index++;
            }
        }

        // estimate the Pose the determine the outliers
        const double chi2_th = 5.991;
        int cnt_outlier = 0;

        // 总共优化40遍
        // 以10遍为一个优化周期：对outlier进行一次判断，并舍弃掉outlier的边，随后再进行下一个10步优化
        for (int iteration = 0; iteration < 4; ++iteration)
        {
            //每次优化的初值（同一帧的相机位置一致）都设定为current_frame_->Pose()，
            // 但每次涉及的特征都不一样，所以每次的重投影误差都不一样（同一帧有多个特征点）
            // 就有可能发现新的outlier，这是一个不断筛查,删除,精化的过程
            vertex_pose->setEstimate(current_frame_->Pose());

            optimizer.initializeOptimization();
            optimizer.optimize(10); // 每次循环迭代10次
            cnt_outlier = 0;

            // count the outliers
            //遍历优化后的边（当前帧左目各个特征点的重投影误差）
            for (size_t i = 0; i < edges.size(); ++i)
            {
                auto e = edges[i]; // 取出各边

                // 特征点本身就是异常点，计算重投影误差
                if (features[i]->is_outlier_)
                {
                    e->computeError();
                }

                // TODO
                // 因为前面已经知道了一个current_frame_pose，所以可以根据这个pose初值判断inliers和outliers
                // 这个是前面的解释，与这里的信息矩阵有关系吗？？
                //（信息矩阵对应的范数）误差超过阈值，判定为异常点，并计数
                if (e->chi2() > chi2_th)
                {
                    features[i]->is_outlier_ = true;
                    /* 设置等级：（这里每个边都有一个level的概念）
                     * 一般情况下g2o只处理level=0的边，
                     * 在orbslam中，如果确定某个边的重投影误差过大，则把level设置为1，
                     * 也就是舍弃这个边对于整个优化的影响（下次循环g2o不再优化异常值）
                     */
                    e->setLevel(1);
                    cnt_outlier++;
                }
                // 否则恢复为正常点
                else
                {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);
                }

                //后20次不设置鲁棒核函数了，意味着此时不太可能出现大的异常点
                if (iteration == 2)
                {
                    e->setRobustKernel(nullptr);
                }
            }
        }
        // 异常点/正常点记录到日志
        LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
                  << features.size() - cnt_outlier;

        // 这个函数的主要目的！！
        // 保存优化后的位姿 // Set pose and outlier
        current_frame_->SetPose(vertex_pose->estimate());

        LOG(INFO) << "Current Pose = \n"
                  << current_frame_->Pose().matrix();

        // 对于被认为是异常值的特征，重置特征与路标点的对应关系（而不是重置路标点）
        // 并把它重新记作正常值，认为它只是对应关系错了，并不是所谓的噪点，可能未来有用
        for (auto &feat : features)
        {
            if (feat->is_outlier_)
            {
                // 弱指针自带的操作函数reset，作用是将指针置空
                // 清空这个外点特征点持有的路标点
                feat->map_point_.reset();

                feat->is_outlier_ = false; // maybe we can still use it in future
            }
        }
        return features.size() - cnt_outlier; // inliers
    }

    // 在完成前后帧的跟踪和pose估计后，我们需要对新来的每一帧进行关键帧判别，
    // 看它是不是一个关键帧，这里就需要用到InsertKeyframe函数
    // 在InsertKeyFrame函数中出现了一个三角化步骤，这是因为当一个新的关键帧到来后，
    // 我们势必需要补充一系列新的特征点，此时则需要像建立初始地图一样，
    // 对这些新加入的特征点进行三角化，求其3D位置
    bool Frontend::InsertKeyframe()
    {
        // still have enough features, don't insert keyframe
        if (tracking_inliers_ >= num_features_needed_for_keyframe_)
        {
            return false;
        }

        // 当跟踪到的特征数目小于阈值时：
        // 认为运动已有足够大的空间，时间幅度，可视做一个新的关键帧
        // current frame is a new keyframe
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);

        LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
                  << current_frame_->keyframe_id_;

        //添加关键帧的路标点
        SetObservationsForKeyFrame();

        //检测当前关键帧的左目特征点
        //如果是关键帧才能执行到这一步（是关键帧的话其跟踪到的内点数目就会相应不足，需要补充）
        DetectFeatures(); // detect new features

        //在右目追踪左目的匹配点 // track in right image
        FindFeaturesInRight();

        // triangulate map points
        TriangulateNewPoints();

        // update backend because we have a new keyframe
        backend_->UpdateMap();

        if (viewer_)
            viewer_->UpdateMap();

        return true;
    }

    //这个函数其实与BuildInitMap差不多
    int Frontend::TriangulateNewPoints()
    {
        // 利用花括号初始化位姿数组，内容是左目图位姿和右目图位姿，左目图位姿就是当前帧位姿
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};

        // 当前帧位姿的逆矩阵：current_frame_->Pose()是从世界到相机,逆就是从相机到世界
        SE3 current_pose_Twc = current_frame_->Pose().inverse();

        // 三角化的点计数
        int cnt_triangulated_pts = 0; //三角化成功的点的数目

        // 对于每个当前帧的特征点
        // 遍历新的关键帧（左目）内的所有特征点，和右目对应点做三角化
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            // expired()用于检查智能指针指向的对象是否为空，expired()为true等价于use_count() == 0
            // 所以true表示关联的路标点不存在，false表示关联的路标点存在。
            // 即：左目特征点没有关联的路标点 && 右目有对应的特征点 -> 则三角化
            if (current_frame_->features_left_[i]->map_point_.expired() &&
                current_frame_->features_right_[i] != nullptr)
            {
                // points数组保存双目图所有特征点的归一化平面坐标
                // 将匹配的像素点从像素坐标转化到相机坐标下
                std::vector<Vec3> points{
                    camera_left_->pixel2camera(
                        Vec2(
                            current_frame_->features_left_[i]->position_.pt.x,
                            current_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(
                        Vec2(
                            current_frame_->features_right_[i]->position_.pt.x,
                            current_frame_->features_right_[i]->position_.pt.y))
                    // Vec3 pixel2camera(const Vec2 &p_p, double depth = 1); 有默认参数
                };

                //相机坐标系下点的3D位置坐标 x,y,z
                Vec3 pworld = Vec3::Zero();

                //正式三角化
                // triangulation()函数：相机位姿 某个feature左右目的坐标 三角化后的坐标保存
                // bool triangulation(const std::vector<SE3> &poses, const std::vector<Vec3> points, Vec3 &pt_world)
                //如果三角化流程运行成功且得到的深度值pworld[2]大于0有意义
                if (triangulation(poses, points, pworld) && pworld[2] > 0)
                {
                    //创建并设立路标点
                    auto new_map_point = MapPoint::CreateNewMappoint(); // MapPoint::Ptr

                    //注意这里与初始化地图不同 triangulation计算出来的点pworld，
                    //实际上是相机坐标系下的点，所以需要乘以一个TWC
                    //但是初始化地图时，是第一帧：一般以第一帧为世界坐标系
                    // 普通三角化得到的是左图的相机坐标系下的坐标，左乘Twc得到世界坐标系坐标
                    pworld = current_pose_Twc * pworld; //从相机坐标系转到世界坐标系

                    //设置mapoint类中的坐标
                    new_map_point->SetPos(pworld);

                    // 设置新路标点坐标，拥有的特征点：建立当前帧所有特征点与路标点的对应关系
                    // 为路标点类添加特征成员变量 //增加mappoint类中的对应的那个feature（左右目）
                    new_map_point->AddObservation(current_frame_->features_left_[i]);
                    new_map_point->AddObservation(current_frame_->features_right_[i]);

                    //为特征类Feature的对象添加路标点成员：feature持有landmark
                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    current_frame_->features_right_[i]->map_point_ = new_map_point;

                    // 将路标点加入到地图中，
                    //既然有了一个新的路标点mappoint，那就应当更新一下地图（类），向地图类的对象中添加路标点。
                    map_->InsertMapPoint(new_map_point);

                    // 三角化点计数
                    cnt_triangulated_pts++;
                }
            }
        }
        // 记录并返回新三角化的点的数目
        LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
        return cnt_triangulated_pts; //新添加了多少个路标点（landmarks）
    }

    void Frontend::SetObservationsForKeyFrame()
    {
        for (auto &feat : current_frame_->features_left_)
        {
            //查找当前帧中的特征，看是否对应已有的路标点
            auto mp = feat->map_point_.lock();

            //若对应则为路标点添加当前帧内的特征观测
            if (mp)
                mp->AddObservation(feat);
            //若不对应则不做操作，跳过即可
        }
    }
}