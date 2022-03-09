#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H
#include "myslam/common_include.h"

#include "myslam/basicStruct/frame.h"

#include "myslam/camera.h"

///数据集操作类

namespace myslam
{

    class Dataset
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; //内存位数对齐，反正加上就对了吧

        typedef std::shared_ptr<Dataset> Ptr;

        Dataset(const std::string &dataset_path); //构造函数

        /// 初始化，返回是否成功
        bool Init(); //初始化函数，为什么这里需要初始化呢？

        /// create and return the next frame containing the stereo images
        Frame::Ptr NextFrame();

        Camera::Ptr GetCamera(int camera_id) const
        {
            /*
            The function automatically checks whether n is within the bounds of
            valid elements in the vector, throwing an out_of_range exception if it is not
            (i.e., if  n is greater than, or equal to, its size).
            This is in contrast with member operator[], that does not check against bounds.
            */
            return cameras_.at(camera_id);
        }

    private:
        std::string dataset_path_; // /home/cw/code/slam_learning/data_odometry_gray_ch13/05
        int current_image_index_ = 0;

        std::vector<Camera::Ptr> cameras_;
    };

}

#endif