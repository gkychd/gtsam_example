/**
 * @file GPSPose2Factor.h
 * @author gongkeyang
 * @brief 2D 'GPS' like factor for Pose2
 * @version 0.1
 * @date 2022-08-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/**
 * @brief 简单的类2D GPS 因子
 * 该因子包含X、Y的测量值（Pose2），包含旋转信息
 * The error vector will be [x-mx, y-my]'
 */

#pragma once 

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>

namespace gtsamexamples{

class GPSPose2Factor: public gtsam::NoiseModelFactor1<gtsam::Pose2>{

private:

    double mx_, my_;

public:

    /**
     * @brief 构造函数
     * @param poseKey             待优化的位姿节点
     * @param model                 噪声模型
     * @param m                          point2的测量值
     * 
     */

    GPSPose2Factor(gtsam::Key poseKey, const gtsam::Point2 m, gtsam::SharedNoiseModel model) : 
        gtsam::NoiseModelFactor1<gtsam::Pose2>(model, poseKey), mx_(m.x()), my_(m.y()){}

    /**
     * @brief error function 误差方程
     * @param p Pose2格式的位姿
     * @param H 可选项 雅可比矩阵 默认值为boost::none
     * 
     */
    gtsam::Vector evaluateError(const gtsam::Pose2&p, boost::optional<gtsam::Matrix&> H = boost::none) const {

            //将boost::optional看作指针
            //当传入非空指针时才进行雅可比的计算
            if(H) *H = (gtsam::Matrix23() << 1.0, 0.0, 0.0,
                                                                0.0, 1.0, 0.0).finished();

            return (gtsam::Vector2() << p.x() - mx_, p.y() - my_).finished();
    }

};

}//namespace gtsamexamples