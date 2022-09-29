/**
 * @file functions.cpp
 * @author gongkeyang
 * @brief functions for expressions
 * @version 0.1
 * @date 2022-08-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "functions.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

namespace gtsamexamples{

gtsam::Point2 projectPose2(const gtsam::Pose2& pose, gtsam::OptionalJacobian<2, 3> H){
    if(H) *H = (gtsam::Matrix23() << 1.0, 0.0, 0.0,
                                                            0.0, 1.0, 0.0).finished();
    return gtsam::Point2(pose.x(), pose.y());
}

}