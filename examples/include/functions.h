/**
 * @file functions.h
 * @author gongkeyang
 * @brief functions for expressions
 * @version 0.1
 * @date 2022-08-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

namespace gtsamexamples{

//function project from Pose2 to Point2
gtsam::Point2 projectPose2(const gtsam::Pose2& pose, gtsam::OptionalJacobian<2,3> H = boost::none);

}//namespace gtsamexamples