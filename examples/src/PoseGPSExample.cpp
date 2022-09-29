/**
 * @file PoseGPSExample.cpp
 * @author gongkeyang
 * @brief 2D example for custom 'GPS' factor
 * @version 0.1
 * @date 2022-08-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/**
 * @brief 一个简单的2D图优化SLAM系统，添加'GPS'观测
 * The robot moves from x1 to x3, with odometry infomation between each other
 * each step has an associated 'GPS' measurement by GPSPose2Factor
 * 图结构如下
 * 
 * g1   g2   g3
 *   |      |       |
 * x1 - x2 - x3
 * 
 */

#include "GPSPose2Factor.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;
using namespace gtsamexamples;

int main(int argc, char** argv){

    //创建因子图容器
    NonlinearFactorGraph graph;

    //创建里程计测量噪声
    noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));
    //加入里程计因子
    graph.add(BetweenFactor<Pose2>(Symbol('x', 1), Symbol('x', 2), Pose2(5, 0, 0), odomModel));
    graph.add(BetweenFactor<Pose2>(Symbol('x', 2), Symbol('x', 3), Pose2(5, 0, 0), odomModel));

    //创建GPS噪声
    noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(Vector2(1.0, 1.0));
    //加入GPS因子
    //注意这里对于第一帧没有先验因子，是因为GPS提供了全局位姿（通过两个GPS以上的GPS数据也可以得到旋转数据）
    graph.add(GPSPose2Factor(Symbol('x', 1), Point2(0, 0), gpsModel));
    graph.add(GPSPose2Factor(Symbol('x', 2), Point2(5, 0), gpsModel));
    graph.add(GPSPose2Factor(Symbol('x', 3), Point2(10, 0), gpsModel));

    //print factor graph
    graph.print("\nFactor Graph:\n");

    //设置优化的初始值，在真值的基础上加入随机噪声
    Values initials;
    initials.insert(Symbol('x', 1), Pose2(0.2, -0.3, 0.2));
    initials.insert(Symbol('x', 2), Pose2(5.1, 0.3, -0.1));
    initials.insert(Symbol('x', 3), Pose2(9.9, -0.1, -0.2));

    //print initial values
    initials.print("\nInitials Values:\n");

    //采用高斯牛顿法优化初始值

    GaussNewtonParams parameters;

    //print per itetation
    parameters.setVerbosity("ERROR");

    GaussNewtonOptimizer optimizer(graph, initials, parameters);
    Values results = optimizer.optimize();

    //print final results
    results.print("Final Results:\n");

    //计算每个位姿的协方差矩阵

    Marginals marginals(graph, results);

    cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;
    cout << "x2 covariance:\n" << marginals.marginalCovariance(Symbol('x', 2)) << endl;
    cout << "x3 covariance:\n" << marginals.marginalCovariance(Symbol('x', 3)) << endl;


    return 0;
}