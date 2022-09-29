/**
 * @file CustomPoint2Example.cpp
 * @author gongkeyang (you@domain.com)
 * @brief 2D example with custom Point2c type
 * @version 0.1
 * @date 2022-08-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/**
 * @brief 简单的2D开环位姿图例子
 * The robot moves from x1 to x5 with 2 stepsize, with odometry information between each pair
 * prior factor on fisrt state, remainings are open-loop
 * The graph structure is shown:
 * 
 * p -> x1 - x2 - x3 - x4 - x5
 * 
 */

#include "Point2c.h"

//GTSAM headers
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;
using namespace gtsamexamples;

int main(int argc, char** argv){

    //创建因子图
    NonlinearFactorGraph graph;

    //先验噪声模型，协方差矩阵（权重）
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas(Vector2(0.2, 0.2));

    //加入先验因子
    graph.add(PriorFactor<Point2c>(Symbol('x', 1), Point2c(0, 0), priorModel));

    //里程计测量噪声模型，协方差矩阵
    noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.5));

    //加入里程计因子
    //在连续的point2c之间创建里程计因子
    graph.add(BetweenFactor<Point2c>(Symbol('x', 1), Symbol('x', 2), Point2c(2, 0), odomModel));
    graph.add(BetweenFactor<Point2c>(Symbol('x', 2), Symbol('x', 3), Point2c(2, 0), odomModel));
    graph.add(BetweenFactor<Point2c>(Symbol('x', 3), Symbol('x', 4), Point2c(2, 0), odomModel));
    graph.add(BetweenFactor<Point2c>(Symbol('x', 4), Symbol('x', 5), Point2c(2, 0), odomModel));

    //print factor graph
    graph.print("\nFactor Graph:\n");
    

    //设置优化的初值
    //对真值加入随机噪声得到
    Values initials;
    initials.insert(Symbol('x', 1), Point2c(0.2, -0.3));
    initials.insert(Symbol('x', 2), Point2c(2.1, 0.3));
    initials.insert(Symbol('x', 3), Point2c(3.9, -0.1));
    initials.insert(Symbol('x', 4), Point2c(5.9, -0.3));
    initials.insert(Symbol('x', 5), Point2c(8.2, 0.1));

    initials.print("\nInitial Values:\n");

    //使用高斯牛顿法优化
    GaussNewtonParams parameters;

    //每次迭代都要打印
    parameters.setVerbosity("ERROR");
    
    //优化开始
    GaussNewtonOptimizer optimizer(graph, initials, parameters);
    Values results = optimizer.optimize();

    //打印最终结果
    results.print("Final Results:\n");

    Marginals marginals(graph, results);
    //打印marginal covariances 当前位姿的协方差，也就是当前位姿的不确定程度
    cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;
    cout << "x2 covariance:\n" << marginals.marginalCovariance(Symbol('x', 2)) << endl;
    cout << "x3 covariance:\n" << marginals.marginalCovariance(Symbol('x', 3)) << endl;
    cout << "x4 covariance:\n" << marginals.marginalCovariance(Symbol('x', 4)) << endl;
    cout << "x5 covariance:\n" << marginals.marginalCovariance(Symbol('x', 5)) << endl;

    return 0;
}