//参照liosam图优化需要的的头文件
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
//isam2
#include <gtsam/nonlinear/ISAM2.h>

//平面约束需要的头文件
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/geometry/OrientedPlane3.h>

using namespace std;
using namespace gtsam;
/*以下因子图的构建利用了gtsam自带的OrientedPlane.h平面属性以及OrientedPlane3Factor.h中构建的边
                    x1->odom->x2
                        \                   /
                            plane0
    x1与x2之间为里程计约束
    x1和x2都由平面plane0约束
*/
int main (int argc, char** argv){

    NonlinearFactorGraph graph;
    Values initialEstimate;
    Values optimizedEstimate;
    
    ISAM2 isam2;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    //确定因子图中各个因子的噪声
    noiseModel::Diagonal::shared_ptr PriorModel = noiseModel::Diagonal::Sigmas((Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished());
    noiseModel::Diagonal::shared_ptr PlaneModel = noiseModel::Diagonal::Sigmas(Vector3(0.005, 0.005, 0.005));
    noiseModel::Diagonal::shared_ptr OdomModel = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

    //Step 1 创建各个节点
    //位姿节点
    //给节点命名
    Symbol init_pose_node('x', 0);
    //给定节点的预测值
    Pose3 init_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
    //设置节点的预测值(优化的初值)
    initialEstimate.insert(init_pose_node, init_pose);

    Symbol second_pose_node('x', 1);
    Pose3 second_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(1.0, 0.0, 1.0));
    initialEstimate.insert(second_pose_node, second_pose);

    //平面节点
    Symbol init_plane_node('p', 0);
    OrientedPlane3 init_plane(0.0, 0.0, 1.0, 0.0);
    initialEstimate.insert(init_plane_node, init_plane);

    //Step2 往图中加入因子
        //先验因子
    PriorFactor<Pose3> pose_prior(init_pose_node, init_pose, PriorModel);
    graph.add(pose_prior);

        //里程计因子
    Pose3 between_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(1.0, 0.0, 1.0));
    BetweenFactor<Pose3> odom_meas(init_pose_node, second_pose_node, between_pose, OdomModel);
    graph.add(odom_meas);

        //平面约束因子
            //确定测量值
    Vector test_meas0(4);
    test_meas0 << 0.0, 0.0, 1.0, 0.0;
    //给0号位姿加入平面约束
    OrientedPlane3Factor plane_test0(test_meas0, PlaneModel, init_pose_node, init_plane_node);
    graph.add(plane_test0);
    //给1号位姿加入平面约束
    OrientedPlane3Factor plane_test2(test_meas0, PlaneModel, second_pose_node, init_plane_node);
    graph.add(plane_test2);

    /*
    Vector test_meas1(4);
    test_meas1 << -1.0, 0.0, 0.0, 1.0;
    OrientedPlane3Factor plane_test1(test_meas1, PlaneModel, init_pose_node, init_plane_node);
    graph.add(plane_test1);
    */

    //Step3 以上已经构建好了整个图，下面开始优化
    isam2.update(graph, initialEstimate);
    isamCurrentEstimate = isam2.calculateEstimate();
    OrientedPlane3 optimized_plane = isamCurrentEstimate.at<OrientedPlane3>(init_plane_node);
    Pose3 optimized_pose1 = isamCurrentEstimate.at<Pose3>(init_pose_node);
    Pose3 optimized_pose2 = isamCurrentEstimate.at<Pose3>(second_pose_node);

    //输出优化后的结果
    optimized_plane.print("plane");
    optimized_pose1.print("pose1");
    optimized_pose2.print("pose2");
    return 0;
}