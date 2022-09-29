/**
 * @file Point2c.h
 * @author gongkeyang 
 * @brief 
 * @version 0.1
 * @date 2022-08-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once   //有的编译器支持，有的编译器不支持，为了避免使一个头文件被include多次，保证头文件只被编译一次

#include<gtsam/base/Matrix.h>
#include<gtsam/base/Vector.h>
#include<gtsam/base/Lie.h>

#include<cmath>
#include<iostream>

namespace gtsamexamples{

//最小的2D点类，'c' means custum 定制的
struct Point2c{
    double x;
    double y;

//构造函数
    Point2c(double xi, double yi) : x(xi), y(yi){}
};

}//namespace gtsamexamples

/**
 * @brief traits for Point2c / Point2c应该具备的特征
 * 任何与gtsam兼容的类型必须包含以下特征：
 *  -Print function with an optional begining string, in foramt：          可选开头字符串的打印函数，结构为：
 *     static void Print(const T& m, const std::string& str = "");
 * - Equal function with optional tolerance, in format:                             带有度量的等价函数，判断是否相等，结构为：
 *     static void Equal(const T& m1, const T& m2, const double tol = 1e-8);
 * 
 * 流形的类必须包含：
 *  -int dimension 维度信息
 *  -Typedefs TangentVector 定义切向量，二维点结构则是二维向量  TangentVector = Eigen::Matrix<double, dimension, 1>
 *  -Local coordinate function 求局部坐标，结构为：
 *     static TangentVector Local(const Class& origin, const Class& other);
 *  -Retraction back to manifold，回退到流形，结构为：
 *     static Class Retract(const Class& origin, const TangentVector& v);
 * 
 * 李群类型中必须包含：
 *  -Identity function 单位化函数
 *  -Logmap function, with optional jacobians
 *  -Expmap function, with optional jacobians
 *  -Compose function, with optional jacobians
 *  -Between function, with optional jacobians
 *  -Inverse function, with optional jacobians
 * 
 * 对于李群类，operator * or (+ and -) 应该在compose / between运算中被重载
 *  可以将其定义在类内或类外
 *  在本例中operator *定义在类外
 */



//traits must in namespace gtsam

namespace gtsam{

template<> //函数模板的特化 由于统一的函数模板无法在所有实例下工作，需要定义类型参数在实例化为特定类型时函数模板的特定实现版本
struct traits<gtsamexamples::Point2c>{

    //structural category(结构类别): this is a lie group
    //available options: manifold_tag, group_tag, lie_group_tag
    typedef lie_group_tag structure_category;

    /**
     * @brief Basic (Testable)
     * 
     */

        //print
        static void Print(const gtsamexamples::Point2c& m, const std::string& str = ""){
            std::cout << str << "(" << m.x << ", " << m.y << ")" << std::endl;
        }

        //判断相等的函数，可选度量
        static bool Equals(const gtsamexamples::Point2c& m1, const gtsamexamples::Point2c& m2, double tol = 1e-8){
            if(fabs(m1.x - m2.x) < tol && fabs(m1.y - m2.y) < tol)
            return true;
            else
            return false;
        }


        /**
         * @brief Manifold 流形
         * 
         */

        //use enum dimension
        enum {dimension = 2};
        static int GetDimension(const gtsamexamples::Point2c&) { return dimension;}

        //Typedefs needed
        typedef gtsamexamples::Point2c ManifoldType;
        typedef Eigen::Matrix<double, dimension, 1> TangentVector;

        //Point2c的局部坐标
        static TangentVector Local(const gtsamexamples::Point2c& origin, const gtsamexamples::Point2c& other){
            return Vector2(other.x - origin.x, other.y - origin.y);
        }

        //将Point2c回退到流行
        static gtsamexamples::Point2c Retract(const gtsamexamples::Point2c& origin, const TangentVector& v){
            return gtsamexamples::Point2c(origin.x + v(0), origin.y + v(1));
        }

        /**
         * @brief lie group李群
         * 
         */

        //注意李群运算使用operator *，如果使用+/-，则需要选择 additive_group_tag
        typedef multiplicative_group_tag group_flavor;

        //typedefs
        typedef OptionalJacobian<dimension, dimension> ChartJacobian;

        static gtsamexamples::Point2c Identity() {
            return gtsamexamples::Point2c(0, 0);
        }

        static TangentVector Logmap(const gtsamexamples::Point2c& m, ChartJacobian Hm = boost::none){
            if(Hm) *Hm = Matrix2::Identity();
            return Vector2(m.x, m.y);
        }

        static gtsamexamples::Point2c Expmap(const TangentVector& v, ChartJacobian Hv = boost::none){
            if(Hv) *Hv = Matrix2::Identity();
            return gtsamexamples::Point2c(v(0), v(1));
        }

        static gtsamexamples::Point2c Compose(const gtsamexamples::Point2c& m1, const gtsamexamples::Point2c& m2, 
            ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none){
            if(H1) *H1 = Matrix2::Identity();
            if(H2) *H2 = Matrix2::Identity();
            return gtsamexamples::Point2c(m1.x + m2.x, m1.y + m2.y);
        }

        static gtsamexamples::Point2c Between(const gtsamexamples::Point2c& m1, const gtsamexamples::Point2c& m2, 
            ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
            if (H1) *H1 = -Matrix2::Identity();
            if (H2) *H2 = Matrix2::Identity();
            return gtsamexamples::Point2c(m2.x - m1.x, m2.y - m1.y);
        }

        static gtsamexamples::Point2c Inverse(const gtsamexamples::Point2c& m, ChartJacobian H = boost::none){
            if(H) *H = -Matrix2::Identity();
            return gtsamexamples::Point2c(-m.x, -m.y);
        }
};

}//namespace gtsam

namespace gtsamexamples{
    
    //operator *
    Point2c operator*(const Point2c& m1, const Point2c& m2){
        return Point2c(m1.x + m2.x, m1.y + m2.y);
    }
}