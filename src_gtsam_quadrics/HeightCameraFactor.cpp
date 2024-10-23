/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BoundingBoxFactor.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief factor between Pose3 and ConstrainedDualQuadric
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam_quadrics/base/QuadricProjectionException.h>
#include <gtsam_quadrics/geometry/HeightCameraFactor.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>

#include <boost/bind/bind.hpp>

#define NUMERICAL_DERIVATIVE false

using namespace std;

namespace gtsam_quadrics {



/* ************************************************************************* */
gtsam::Vector HeightCameraFactor::evaluateError(
    const gtsam::Pose3& pose,
    gtsam::OptionalMatrixType H1) const {

  // 1.输入参数：
  // pose：一个 gtsam::Pose3 对象，表示相机的位姿。
  // quadric：一个 ConstrainedDualQuadric 对象，表示约束双二次曲面，通常代表一个椭球体或其他几何体。
  // H1：一个可选的输出参数，用于存储误差相对于 pose 的雅可比矩阵（4x6 矩阵）。
  // H2：一个可选的输出参数，用于存储误差相对于 quadric 的雅可比矩阵（4x9 矩阵）。

  // Quadric_.print("[HeightCameraFactor::evaluateError]: 椭球体信息：");

  // 2.投影和几何检查：
  // 在投影操作之前，首先检查 quadric 是否在相机的后面（isBehind）或者相机是否在 quadric 内部（contains）。如果满足这些条件，投影操作无效，会抛出 QuadricProjectionException 异常。
  try {
    // check pose-quadric pair
    if (Quadric_.isBehind(pose)) {
      throw QuadricProjectionException("Quadric is behind camera");
    }
    if (Quadric_.contains(pose)) {
      throw QuadricProjectionException("Camera is inside quadric");
    }

    // 3. 投影四次曲面：
    // project quadric taking into account partial derivatives
    // 使用 QuadricCamera::project 函数将双二次曲面 quadric 投影到 pose 所代表的相机坐标系中，生成一个 DualConic（双圆锥），表示在图像平面上的投影。
    Eigen::Matrix<double, 9, 6> dC_dx;   // 二次曲面对相机位姿的偏导
    Eigen::Matrix<double, 9, 9> dC_dq;
    // 将 quadric 投影到 pose 所在的相机坐标系中，生成 DualConic。
    DualConic dualConic;
    if (!NUMERICAL_DERIVATIVE) {
      dualConic = QuadricCamera::project(Quadric_, pose, gtsam_calibration_,
                                         {}, H1 ? &dC_dx : 0);
    } else {
      dualConic = QuadricCamera::project(Quadric_, pose, gtsam_calibration_);
    }

    // check dual conic is valid for error function
    // 如果投影结果不是椭圆（例如，是双曲线或其他形状），则无法继续计算误差，抛出异常。
    if (!dualConic.isEllipse()) {
      throw QuadricProjectionException("Projected Conic is non-ellipse");
    }

    // calculate conic bounds with derivatives
    bool computeJacobians = bool(H1) && !NUMERICAL_DERIVATIVE;
    // bbox对二维二次曲线的偏导
    Eigen::Matrix<double, 4, 9> db_dC;
    AlignedBox2 predictedBounds;
    // 图像平面上的预测边界框 predictedBounds:  通过 dualConic 计算
    // db_dC : bbox误差对 DualConic 的导数
    if (measurementModel_ == STANDARD) {
      predictedBounds = dualConic.bounds(computeJacobians ? &db_dC : 0);
    } else if (measurementModel_ == TRUNCATED) {
      try {
        predictedBounds =
            dualConic.smartBounds(gtsam_calibration_, computeJacobians ? &db_dC : 0);
      } catch (std::runtime_error& e) {
        throw QuadricProjectionException("smartbounds failed");
      }
    }


    // 4. 计算误差：
    // evaluate error
    // 如果投影的 DualConic 不是椭圆（例如，圆锥或双曲线），则抛出异常，表示无法计算误差。
    // 误差 ———— 预测的边界框（predictedBounds）和实际测量的边界框（measured_）之间的差异
    // 通过 dualConic.bounds() 或 dualConic.smartBounds() 计算预测的边界框（predictedBounds），该边界框与实际测量值（measured_）之间的差异即为误差。

    // 5. 计算雅可比矩阵：
    if (NUMERICAL_DERIVATIVE) {
      // std::function<gtsam::Vector(const gtsam::Pose3&,
      //                             const ConstrainedDualQuadric&)>
      //     funPtr(boost::bind(&HeightCameraFactor::evaluateError, this,
      //                        boost::placeholders::_1, boost::placeholders::_2,
      //                        nullptr, nullptr));
      // if (H1) {
      //   Eigen::Matrix<double, 4, 6> db_dx_ =
      //       gtsam::numericalDerivative21(funPtr, pose, Quadric_, 1e-6);
      //   *H1 = db_dx_;
      // }
      std::cerr << "错误采用NUMERICAL_DERIVATIVE. Exiting the program." << std::endl;
      std::exit(EXIT_FAILURE);  // 退出并返回状态码 1
    } else {
      // calculate derivative of error wrt pose
      if (H1) {
        // combine partial derivatives
        Eigen::Matrix<double, 1, 6> normal;
        normal << 0, 0, 0, 0, 0, 1;
        *H1 = normal;
      }

      // calculate derivative of error wrt quadric

    }
    gtsam::Vector1 error = gtsam::Vector1::Ones();
    double dis = 1.2 - pose.z();
    if(dis<0)
        error[0] = -1*dis;
    return error;

    // check for nans
    if (error.array().isInf().any() || error.array().isNaN().any() ||
        (H1 && (H1->array().isInf().any() || H1->array().isNaN().any()))
        ) {
      throw std::runtime_error("nan/inf error in bbf");
    }

    // handle projection failures
  } catch (QuadricProjectionException& e) {
    // std::cout << "  Landmark " << symbolIndex(this->objectKey()) << "
    // received: " << e.what() << std::endl;

    // if error cannot be calculated
    // set error vector and jacobians to zero
    gtsam::Vector1 error = gtsam::Vector1::Ones() * 1000.0;
    if (H1) {
      *H1 = gtsam::Matrix::Zero(1, 6);
    }

    return error;
  }
}



/* ************************************************************************* */
void HeightCameraFactor::print(const std::string& s,
                              const gtsam::KeyFormatter& keyFormatter) const {
  cout << s << "HeightCameraFactor(" << keyFormatter(key1())  << endl;
  measured_.print("    Measured: ");
  cout << "    NoiseModel: ";
  noiseModel()->print();
  cout << endl;
}

/* ************************************************************************* */
bool HeightCameraFactor::equals(const HeightCameraFactor& other,
                               double tol) const {
  bool equal = measured_.equals(other.measured_, tol) &&
               gtsam_calibration_->equals(*other.gtsam_calibration_, tol) &&
               noiseModel()->equals(*other.noiseModel(), tol) ;
  return equal;
}

}  // namespace gtsam_quadrics
