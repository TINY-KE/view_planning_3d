/**
 *  @file  BboxPlaneArmLink.h
 *  @brief Obstacle avoidance cost factor, using 3D signed distance field
 *  @author Jing Dong
 *  @date  May 11, 2016
 **/

#ifndef GTSAM_BBOX_PLANE_ARM_LINK_H
#define GTSAM_BBOX_PLANE_ARM_LINK_H

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>

#include "core/Plane.h"

namespace gpmp2 {

/**
 * unary factor for obstacle avoidance
 * template robot model version
 */
template <class ROBOT>
class BboxPlaneArmLink
    : public gtsam::NoiseModelFactorN<typename ROBOT::Pose> {
 public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;

 private:
  // typedefs
  typedef BboxPlaneArmLink This;
  typedef gtsam::NoiseModelFactorN<Pose> Base;

  // obstacle cost settings
  double epsilon_;  // distance from object that start non-zero cost

  // arm: planar one, all alpha = 0
  const Robot& robot_;

 public:
  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /* Default constructor */
  BboxPlaneArmLink() {}

  /**
   * Constructor
   * @param cost_model cost function covariance, should to identity model
   * @param field      signed distance field
   * @param nn_index   nearest neighbour index of signed distance field
   */
  BboxPlaneArmLink(gtsam::Key poseKey, const Robot& robot,
                    double cost_sigma,
                    double epsilon)
      : Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(),
                                                 cost_sigma),
             poseKey),
        epsilon_(epsilon),
        robot_(robot)
        {}

  virtual ~BboxPlaneArmLink() {}

  /// error function
  /// numerical jacobians / analytic jacobians from cost function
  gtsam::Vector evaluateError(
      const typename Robot::Pose& conf,
      gtsam::OptionalMatrixType H1 = nullptr) const override;

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "BboxPlaneArmLink :" << std::endl;
    Base::print("", keyFormatter);
  }

#ifdef GPMP2_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int version) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gpmp2















double hingeLossFovCost(
    const gtsam::Point3& point, g2o::plane* plane_low, double eps,
    gtsam::OptionalJacobian<1, 3> H_point = nullptr) {

      gtsam::Vector3 field_gradient;  //距离对Point3的导数，
      double dist_signed;   //Point3与最低视场平面之间的最近距离

      //（注意平面的normal是向上，因此带入平面方程后的距离结果要取负数）
      dist_signed = -1 * plane_low->distanceToPoint(point, true);

      if (dist_signed > eps) {
         // faraway no error
         if (H_point) *H_point = gtsam::Matrix13::Zero();
         return 0.0;
      } else {
       // outside but < eps or inside object
       if (H_point) {
          // ERROR增加的方向是靠近障碍物的方向，与平面的normal方向相同。
          field_gradient = plane_low->normal();
          field_gradient.normalize();
          *H_point = field_gradient.transpose();
       }
       return eps - dist_signed;
      }

}












#include <gpmp2/obstacle/ObstacleCost.h>

using namespace std;
using namespace gtsam;

namespace gpmp2 {

 /* ************************************************************************** */
 template <class ROBOT>
 gtsam::Vector BboxPlaneArmLink<ROBOT>::evaluateError(
     const typename Robot::Pose& conf, gtsam::OptionalMatrixType H1) const {

  // 计算视场的下平面
  g2o::plane* plane_low;

  // if Jacobians used, initialize as zeros
  // size: arm_nr_points_ * DOF
  if (H1) *H1 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());

  // run forward kinematics of this configuration
  vector<Point3> sph_centers;
  vector<Matrix> J_px_jp;
  if (H1)
   robot_.sphereCenters(conf, sph_centers, &J_px_jp);
  else
   robot_.sphereCenters(conf, sph_centers);

  // allocate cost vector
  Vector err(robot_.nr_body_spheres());

  // for each point on arm stick, get error
  for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {
   const double total_eps = robot_.sphere_radius(sph_idx) + epsilon_;

   if (H1) {
    Matrix13 Jerr_point;
    err(sph_idx) = hingeLossFovCost(sph_centers[sph_idx], plane_low,
                                         total_eps, Jerr_point);

    // chain rules
    H1->row(sph_idx) = Jerr_point * J_px_jp[sph_idx];

   } else {
    err(sph_idx) =
        hingeLossFovCost(sph_centers[sph_idx], plane_low, total_eps);
   }
  }

  return err;
 }

}  // namespace gpmp2


#endif
