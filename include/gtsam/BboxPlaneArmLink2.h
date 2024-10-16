/**
 *  @file  BboxPlaneArmLink.h
 *  @brief Obstacle avoidance cost factor, using 3D signed distance field
 *  @author Jing Dong
 *  @date  May 11, 2016
 **/

#ifndef GTSAM_BBOX_PLANE_ARM_LINK_H
#define GTSAM_BBOX_PLANE_ARM_LINK_H

#pragma once

#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>

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

  // signed distance field from matlab
  const SignedDistanceField& sdf_;

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
                    const SignedDistanceField& sdf, double cost_sigma,
                    double epsilon)
      : Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(),
                                                 cost_sigma),
             poseKey),
        epsilon_(epsilon),
        robot_(robot),
        sdf_(sdf) {}

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


#include <gpmp2/obstacle/ObstacleCost.h>

using namespace std;
using namespace gtsam;

namespace gpmp2 {

 /* ************************************************************************** */
 template <class ROBOT>
 gtsam::Vector BboxPlaneArmLink<ROBOT>::evaluateError(
     const typename Robot::Pose& conf, gtsam::OptionalMatrixType H1) const {
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
    err(sph_idx) = hingeLossObstacleCost(sph_centers[sph_idx], sdf_,
                                         total_eps, Jerr_point);

    // chain rules
    H1->row(sph_idx) = Jerr_point * J_px_jp[sph_idx];

   } else {
    err(sph_idx) =
        hingeLossObstacleCost(sph_centers[sph_idx], sdf_, total_eps);
   }
  }

  return err;
 }

}  // namespace gpmp2


#endif
