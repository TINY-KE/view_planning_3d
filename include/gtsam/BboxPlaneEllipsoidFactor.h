/**
 *  @file   BboxPlaneEllipsoidFactor.h
 *  @brief  Gaussian prior defined on the workspace pose of any joint of a robot
 *          given its state in configuration space
 *  @author Mustafa Mukadam
 *  @date   Jan 8, 2018
 **/
#ifndef BboxPlaneEllipsoidFactorVector_H
#define BboxPlaneEllipsoidFactorVector_H

#pragma once

#include <gpmp2/kinematics/RobotModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include  "core/Plane.h"

namespace gpmp2 {

/**
 * Gaussian prior defined on the workspace pose
 */
template <class ROBOT>
class BboxPlaneEllipsoidFactorVector
    : public gtsam::NoiseModelFactorN<typename ROBOT::Pose> {

public:
    // typedefs
    typedef ROBOT Robot;
    typedef typename Robot::Pose Pose;

private:
    // typedefs
    typedef BboxPlaneEllipsoidFactorVector This;
    typedef gtsam::NoiseModelFactorN<Pose> Base;

    const Robot& robot_;     // Robot
    int joint_;              // joint on the robot to be constrained

public:
    /* Default constructor */
    BboxPlaneEllipsoidFactorVector() {}

    /// Constructor
    BboxPlaneEllipsoidFactorVector(gtsam::Key poseKey, const Robot& robot, int joint,
                             const gtsam::SharedNoiseModel& cost_model)
      : Base(cost_model, poseKey),
        robot_(robot),
        joint_(joint)
         {}

    virtual ~BboxPlaneEllipsoidFactorVector() {}


    // 计算平面和点之间的距离


    /// factor error function
    /// 输入参数： pose —— robot pose in config space
    gtsam::Vector evaluateError(
      const Pose& pose, gtsam::OptionalMatrixType H1 = nullptr) const override {
    using namespace gtsam;

    std::vector<Pose3> joint_pos;   //  link poses in 3D work space
    std::vector<Matrix> J_jpx_jp;   //  et al. optional Jacobians
    robot_.fk_model().forwardKinematics(pose, {}, joint_pos, {}, &J_jpx_jp);

    if (H1) {
        // Matrix66 H_ep;
        // Vector error = des_pose_.logmap(joint_pos[joint_], {}, H_ep);
        // *H1 = H_ep * J_jpx_jp[joint_];
        // return error;
    } else {
        // 计算椭球体和视场平面的距离。
        // return des_pose_.logmap(joint_pos[joint_]);
    }
    }

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /** print contents */
    void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "BboxPlaneEllipsoidFactorVector :" << std::endl;
    Base::print("", keyFormatter);
    }

private:
    #ifdef GPMP2_ENABLE_BOOST_SERIALIZATION
      /** Serialization function */
      friend class boost::serialization::access;
      template <class ARCHIVE>
      void serialize(ARCHIVE& ar, const unsigned int version) {
        ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      }
    #endif
};

}  // namespace gpmp2

#endif