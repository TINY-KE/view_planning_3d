/**
 *  @file   BboxEllipsoidFactor.h
 *  @brief  Gaussian prior defined on the workspace pose of any joint of a robot
 *          given its state in configuration space
 *  @author zhang jiadong
 *  @date   Jan 8, 2018
 **/
#ifndef BboxEllipsoidFactor_H
#define BboxEllipsoidFactor_H

#pragma once

#include <gpmp2/kinematics/RobotModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "MapObject.h"
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_quadrics/geometry/AlignedBox2.h>
#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <gtsam_quadrics/geometry/BoundingBoxFactor.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>
#include <gtsam_quadrics/base/QuadricProjectionException.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/bind/bind.hpp>

#include "Converter.h"

typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Matrix<double, 3, 8> Matrix38d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;

// #define NUMERICAL_DERIVATIVE false

using namespace gtsam_quadrics;

namespace gpmp2 {
    /**
     * Gaussian prior defined on the workspace pose
     */
    template<class ROBOT>
    class BboxEllipsoidFactor
            : public gtsam::NoiseModelFactorN<typename ROBOT::Pose> {
    public:
        enum MeasurementModel {
            STANDARD,
            TRUNCATED
        };

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef ROBOT Robot;
        typedef typename Robot::Pose Pose;

    private:
        typedef BboxEllipsoidFactor This;
        typedef gtsam::NoiseModelFactorN<Pose> Base;

        const Robot &robot_; // Robot

        int miImageCols, miImageRows;


        // 目标物体。物体的位姿实在world坐标系中
        ConstrainedDualQuadric Quadric_;

        //用于 bbox与Pose3的误差
        AlignedBox2 measured_; ///< measured bounding box
        boost::shared_ptr<gtsam::Cal3_S2> gtsam_calibration_;
        // = boost::make_shared<gtsam::Cal3_S2>(fx, fy, 0.0, cx, cy);
        MeasurementModel measurementModel_;

    public:
        /* Default constructor */
        BboxEllipsoidFactor() {
        }

        /// Constructor
        BboxEllipsoidFactor(gtsam::Key poseKey, const Robot &robot,
                            double cost_sigma,
                            const AlignedBox2& measured,
                            MapObject *ob_,
                            Eigen::Matrix4f mRobotPose_,
                            int width, int height, Matrix3d calib)
            : Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(),
                                                       cost_sigma),
                   poseKey),
              robot_(robot),
              miImageCols(width),
              miImageRows(height) {
            // 生成世界坐标系下的目标物体
            GenerateConstrainedDualQuadric(ob_,mRobotPose_);
            // 生成 内参
            GenerateCalibration(calib);
            //
            measurementModel_ = STANDARD;

        }

        virtual ~BboxEllipsoidFactor() {
        }


        /// factor error function
        /// 输入参数： pose —— robot pose in config space
        /// numerical jacobians / analytic jacobians from cost function
        // zhjd: 此处的Robot::Pose对应的是gtsam::Vector。   根据 ForwardKinematics<gtsam::Vector, gtsam::Vector>。 因此这里的pose就是关节角度。
        gtsam::Vector evaluateError(
            const typename Robot::Pose &conf,
            gtsam::OptionalMatrixType H1 = nullptr
            ) const override;

        gtsam::Vector getPredictedBounds(
            const typename Robot::Pose &conf,
            gtsam::OptionalMatrixType H1 = nullptr
            ) const;

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return std::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        /** print contents */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter =
                           gtsam::DefaultKeyFormatter) const {
            std::cout << s << "BboxEllipsoidFactor :" << std::endl;
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

    public:
        void GenerateConstrainedDualQuadric(MapObject *object, Eigen::Matrix4f& T_world_baselink) {
            // 一、将物体的旋转矩阵，转为四元数
            auto matrix = object->mCuboid3D.pose_mat;
            Eigen::Matrix4d T_world_object = Converter::cvMattoMatrix4d(matrix);
            Eigen::Matrix4d T_baselink_object = T_world_baselink.cast<double>().inverse() * T_world_object;

            // 二、物体的尺寸
            double lenth = object->mCuboid3D.lenth;
            double width = object->mCuboid3D.width;
            double height = object->mCuboid3D.height;

            // 三、转为ConstrainedDualQuadric
            //                Eigen::Matrix4d matrix_4x4;
            //                matrix_4x4 << 1, 0, 0, x,
            //                              0, 1, 0, y,
            //                              0, 0, 1, z,
            //                              0, 0, 0, 1;
            //                ConstrainedDualQuadric Quadric(gtsam::Pose3(matrix_4x4), gtsam::Vector3(lenth/2.0,width/2.0,height/2.0));
            Quadric_ = ConstrainedDualQuadric(gtsam::Pose3(T_baselink_object),
                                                              gtsam::Vector3(lenth / 2.0, width / 2.0, height / 2.0));
        }

        void GenerateCalibration(Matrix3d &calib) {
            //            Eigen::Matrix3d Calib;
            //            Calib << fx,  0,  cx,
            //                      0,  fy,  cy,
            //                      0,   0,   1;
            //            boost::shared_ptr<gtsam::Cal3_S2> gtsam_calib = boost::make_shared<gtsam::Cal3_S2>(fx, fy, 0.0, cx, cy);
            gtsam_calibration_ = boost::make_shared<gtsam::Cal3_S2>(calib(0, 0), calib(1, 1), 0.0, calib(0, 2),
                                                                    calib(1, 2));
        }
    };
}


#include <gpmp2/obstacle/ObstacleCost.h>
#include <vector>
using namespace std;
using namespace gtsam;

namespace gpmp2 {
    /* ************************************************************************** */


    template<class ROBOT>
    gtsam::Vector BboxEllipsoidFactor<ROBOT>::evaluateError(
        const typename Robot::Pose &conf, gtsam::OptionalMatrixType H1) const {

        // 一, 获取相机位姿（相对于baselink），获取dx_djoint偏导
        std::vector<Pose3> joint_pos;
        std::vector<Matrix> dx_djoint;
        robot_.fk_model().forwardKinematics(conf, {}, joint_pos, {}, &dx_djoint);
        gtsam::Pose3 T_baselink_to_endlink = joint_pos[joint_pos.size()-1];
        Eigen::Matrix4d T_endlink_to_c;
        T_endlink_to_c <<   0, 0, 1, 0.02,
                            -1, 0, 0, -0.013,
                            0, -1, 0, 0.07, //实际为0.13(用于ros)，改为0.07(用于gpmp2 forwardKinematics)
                            0, 0, 0, 1;
        Eigen::Matrix4d T_baselink_to_camera = T_baselink_to_endlink.matrix() * T_endlink_to_c;
        gtsam::Pose3 camera_pose(T_baselink_to_camera);

        // 二、 通过bbox计算Error，并获得 db_dC 和 dC_dx 偏导
        try {
            // 2.投影和几何检查：
            // 在投影操作之前，首先检查 quadric 是否在相机的后面（isBehind）或者相机是否在 quadric 内部（contains）。如果满足这些条件，投影操作无效，会抛出 QuadricProjectionException 异常。
            // check pose-quadric pair
            if (Quadric_.isBehind(camera_pose)) {
                throw QuadricProjectionException("Quadric is behind camera");
            }
            if (Quadric_.contains(camera_pose)) {
                throw QuadricProjectionException("Camera is inside quadric");
            }

            // 3. 投影四次曲面：
            // project quadric taking into account partial derivatives
            // 使用 QuadricCamera::project 函数将双二次曲面 quadric 投影到 pose 所代表的相机坐标系中，生成一个 DualConic（双圆锥），表示在图像平面上的投影。
            Eigen::Matrix<double, 9, 6> dC_dx; // 二次曲面对相机位姿的偏导
            // 将 quadric 投影到 pose 所在的相机坐标系中，生成 DualConic。
            DualConic dualConic;
            dualConic = QuadricCamera::project(Quadric_, camera_pose, gtsam_calibration_);


            // check dual conic is valid for error function
            // 如果投影结果不是椭圆（例如，是双曲线或其他形状），则无法继续计算误差，抛出异常。
            if (!dualConic.isEllipse()) {
                throw QuadricProjectionException("Projected Conic is non-ellipse");
            }

            // calculate conic bounds with derivatives
            bool computeJacobians = bool(H1);
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
                } catch (std::runtime_error &e) {
                    throw QuadricProjectionException("smartbounds failed");
                }
            }


            // 4. 计算误差：
            // evaluate error
            // 如果投影的 DualConic 不是椭圆（例如，圆锥或双曲线），则抛出异常，表示无法计算误差。
            // 误差 ———— 预测的边界框（predictedBounds）和实际测量的边界框（measured_）之间的差异
            // 通过 dualConic.bounds() 或 dualConic.smartBounds() 计算预测的边界框（predictedBounds），该边界框与实际测量值（measured_）之间的差异即为误差。
            gtsam::Vector4 error = predictedBounds.vector() - measured_.vector();


            // 5. 计算雅可比矩阵：
            // calculate derivative of error wrt joints
            if (H1) {
                // combine partial derivatives
                *H1 = db_dC * dC_dx * dx_djoint[dx_djoint.size()-1];
                std::cout << " [debug0] db_dC = " << std::endl << db_dC << std::endl;
                std::cout << " [debug0] dC_dx = " << std::endl << dC_dx << std::endl;
                std::cout << " [debug0] dx_djoint = " << std::endl << dx_djoint[dx_djoint.size()-1] << std::endl;
            }

            // calculate derivative of error wrt quadric



            return error;

            // check for nans
            if (    error.array().isInf().any() || error.array().isNaN().any() ||
                (H1 && (H1->array().isInf().any() || H1->array().isNaN().any()))    ) {
                throw std::runtime_error("nan/inf error in bbf");
            }

            // handle projection failures
        } catch (QuadricProjectionException &e) {
            // std::cout << "  Landmark " << symbolIndex(this->objectKey()) << "
            // received: " << e.what() << std::endl;

            // if error cannot be calculated
            // set error vector and jacobians to zero
            gtsam::Vector4 error = gtsam::Vector4::Ones() * 1000.0;
            if (H1) {
                *H1 = gtsam::Matrix::Zero(4, 6);
            }

            return error;
        }
    }

    template<class ROBOT>
    gtsam::Vector BboxEllipsoidFactor<ROBOT>::getPredictedBounds(
        const typename Robot::Pose &conf, gtsam::OptionalMatrixType H1) const {

        // 一, 获取相机位姿（相对于baselink），获取dx_djoint偏导
        std::vector<Pose3> joint_pos;
        std::vector<Matrix> dx_djoint;
        robot_.fk_model().forwardKinematics(conf, {}, joint_pos, {}, &dx_djoint);
        gtsam::Pose3 T_baselink_to_endlink = joint_pos[joint_pos.size()-1];
        Eigen::Matrix4d T_endlink_to_c;
        T_endlink_to_c <<   0, 0, 1, 0.02,
                            -1, 0, 0, -0.013,
                            0, -1, 0, 0.07, //实际为0.13(用于ros)，改为0.07(用于gpmp2 forwardKinematics)
                            0, 0, 0, 1;
        Eigen::Matrix4d T_baselink_to_camera = T_baselink_to_endlink.matrix() * T_endlink_to_c;
        gtsam::Pose3 camera_pose(T_baselink_to_camera);

        // 二、 通过bbox计算Error，并获得 db_dC 和 dC_dx 偏导
        try {
            // 2.投影和几何检查：
            // 在投影操作之前，首先检查 quadric 是否在相机的后面（isBehind）或者相机是否在 quadric 内部（contains）。如果满足这些条件，投影操作无效，会抛出 QuadricProjectionException 异常。
            // check pose-quadric pair
            if (Quadric_.isBehind(camera_pose)) {
                throw QuadricProjectionException("Quadric is behind camera");
            }
            if (Quadric_.contains(camera_pose)) {
                throw QuadricProjectionException("Camera is inside quadric");
            }

            // 3. 投影四次曲面：
            // project quadric taking into account partial derivatives
            // 使用 QuadricCamera::project 函数将双二次曲面 quadric 投影到 pose 所代表的相机坐标系中，生成一个 DualConic（双圆锥），表示在图像平面上的投影。
            Eigen::Matrix<double, 9, 6> dC_dx; // 二次曲面对相机位姿的偏导
            // 将 quadric 投影到 pose 所在的相机坐标系中，生成 DualConic。
            DualConic dualConic;
            dualConic = QuadricCamera::project(Quadric_, camera_pose, gtsam_calibration_);


            // check dual conic is valid for error function
            // 如果投影结果不是椭圆（例如，是双曲线或其他形状），则无法继续计算误差，抛出异常。
            if (!dualConic.isEllipse()) {
                throw QuadricProjectionException("Projected Conic is non-ellipse");
            }

            // calculate conic bounds with derivatives
            bool computeJacobians = bool(H1);
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
                } catch (std::runtime_error &e) {
                    throw QuadricProjectionException("smartbounds failed");
                }
            }


            // 4. 计算误差：
            // evaluate error
            // 如果投影的 DualConic 不是椭圆（例如，圆锥或双曲线），则抛出异常，表示无法计算误差。
            // 误差 ———— 预测的边界框（predictedBounds）和实际测量的边界框（measured_）之间的差异
            // 通过 dualConic.bounds() 或 dualConic.smartBounds() 计算预测的边界框（predictedBounds），该边界框与实际测量值（measured_）之间的差异即为误差。
            gtsam::Vector4 error = predictedBounds.vector() - measured_.vector();


            // 5. 计算雅可比矩阵：
            // calculate derivative of error wrt joints
            if (H1) {
                // combine partial derivatives
                *H1 = db_dC * dC_dx * dx_djoint[dx_djoint.size()-1];
                std::cout << " [debug0] db_dC = " << std::endl << db_dC << std::endl;
                std::cout << " [debug0] dC_dx = " << std::endl << dC_dx << std::endl;
                std::cout << " [debug0] dx_djoint = " << std::endl << dx_djoint[dx_djoint.size()-1] << std::endl;
            }

            // calculate derivative of error wrt quadric



            return predictedBounds.vector();

            // check for nans
            if (    error.array().isInf().any() || error.array().isNaN().any() ||
                (H1 && (H1->array().isInf().any() || H1->array().isNaN().any()))    ) {
                throw std::runtime_error("nan/inf error in bbf");
            }

            // handle projection failures
        } catch (QuadricProjectionException &e) {
            // std::cout << "  Landmark " << symbolIndex(this->objectKey()) << "
            // received: " << e.what() << std::endl;

            // if error cannot be calculated
            // set error vector and jacobians to zero
            gtsam::Vector4 error = gtsam::Vector4::Ones() * 1000.0;
            if (H1) {
                *H1 = gtsam::Matrix::Zero(4, 6);
            }

            return error;
        }
    }



    // template<class ROBOT>
    // gtsam::Vector BboxEllipsoidFactor<ROBOT>::evaluateError(
    //     const typename Robot::Pose &conf, gtsam::OptionalMatrixType H1) const {
    //
    //     // 一, 获取相机位姿（相对于baselink），获取dx_djoint偏导
    //     std::vector<Pose3> joint_pos;
    //     std::vector<Matrix> dx_djoint;
    //     robot_.fk_model().forwardKinematics(conf, {}, joint_pos, {}, &dx_djoint);
    //     gtsam::Pose3 T_baselink_to_endlink = joint_pos[joint_pos.size()];
    //     Eigen::Matrix4d T_endlink_to_c;
    //     T_endlink_to_c <<   0, 0, 1, 0.02,
    //                         -1, 0, 0, -0.013,
    //                         0, -1, 0, 0.07, //实际为0.13(用于ros)，改为0.07(用于gpmp2 forwardKinematics)
    //                         0, 0, 0, 1;
    //     Eigen::Matrix4d T_baselink_to_camera = T_baselink_to_endlink.matrix() * T_endlink_to_c;
    //     gtsam::Pose3 camera_pose(T_baselink_to_camera);
    //
    //     // 二、 通过bbox计算Error，并获得 db_dC 和 dC_dx 偏导
    //     try {
    //         // 2.投影和几何检查：
    //         // 在投影操作之前，首先检查 quadric 是否在相机的后面（isBehind）或者相机是否在 quadric 内部（contains）。如果满足这些条件，投影操作无效，会抛出 QuadricProjectionException 异常。
    //         // check pose-quadric pair
    //         if (Quadric_.isBehind(camera_pose)) {
    //             throw QuadricProjectionException("Quadric is behind camera");
    //         }
    //         if (Quadric_.contains(camera_pose)) {
    //             throw QuadricProjectionException("Camera is inside quadric");
    //         }
    //
    //         // 3. 投影四次曲面：
    //         // project quadric taking into account partial derivatives
    //         // 使用 QuadricCamera::project 函数将双二次曲面 quadric 投影到 pose 所代表的相机坐标系中，生成一个 DualConic（双圆锥），表示在图像平面上的投影。
    //         Eigen::Matrix<double, 9, 6> dC_dx; // 二次曲面对相机位姿的偏导
    //         // 将 quadric 投影到 pose 所在的相机坐标系中，生成 DualConic。
    //         DualConic dualConic;
    //         if (!NUMERICAL_DERIVATIVE) {
    //             dualConic = QuadricCamera::project(Quadric_, camera_pose, gtsam_calibration_,
    //                                                     0,  H1 ? &dC_dx : 0);
    //         } else {
    //             dualConic = QuadricCamera::project(Quadric_, camera_pose, gtsam_calibration_);
    //         }
    //
    //         // check dual conic is valid for error function
    //         // 如果投影结果不是椭圆（例如，是双曲线或其他形状），则无法继续计算误差，抛出异常。
    //         if (!dualConic.isEllipse()) {
    //             throw QuadricProjectionException("Projected Conic is non-ellipse");
    //         }
    //
    //         // calculate conic bounds with derivatives
    //         bool computeJacobians = bool(H1) && !NUMERICAL_DERIVATIVE;
    //         // bbox对二维二次曲线的偏导
    //         Eigen::Matrix<double, 4, 9> db_dC;
    //         AlignedBox2 predictedBounds;
    //         // 图像平面上的预测边界框 predictedBounds:  通过 dualConic 计算
    //         // db_dC : bbox误差对 DualConic 的导数
    //         if (measurementModel_ == STANDARD) {
    //             predictedBounds = dualConic.bounds(computeJacobians ? &db_dC : 0);
    //         } else if (measurementModel_ == TRUNCATED) {
    //             try {
    //                 predictedBounds =
    //                         dualConic.smartBounds(gtsam_calibration_, computeJacobians ? &db_dC : 0);
    //             } catch (std::runtime_error &e) {
    //                 throw QuadricProjectionException("smartbounds failed");
    //             }
    //         }
    //
    //
    //         // 4. 计算误差：
    //         // evaluate error
    //         // 如果投影的 DualConic 不是椭圆（例如，圆锥或双曲线），则抛出异常，表示无法计算误差。
    //         // 误差 ———— 预测的边界框（predictedBounds）和实际测量的边界框（measured_）之间的差异
    //         // 通过 dualConic.bounds() 或 dualConic.smartBounds() 计算预测的边界框（predictedBounds），该边界框与实际测量值（measured_）之间的差异即为误差。
    //         gtsam::Vector4 error = predictedBounds.vector() - measured_.vector();
    //
    //
    //         // 5. 计算雅可比矩阵：
    //         if (NUMERICAL_DERIVATIVE) {
    //             std::function<gtsam::Vector(const gtsam::Pose3 &,
    //             const ConstrainedDualQuadric &)>
    //             funPtr(boost::bind(&BoundingBoxFactor::evaluateError, this,
    //                                boost::placeholders::_1, boost::placeholders::_2,
    //                                nullptr, nullptr));
    //             if (H1) {
    //                 Eigen::Matrix<double, 4, 6> db_dx_ =
    //                         gtsam::numericalDerivative21(funPtr, camera_pose, Quadric_, 1e-6);
    //                 *H1 = db_dx_;
    //             }
    //
    //         } else {
    //             // calculate derivative of error wrt pose
    //             if (H1) {
    //                 // combine partial derivatives
    //                 *H1 = db_dC * dC_dx;
    //                 std::cout << " [debug0] db_dC = " << std::endl << db_dC << std::endl;
    //                 std::cout << " [debug0] dC_dx = " << std::endl << dC_dx << std::endl;
    //             }
    //
    //             // calculate derivative of error wrt quadric
    //
    //         }
    //
    //         return error;
    //
    //         // check for nans
    //         if (    error.array().isInf().any() || error.array().isNaN().any() ||
    //             (H1 && (H1->array().isInf().any() || H1->array().isNaN().any()))    ) {
    //             throw std::runtime_error("nan/inf error in bbf");
    //         }
    //
    //         // handle projection failures
    //     } catch (QuadricProjectionException &e) {
    //         // std::cout << "  Landmark " << symbolIndex(this->objectKey()) << "
    //         // received: " << e.what() << std::endl;
    //
    //         // if error cannot be calculated
    //         // set error vector and jacobians to zero
    //         gtsam::Vector4 error = gtsam::Vector4::Ones() * 1000.0;
    //         if (H1) {
    //             *H1 = gtsam::Matrix::Zero(4, 6);
    //         }
    //
    //         return error;
    //     }
    // }
} // namespace gpmp2

#endif
