/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BoundingBoxFactor.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief factor between Pose3 and ConstrainedDualQuadric
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_quadrics/geometry/AlignedBox2.h>
#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/bind/bind.hpp>

#include "MapObject.h"
#include "Converter.h"
#include <opencv2/opencv.hpp>


namespace gtsam_quadrics {
    /**
     * @class BoundingBoxFactor
     * AlignedBox3 factor between Pose3 and ConstrainedDualQuadric
     * Projects the quadric at the current pose estimates,
     * Calculates the bounds of the dual conic,
     * and compares this to the measured bounding box.
     */
    class HeightCameraFactor
            : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
    public:
        enum MeasurementModel {
            STANDARD,
            TRUNCATED
        }; ///< enum to declare which error function to use

    protected:
        AlignedBox2 measured_; ///< measured bounding box
        typedef NoiseModelFactorN<gtsam::Pose3> Base; ///< base class has keys and noisemodel as private members
        MeasurementModel measurementModel_;

        int miImageCols, miImageRows;

        boost::shared_ptr<gtsam::Cal3_S2> gtsam_calibration_;

        // 目标物体。物体的位姿实在world坐标系中
        ConstrainedDualQuadric Quadric_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// @name Constructors and named constructors
        /// @{

        /** Default constructor */
        HeightCameraFactor()
            : measured_(0., 0., 0., 0.), measurementModel_(STANDARD) {
        };

        /** Constructor from measured box, calbration, dimensions and posekey,
         * quadrickey, noisemodel */
        HeightCameraFactor(gtsam::Key poseKey,
                         double cost_sigma,
                         const AlignedBox2 &measured,
                         MapObject *ob_,
                         Eigen::Matrix4f mRobotPose_,
                         int width, int height, Eigen::Matrix3d calib)
            : Base(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), poseKey),
              measured_(measured),
              miImageCols(width),
              miImageRows(height)
        {
            // 生成世界坐标系下的目标物体
            GenerateConstrainedDualQuadric(ob_, mRobotPose_);
            // 生成 内参
            GenerateCalibration(calib);
            // 生成 目标bbox
            // GenerateMeasureBbox(measured);
            //
            measurementModel_ = STANDARD;
        };


        /// @}
        /// @name Class accessors
        /// @{

        /** Returns the measured bounding box */
        AlignedBox2 measurement() const { return AlignedBox2(measured_.vector()); }

        /** Returns the pose key */
        gtsam::Key poseKey() const { return key1(); }

        /// @}
        /// @name Class methods
        /// @{

        /**
         * Evaluate the error between a quadric and 3D pose
         * @param pose the 6DOF camera position
         * @param quadric the constrained dual quadric
         * @param H1 the derivative of the error wrt camera pose (4x6)
         * @param H2 the derivative of the error wrt quadric (4x9)
         */
        gtsam::Vector evaluateError(
            const gtsam::Pose3 &pose,
            gtsam::OptionalMatrixType H1 = nullptr) const;


        /// @}
        /// @name Testable group traits
        /// @{

        /** Prints the boundingbox factor with optional string */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter =
                           gtsam::DefaultKeyFormatter) const override;

        /** Returns true if equal keys, measurement, noisemodel and calibration */
        bool equals(const HeightCameraFactor &other, double tol = 1e-9) const;

    private:
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

        void GenerateCalibration(Eigen::Matrix3d &calib) {
            //            Eigen::Matrix3d Calib;
            //            Calib << fx,  0,  cx,
            //                      0,  fy,  cy,
            //                      0,   0,   1;
            //            boost::shared_ptr<gtsam::Cal3_S2> gtsam_calib = boost::make_shared<gtsam::Cal3_S2>(fx, fy, 0.0, cx, cy);
            gtsam_calibration_ = boost::make_shared<gtsam::Cal3_S2>(calib(0, 0), calib(1, 1), 0.0, calib(0, 2),
                                                                    calib(1, 2));
        }

        gtsam_quadrics::AlignedBox2 AdjustingBBox(const AlignedBox2& predicted_bbox, const AlignedBox2& measured_bbox) const{
            // gtsam_quadrics::AlignedBox2 gtsam_bbox(0+s, 0+s, CameraWidth-s, CameraHeight-s);   //预期的物体检测框
            // measured_ = measured;
            double predicted_width = predicted_bbox.xmax() - predicted_bbox.xmin();
            double predicted_height = predicted_bbox.ymax() - predicted_bbox.ymin();

            double measured_width = measured_bbox.xmax() - measured_bbox.xmin();
            double measured_height = measured_bbox.ymax() - measured_bbox.ymin();

            if( predicted_width/predicted_height > measured_width/measured_height) {

                double new_height = measured_width * predicted_height / predicted_width;
                double new_ymin = measured_bbox.ymin() + (measured_height - new_height) / 2;
                double new_ymax = new_ymin + new_height;
                gtsam_quadrics::AlignedBox2 new_measured_bbox (measured_bbox.xmin(), new_ymin, measured_bbox.xmax(), new_ymax);
                std::cout<<"[debug] new_measured_bbox: "<<new_measured_bbox.vector().transpose()<<std::endl;
                return new_measured_bbox;
            } else {
                double new_width = measured_height * predicted_width / predicted_height;
                double new_xmin = measured_bbox.xmin() + (measured_width - new_width) / 2;
                double new_xmax = new_xmin + new_width;
                gtsam_quadrics::AlignedBox2 new_measured_bbox (new_xmin, measured_bbox.ymin(), new_xmax, measured_bbox.ymax());
                std::cout<<"[debug] new_measured_bbox: "<<new_measured_bbox.vector().transpose()<<std::endl;
                return new_measured_bbox;
            }

        }




    };
} // namespace gtsam_quadrics

/** \cond PRIVATE */
// Add to testable group
// template <>
// struct gtsam::traits<gtsam_quadrics::HeightCameraFactor>
//     : public gtsam::Testable<gtsam_quadrics::HeightCameraFactor> {};
/** \endcond */
