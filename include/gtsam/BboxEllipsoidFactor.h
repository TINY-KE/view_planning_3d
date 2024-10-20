/**
 *  @file   BboxPlaneEllipsoidFactor.h
 *  @brief  Gaussian prior defined on the workspace pose of any joint of a robot
 *          given its state in configuration space
 *  @author zhang jiadong
 *  @date   Jan 8, 2018
 **/
#ifndef BboxPlaneEllipsoidFactor_H
#define BboxPlaneEllipsoidFactor_H

#pragma once

#include <gpmp2/kinematics/RobotModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include  "core/Plane.h"
#include "MapObject.h"

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



namespace gpmp2 {
    /**
     * Gaussian prior defined on the workspace pose
     */
    template <class ROBOT>
    class BboxPlaneEllipsoidFactor
        : public gtsam::NoiseModelFactorN<typename ROBOT::Pose> {

    public:
        // typedefs
        typedef ROBOT Robot;
        typedef typename Robot::Pose Pose;

    private:
        // typedefs
        typedef BboxPlaneEllipsoidFactor This;
        typedef gtsam::NoiseModelFactorN<Pose> Base;

        const Robot& robot_;     // Robot
        int joint_;              // joint on the robot to be constrained

//        boost::shared_ptr<gtsam::Cal3_S2> gtsam_calib = boost::make_shared<gtsam::Cal3_S2>(fx, fy, 0.0, cx, cy);

        // 相机坐标系中的四个平面
        std::vector<g2o::plane*> mPlanes;   // 顺序： 左上右小

        // 目标物体。物体的位姿实在world坐标系中
        MapObject* object;
        g2o::ellipsoid* mEllipsoid_;

        //机器人在world中的目标位姿
        Eigen::Matrix4f mRobotPose;


    public:
        /* Default constructor */
        BboxPlaneEllipsoidFactor() {}

        /// Constructor
        BboxPlaneEllipsoidFactor(gtsam::Key poseKey, const Robot& robot,
                        double cost_sigma,
                        MapObject* ob_,
                        Eigen::Matrix4f mRobotPose_,
                        int width, int height, Matrix3d calib)
          : Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(),
                                                     cost_sigma),
                 poseKey),
                robot_(robot),
                object(ob_),
                mRobotPose(mRobotPose_),
                miImageCols(width),
                miImageRows(height),
                mCalib(calib)
        {
            GenerateBboxPlanes_g2o();
            GenerateEllipsoid(ob_);
        }

        virtual ~BboxPlaneEllipsoidFactor() {}


        /// factor error function
        /// 输入参数： pose —— robot pose in config space
        /// numerical jacobians / analytic jacobians from cost function
        // zhjd: 此处的Robot::Pose对应的是gtsam::Vector。   根据 ForwardKinematics<gtsam::Vector, gtsam::Vector>。 因此这里的pose就是关节角度。
        gtsam::Vector evaluateError(
            const typename Robot::Pose& conf,
            gtsam::OptionalMatrixType H1 = nullptr) const override;

        std::vector<g2o::plane*>  computeplanes(
               const typename Robot::Pose& conf, bool output = false);

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return std::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        /** print contents */
        void print(const std::string& s = "",
                 const gtsam::KeyFormatter& keyFormatter =
                     gtsam::DefaultKeyFormatter) const {
            std::cout << s << "BboxPlaneEllipsoidFactor :" << std::endl;
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

    private:
        Eigen::Matrix3Xd generateProjectionMatrix();
        Eigen::MatrixXd fromDetectionsToLines(Vector4d &detections);
        Eigen::MatrixXd GenerateBboxPlanes(Eigen::Vector4d &bbox);
        void GenerateBboxPlanes_g2o();   //计算在相机坐标系中的bboxPlanes

    public:
        void GenerateEllipsoid(MapObject* object) {

            // 一、将物体的旋转矩阵，转为四元数
            auto matrix =  object->mCuboid3D.pose_mat;
            Eigen::Isometry3d Iso=Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵

            //----对各个元素赋值----
            Iso(0, 0) = matrix.at<float>(0,0), Iso(0, 1) = matrix.at<float>(0, 1), Iso(0, 2) = matrix.at<float>(0, 2), Iso(0, 3) = matrix.at<float>(0, 3);
            Iso(1, 0) = matrix.at<float>(1, 0), Iso(1, 1) = matrix.at<float>(1, 1), Iso(1, 2) = matrix.at<float>(1, 2), Iso(1, 3) = matrix.at<float>(1, 3);
            Iso(2, 0) = matrix.at<float>(2, 0), Iso(2, 1) = matrix.at<float>(2, 1), Iso(2, 2) = matrix.at<float>(2, 2), Iso(2, 3) = matrix.at<float>(2, 3);
            Iso(3, 0) = matrix.at<float>(3, 0), Iso(3, 1) = matrix.at<float>(3, 1), Iso(3, 2) = matrix.at<float>(3, 2), Iso(3, 3) = matrix.at<float>(3, 3);

            Eigen::Quaterniond q = Eigen::Quaterniond(Iso.rotation());

            // 二、转为椭球体
            mEllipsoid_ = new g2o::ellipsoid();
            Eigen::Matrix<double, 10, 1>  e_param;
            e_param <<     object->mCuboid3D.cuboidCenter.x(), object->mCuboid3D.cuboidCenter.y(), object->mCuboid3D.cuboidCenter.z(),   // x y z
                            q.x(), q.y(), q.z(), q.w(),  // qx qy qz qw
                            object->mCuboid3D.lenth, object->mCuboid3D.width, object->mCuboid3D.height   // length_a  length_b  length_c
                            ;
            // std::cout << "e_param: 11:" << e_param.transpose() << std::endl;
            mEllipsoid_->fromVector(e_param);
            mEllipsoid_->prob = 1.0;
            mEllipsoid_->setColor(Vector3d(1,0,0.1));
        }
    };
}



//
// double hingeLossFovCost(
//         const gtsam::Point3& point, g2o::plane* plane_low, double eps,
//         gtsam::OptionalJacobian<1, 3> H_point = nullptr) {
//
//     gtsam::Vector3 field_gradient;  //距离对Point3的导数，
//     double dist_signed;   //Point3与最低视场平面之间的最近距离
//
//     //（注意平面的normal是向上，因此带入平面方程后的距离结果要取负数）
//     dist_signed = -1 * plane_low->distanceToPoint(point, true);
//
//     if (dist_signed > eps) {
//         // faraway no error
//         if (H_point) *H_point = gtsam::Matrix13::Zero();
//         return 0.0;
//     } else {
//         // outside but < eps or inside object
//         if (H_point) {
//             // ERROR增加的方向是靠近障碍物的方向，与平面的normal方向相同。
//             field_gradient = plane_low->normal();
//             field_gradient.normalize();
//             *H_point = field_gradient.transpose();
//         }
//         return eps - dist_signed;
//     }
// }

double GetNearestAndFarthestPointOnEllipsoidToPlane(g2o::plane &pl, g2o::ellipsoid &e,
                                                  Eigen::Vector3d &nearestP, double &nearestDis, Eigen::Vector3d &farthestP, double &farthestDis,
                                                  gtsam::OptionalJacobian<1, 3> H_point = nullptr) {

    // 1. 坐标系转换：将平面 pl 也转换到椭球体的本地坐标系 ple，
    g2o::SE3Quat Tew = e.pose.inverse();

    g2o::ellipsoid e_e = e.transform_from(Tew);
    g2o::plane ple = pl;
    ple.transform(Tew);

    // 2. 提取平面方程：
    // AX+BY+CZ+D = 0
    Eigen::Vector4d plane_e = ple.param;
    double A = plane_e[0];
    double B = plane_e[1];
    double C = plane_e[2];
    double D = plane_e[3];

    // 3. 世界坐标系下的椭球体二次曲面方程：
    // 获得椭球体参数向量形式
    // ax^2+bxy+c^..... = 0;
    Eigen::Matrix4d Q_star = e_e.generateQuadric();
    Eigen::Matrix4d Q = Q_star.inverse();    // 系数矩阵

    // 获得系数
    // ax^2+by^2+cz^2+dxy+eyz+fxz+gx+hy+iz+j=0
    // 正椭球体 check
    // x^2/a^2 + y^2/b^2 + z^2/c^2 = 1
    Q = Q / (-Q(3, 3));

    // 等价于求解后面拉格朗日方程的极值： f(x,y,z) = AX+BY+CZ+D - alpha_pos(x^2/a^2 + y^2/b^2 + z^2/c^2 - 1)

    // 对角线前3个 必须大于0, 其他项必须为0
    // std::cout << "Origin Q check : " << std::endl << Q << std::endl;

    // 4. 椭球体参数的提取：
    double a_2 = 1 / Q(0, 0);
    double b_2 = 1 / Q(1, 1);
    double c_2 = 1 / Q(2, 2);

    // 5. 拉格朗日乘数法求解极值：
    // 目标函数:  f(x,y,z)=AX+BY+CZ+D-alpha_pos(x^2/a^2 + y^2/b^2 + z^2/c^2 - 1)

    // 求解拉格朗日乘数 alpha_pos，表示在椭球表面上满足极值条件的点的拉伸因子
    double alpha_2 = 4 / (A * A * a_2 + B * B * b_2 + C * C * c_2);
    double alpha_pos = sqrt(alpha_2);

    // 求解极值点：
    double x_coeff = A * a_2 / 2;
    double y_coeff = B * b_2 / 2;
    double z_coeff = C * c_2 / 2;
    Eigen::Vector3d coeff_vec(x_coeff, y_coeff, z_coeff);
    // 通过计算出的 alpha_pos 和平面系数，得到两个极值点 extrema_1 和 extrema_2，这两个点分别是椭球体上到平面最近和最远的点。
    Eigen::Vector3d extrema_1 = alpha_pos * coeff_vec;
    Eigen::Vector3d extrema_2 = -extrema_1;

    // 6.计算平面到这些点的距离：
    double dis1 = ple.distanceToPoint(extrema_1);
    double dis2 = ple.distanceToPoint(extrema_2);

    // 7.通过比较 dis1 和 dis2 的绝对值，判断哪个是最近点，哪个是最远点：
    if (std::abs(dis1) > std::abs(dis2)) {
        nearestDis = dis2;
        nearestP = extrema_2;

        farthestDis = dis1;
        farthestP = extrema_1;
    } else {
        nearestDis = dis1;
        nearestP = extrema_1;

        farthestDis = dis2;
        farthestP = extrema_2;
    }

    // std::cout << "nearestP : " << nearestP.transpose() << std::endl;
    // std::cout << "nearestDis : " << nearestDis << std::endl;
    // std::cout << "farthestP : " << farthestP.transpose() << std::endl;
    // std::cout << "farthestDis : " << farthestDis << std::endl;

    return nearestDis;
}






#include <gpmp2/obstacle/ObstacleCost.h>
#include <vector>
using namespace std;
using namespace gtsam;

namespace gpmp2 {

 /* ************************************************************************** */

 template <class ROBOT>
std::vector< g2o::plane*>  BboxPlaneEllipsoidFactor<ROBOT>::computeplanes(
     const typename Robot::Pose& conf, bool output)  {

      // 更改平面的位置
      using namespace gtsam;
      // 相机相对于endlink的位姿
      Eigen::Matrix4f T_endlink_to_c;
      T_endlink_to_c << 0, 0, 1, 0.02,
                        -1, 0, 0, -0.013,
                        0, -1, 0, 0.07,  //实际为0.13，改为0.07
                        0, 0, 0, 1;
      // 机械臂endlink的位姿
      std::vector<Pose3> joint_pos;   //  link poses in 3D work space
      std::vector<Matrix> J_jpx_jp;   //  et al. optional Jacobians
      robot_.fk_model().forwardKinematics(conf, {}, joint_pos);
      Pose3 pose_end_link = joint_pos[joint_pos.size()-1];
      if(output)
          pose_end_link.print("[zhjd-debug] pose_end_link: \n");
      // 将 gtsam::Pose3 转换为 Eigen::Matrix4f
      Eigen::Matrix4f T_baselink_endlink = Eigen::Matrix4f::Identity();  // 创建 4x4 单位矩阵
      // 获取 gtsam::Pose3 的 3x3 旋转矩阵并赋值到 eigenMatrix 的左上角
      T_baselink_endlink.block<3, 3>(0, 0) = pose_end_link.rotation().matrix().cast<float>();
      // 获取 gtsam::Pose3 的 3x1 平移向量并赋值到 eigenMatrix 的右侧
      T_baselink_endlink.block<3, 1>(0, 3) << pose_end_link.x(), pose_end_link.y(), pose_end_link.z();
      if(output)
          std::cout<<"[zhjd-debug] T_baselink_endlink: "<<std::endl<<T_baselink_endlink<<std::endl;

      Eigen::Matrix4f T_world_2_c =  mRobotPose * T_baselink_endlink * T_endlink_to_c;
      g2o::SE3Quat T_baselink_2_c_g2o = g2o2::toSE3Quat(T_world_2_c);
      if(output)
          std::cout<<"[zhjd-debug] T_baselink_2_c_g2o: "<<std::endl<<T_baselink_2_c_g2o.to_homogeneous_matrix()<<std::endl;

      std::vector<g2o::plane*> PlanesInWorld;
      // 将平面变到baselink坐标系
      for(auto plane:mPlanes) {
          g2o::plane* pl_in_world = new g2o::plane(*plane);
          pl_in_world->transform(T_baselink_2_c_g2o);
          PlanesInWorld.push_back(pl_in_world);
      }

      return PlanesInWorld;
 }




 template <class ROBOT>
 gtsam::Vector BboxPlaneEllipsoidFactor<ROBOT>::evaluateError(
        const typename Robot::Pose& conf, gtsam::OptionalMatrixType H1) const {
      
      // 一、获取世界坐标系下的视场平面
      // 更改平面的位置
      using namespace gtsam;
      // 相机相对于endlink的位姿
      Eigen::Matrix4f T_endlink_to_c;
      T_endlink_to_c << 0, 0, 1, 0.02,
                        -1, 0, 0, -0.013,
                        0, -1, 0, 0.07,  //实际为0.13，改为0.07
                        0, 0, 0, 1;
      // 机械臂endlink的位姿
      std::vector<Pose3> joint_pos;   //  link poses in 3D work space
      std::vector<Matrix> J_jpx_jp;   //  et al. optional Jacobians  pose3对config的偏导
      robot_.fk_model().forwardKinematics(conf, {}, joint_pos);
      Pose3 pose_end_link = joint_pos[joint_pos.size()-1];

      // 将 gtsam::Pose3 转换为 Eigen::Matrix4f
      Eigen::Matrix4f T_baselink_endlink = Eigen::Matrix4f::Identity();  // 创建 4x4 单位矩阵
      // 获取 gtsam::Pose3 的 3x3 旋转矩阵并赋值到 eigenMatrix 的左上角
      T_baselink_endlink.block<3, 3>(0, 0) = pose_end_link.rotation().matrix().cast<float>();
      // 获取 gtsam::Pose3 的 3x1 平移向量并赋值到 eigenMatrix 的右侧
      T_baselink_endlink.block<3, 1>(0, 3) << pose_end_link.x(), pose_end_link.y(), pose_end_link.z();

      Eigen::Matrix4f T_world_2_c =  mRobotPose * T_baselink_endlink * T_endlink_to_c;
      g2o::SE3Quat T_baselink_2_c_g2o = g2o2::toSE3Quat(T_world_2_c);

      std::vector<g2o::plane*> PlanesInWorld;
      // 将平面变到baselink坐标系
      for(auto plane:mPlanes) {
          g2o::plane* pl_in_world = new g2o::plane(*plane);
          pl_in_world->transform(T_baselink_2_c_g2o);
          PlanesInWorld.push_back(pl_in_world);
      }




      // 二、

      // if Jacobians used, initialize as zeros
      if (H1) *H1 = Matrix::Zero(4, robot_.dof());

      // allocate cost vector  顺序是左上右下
      Vector err(4);
      
      // for each point on arm stick, get error
      for (int pl_idx = 0; pl_idx < 4; pl_idx++) {

            if (H1) {
                // Matrix13 Jerr_point;
                // err(pl_idx) = hingeLossFovCost(sph_centers[pl_idx], pl_in_baselink,
                //                                      total_eps, Jerr_point);
                //
                // // chain rules
                // H1->row(pl_idx) = Jerr_point * J_px_jp[pl_idx];

            }
            else {
                // 每一行err对应物体和平面的距离。  相切的时候，距离为0，也即error为0.
                double nD, fD;     //距离
                Vector3d nP, fP;   //相切点
                err(pl_idx) = GetNearestAndFarthestPointOnEllipsoidToPlane( *PlanesInWorld[pl_idx], *mEllipsoid_, nP, nD, fP, fD);
            }
      }

      return err;
 }









 template <class ROBOT>
Eigen::Matrix3Xd BboxPlaneEllipsoidFactor<ROBOT>::generateProjectionMatrix() {
    Eigen::Matrix3Xd identity_lefttop;
    identity_lefttop.resize(3, 4);
    identity_lefttop.col(3) = Vector3d(0, 0, 0);
    identity_lefttop.topLeftCorner<3, 3>() = Matrix3d::Identity(3, 3);

    Eigen::Matrix3Xd proj_mat = mCalib * identity_lefttop;

    g2o::SE3Quat campose_wc = g2o::SE3Quat();
    g2o::SE3Quat campose_cw = campose_wc.inverse();
    proj_mat = proj_mat * campose_cw.to_homogeneous_matrix();

    return proj_mat;
}

 template <class ROBOT>
Eigen::MatrixXd BboxPlaneEllipsoidFactor<ROBOT>::fromDetectionsToLines(Vector4d &detections) {
    bool flag_openFilter = false; // filter those lines lying on the image boundary

     double x1 = detections(0);
     double y1 = detections(1);
     double x2 = detections(2);
     double y2 = detections(3);

     //左
     Eigen::Vector3d line1(1, 0, -x1);
     //上
     Eigen::Vector3d line2(0, 1, -y1);
     //右
     Eigen::Vector3d line3(1, 0, -x2);
    // line4表示一条底部水平线的平面方程，其方程为： y = y2
    // 通过向量形式表示为： 0*x + 1*y - y2 = 0
    Eigen::Vector3d line4(0, 1, -y2);

    // those lying on the image boundary have been marked -1
    Eigen::MatrixXd line_selected(3, 0);
    Eigen::MatrixXd line_selected_none(3, 0);

    int config_border_pixel = 10;
     if (!flag_openFilter || (x1 > config_border_pixel && x1 < miImageCols - config_border_pixel)) {
         line_selected.conservativeResize(3, line_selected.cols() + 1);
         line_selected.col(line_selected.cols() - 1) = line1;
     }
     if (!flag_openFilter || (y1 > config_border_pixel && y1 < miImageRows - config_border_pixel)) {
         line_selected.conservativeResize(3, line_selected.cols() + 1);
         line_selected.col(line_selected.cols() - 1) = line2;
     }
     if (!flag_openFilter || (x2 > config_border_pixel && x2 < miImageCols - config_border_pixel)) {
         line_selected.conservativeResize(3, line_selected.cols() + 1);
         line_selected.col(line_selected.cols() - 1) = line3;
     }
     if (!flag_openFilter || (y2 > config_border_pixel && y2 < miImageRows - config_border_pixel)) {
        // 将其列数增加 1。
        line_selected.conservativeResize(3, line_selected.cols() + 1);
        // 将向量 line4 赋值给矩阵 line_selected 的最后一列。
        line_selected.col(line_selected.cols() - 1) = line4;
     }

    return line_selected;
}

 template <class ROBOT>
Eigen::MatrixXd BboxPlaneEllipsoidFactor<ROBOT>::GenerateBboxPlanes(Eigen::Vector4d &bbox) {
        Eigen::MatrixXd planes_all(4, 0);
        // std::cout << " [debug] calib : \n " << calib << std::endl;
        // get projection matrix

        Eigen::MatrixXd P = generateProjectionMatrix();

        Eigen::MatrixXd lines = fromDetectionsToLines(bbox);
        Eigen::MatrixXd planes = P.transpose() * lines;

        // 相机z轴方向，各个平面的normal应该与camera_direction 夹角小于90度
        Eigen::Vector3d camera_direction(0,0,1);  // Z轴方向

        // add to matrix
        for (int m = 0; m < planes.cols(); m++) {
            // 获取平面的法向量 (a, b, c)
            Eigen::Vector3d normal = planes.block<3, 1>(0, m);

            // 检查法向量与z轴朝向相同，如果不是则反转法向量
            if (normal.dot(camera_direction) < 0) {
                // 反转法向量方向
                planes.col(m) = -planes.col(m);
            }

            planes_all.conservativeResize(planes_all.rows(), planes_all.cols() + 1);
            planes_all.col(planes_all.cols() - 1) = planes.col(m);
        }

        return planes_all;
    }


template <class ROBOT>
void BboxPlaneEllipsoidFactor<ROBOT>::GenerateBboxPlanes_g2o() {
//      根据椭球体，投影到相机平面，获得bbox
     Eigen::Vector4d bbox(10, 10, miImageCols-10, miImageRows-10);

      Eigen::MatrixXd mPlanesParamLocal_Col = GenerateBboxPlanes(bbox);  // attention: store as 列
      Eigen::MatrixXd mPlanesParamLocal = mPlanesParamLocal_Col.transpose();
//      mPlane = new g2o::plane(vec.head(4));

     int num = mPlanesParamLocal.rows();

     for( int i=0;i<num;i++)
     {
         Eigen::VectorXd vec = mPlanesParamLocal.row(i);
         g2o::plane* plane_new= new g2o::plane(vec.head(4));
         mPlanes.push_back(plane_new);
     }
}




}  // namespace gpmp2

#endif