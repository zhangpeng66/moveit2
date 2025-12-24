#ifndef POSE_TRANS_HPP
#define POSE_TRANS_HPP

#include <vector>
#include <iostream>
#include <Eigen/Dense>

class PoseTrans
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    template <typename T>
    static int sgn(T val)
    {
        if (val > 0)
        {
            return 1;
        } else
        {   
            return -1;
        }
    }

    static void poseVecMul(const std::vector<double>& pose_vec0, const std::vector<double>& pose_vec1, std::vector<double>& pose_vec_res);
    static void poseVecInverse(const std::vector<double>& pose_vec, std::vector<double>& pose_vec_inv);

    static void transTcpToBase(
        const std::vector<double>& vec_tcp_move, 
        const std::vector<double>& vec_base_tcp,
        std::vector<double>& vec_base_tar);

    static void computeDeltaMoveFromBaseTcp(
        const std::vector<double>& vec_base_tcp0, 
        const std::vector<double>& vec_base_tcp1,
        std::vector<double>& vec_delta_move
        );
public:
    static bool isRotationMatrix(const Eigen::Matrix3d& R);

    static void eulerAngleToMat3d(const std::vector<double>& euler_ang, Eigen::Matrix3d& R);
    static void eulerAngleToMat3d(const Eigen::Vector3d& euler_ang, Eigen::Matrix3d& R);
    static void mat3dToEulerAngles(const Eigen::Matrix3d& R, std::vector<double>& euler_angle);
    static void mat3dToEulerAngles(const Eigen::Matrix3d& R, Eigen::Vector3d& euler_angle);
};

#endif