#pragma once

#include <array>

#include "utility/math/kalman_filter/ukf.hpp"

namespace pingpong_tracker::predictor {

/**
 * @brief UKF 算法参数配置结构体
 * 用于存储无迹卡尔曼滤波器的超参数、噪声系数及初始状态不确定性。
 */
struct UKFParameters {
    // 状态维度: 6 [x, y, z, vx, vy, vz]
    // 观测维度: 3 [x, y, z]
    static constexpr int StateDim = 6;
    static constexpr int ObsDim   = 3;

    using UKF = util::UKF<StateDim, ObsDim>;

    /**
     * @brief 散布参数 alpha (0 < alpha <= 1)
     * 控制 Sigma 点相对于均值的分布范围。
     * 较小的值 (如 1e-3) 适用于强非线性系统，以避免采样点跨越非线性极剧烈的区域。
     */
    double alpha = 0.1;

    /**
     * @brief 分布先验参数 beta
     * 用于整合状态分布的高阶矩信息。
     * 对于高斯分布噪声，beta = 2.0 是最优选择。
     */
    double beta = 2.0;

    /**
     * @brief 辅助缩放参数 kappa
     * 通常设置为 0 或 3 - L (L为状态维度)。
     * 用于微调方差的精度。
     */
    double kappa = 0.0;

    /**
     * @brief 过程噪声标准差 (Process Noise Std Dev) -> Q 矩阵
     * 描述系统模型的不确定性（如空气阻力系数波动、风、模型简化误差）。
     * 维度应与状态向量维度一致 (StateDim)。
     * 典型值: 位置误差较小，速度误差较大。
     */
    std::array<double, StateDim> process_noise_std = {0.01, 0.01, 0.01, 0.1, 0.1, 0.1};

    /**
     * @brief 观测噪声标准差 (Measurement Noise Std Dev) -> R 矩阵
     * 描述传感器的测量误差（如相机标定误差、像素噪声）。
     * 维度应与观测向量维度一致 (ObsDim)。
     */
    std::array<double, ObsDim> measurement_noise_std = {0.005, 0.005, 0.005};

    /**
     * @brief 初始协方差对角元素 (Initial P Diagonal)
     * 描述系统启动时对初始状态估计的不确定度。
     */
    std::array<double, StateDim> initial_covariance = {0.01, 0.01, 0.01, 1.0, 1.0, 1.0};

    /**
     * @brief 重力加速度 (m/s^2)
     * 垂直向下，通常取 9.81。
     */
    double gravity = 9.81;

    /**
     * @brief 空气阻力系数 (Drag Coefficient, Cd)
     * 乒乓球通常在 0.4 到 0.5 之间。
     * 阻力公式: F_d = -0.5 * rho * Cd * A * v * |v|
     */
    double drag_coefficient = 0.47;

    /**
     * @brief 球体质量 (kg)
     * 标准乒乓球质量约为 2.7g (0.0027kg)。
     */
    double mass = 0.0027;

    /**
     * @brief 球体半径 (m)
     * 标准乒乓球直径 40mm，半径 0.02m。
     */
    double radius = 0.02;

    /**
     * @brief 空气密度 (kg/m^3)
     * 标准大气压下约为 1.225 kg/m^3。
     */
    double air_density = 1.225;
};

}  // namespace pingpong_tracker::predictor
