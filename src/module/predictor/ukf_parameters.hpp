#pragma once

#include <array>

#include "utility/math/kalman_filter/ukf.hpp"

namespace pingpong_tracker::predictor {

/**
 * @brief UKF 算法参数配置结构体
 * 用于存储无迹卡尔曼滤波器的超参数、噪声系数及初始状态不确定性。
 */
struct UKFParameters {
    // 状态维度: 9 [x, y, z, vx, vy, vz, wx, wy, wz]
    // 观测维度: 3 [x, y, z]
    static constexpr int StateDim = 9;
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
     * 描述系统模型的不确定性。
     */
    // 增加角速度噪声 [wx, wy, wz]，通常设为较大值以允许滤波器快速适应旋转变化
    std::array<double, StateDim> process_noise_std = {0.01, 0.01, 0.01, 0.1, 0.1,
                                                      0.1,  5.0,  5.0,  5.0};

    /**
     * @brief 观测噪声标准差 (Measurement Noise Std Dev) -> R 矩阵
     * 描述传感器的测量误差。
     */
    std::array<double, ObsDim> measurement_noise_std = {0.005, 0.005, 0.005};

    /**
     * @brief 初始协方差对角元素 (Initial P Diagonal)
     * 描述系统启动时对初始状态估计的不确定度。
     */
    // 初始旋转未知，给予较大的不确定度 (例如 100 rad/s 的方差)
    std::array<double, StateDim> initial_covariance = {0.01, 0.01,  0.01,  1.0,  1.0,
                                                       1.0,  100.0, 100.0, 100.0};
};

}  // namespace pingpong_tracker::predictor
