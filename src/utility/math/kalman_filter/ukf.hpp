#pragma once

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Core>

#include "ekf.hpp"

namespace pingpong_tracker::util {

template <int StateDim, int ObsDim>
class UKF {
public:
    using XVec         = Eigen::Matrix<double, StateDim, 1>;
    using ZVec         = Eigen::Matrix<double, ObsDim, 1>;
    using PMat         = Eigen::Matrix<double, StateDim, StateDim>;
    using RMat         = Eigen::Matrix<double, ObsDim, ObsDim>;
    using QMat         = Eigen::Matrix<double, StateDim, StateDim>;
    using SigmaPoints  = Eigen::Matrix<double, StateDim, 2 * StateDim + 1>;
    using ZSigmaPoints = Eigen::Matrix<double, ObsDim, 2 * StateDim + 1>;
    using WeightVec    = Eigen::Matrix<double, 2 * StateDim + 1, 1>;

    XVec x;

    /**
     * @brief 构造函数
     * @param alpha 散布参数 (Spread): 控制 Sigma 点相对于均值的散布程度。
     *              通常设为小正数 (e.g., 1e-3) 以避免非局部效应。
     * @param beta  分布先验 (Prior): 用于结合状态分布的先验知识。
     *              对于高斯分布，beta=2 是最优的，可以最小化四阶矩误差。
     * @param kappa 辅助缩放 (Scaling): 二次缩放参数，通常设为 0 或 3-n。
     *              用于微调高阶矩的近似精度。
     */
    explicit UKF(double alpha = 1e-3, double beta = 2.0, double kappa = 0.0)
        : x{XVec::Zero()}, P_{PMat::Identity()}, alpha_(alpha), beta_(beta), kappa_(kappa) {
        init_weights();
    }

    /**
     * @brief 构造函数 (带初始状态)
     */
    explicit UKF(XVec const& initial_x, PMat const& initial_P, double alpha = 1e-3,
                 double beta = 2.0, double kappa = 0.0)
        : x(initial_x), P_(initial_P), alpha_(alpha), beta_(beta), kappa_(kappa) {
        init_weights();
    }

    /**
     * @brief 预测步
     * @param f 状态转移 Lambda: (XVec) -> XVec
     * @param Q 过程噪声
     *
     * 算法流程:
     * 1. 生成 Sigma Points (基于上一时刻后验):
     *    X_{k-1} = [x, x + γ√P, x - γ√P], 其中 γ = sqrt(n + λ)
     * 2. 传播 Sigma Points (通过非线性函数 f):
     *    X_{k|k-1}^* = f(X_{k-1})
     * 3. 计算预测均值 (加权求和):
     *    x_{k|k-1} = Σ (wm * X_{k|k-1}^*)
     * 4. 计算预测协方差:
     *    P_{k|k-1} = Σ (wc * (X^* - x)(X^* - x)^T) + Q
     */
    template <typename ModelFunc>
    auto predict(ModelFunc&& f, QMat const& Q) -> void {
        static_assert(StateTransitionModel<ModelFunc, StateDim>,
                      "\n[UKF Error] 状态转移函数 f(x) 签名错误！\n"
                      "预期格式: XVec f(const XVec&)\n");

        // 1. 生成 Sigma Points
        auto X_sig = generate_sigma_points(x, P_);

        // 2. 预测 Sigma Points
        SigmaPoints X_pred_sig;
        for (int i = 0; i < 2 * StateDim + 1; ++i) {
            X_pred_sig.col(i) = f(X_sig.col(i));
        }

        // 3. 计算预测均值
        x = X_pred_sig * wm_;

        // 4. 计算预测协方差
        P_.setZero();
        for (int i = 0; i < 2 * StateDim + 1; ++i) {
            XVec diff = X_pred_sig.col(i) - x;
            P_ += wc_(i) * diff * diff.transpose();
        }
        P_ += Q;
    }

    /**
     * @brief 更新步
     * @param z 观测值
     * @param h 观测 Lambda: (XVec) -> ZVec
     * @param R 观测噪声
     * @param x_add_op 状态更新算子 ⊕
     * @param z_sub_op 残差计算算子 ⊖
     *
     * 算法流程:
     * 1. 重采样 Sigma Points (基于预测分布):
     *    X_{k|k-1} = generate_sigma_points(x_{k|k-1}, P_{k|k-1})
     * 2. 预测观测 Sigma Points (通过非线性函数 h):
     *    Z_{k}^* = h(X_{k|k-1})
     * 3. 计算观测均值:
     *    z_pred = Σ (wm * Z_{k}^*)
     * 4. 计算观测协方差 S 和 互协方差 Pxz:
     *    S = Σ (wc * (Z^* - z_pred)(Z^* - z_pred)^T) + R
     *    Pxz = Σ (wc * (X - x)(Z^* - z_pred)^T)
     * 5. 计算卡尔曼增益:
     *    K = Pxz * S^-1
     * 6. 更新状态和协方差:
     *    x_{k|k} = x_{k|k-1} + K * (z - z_pred)
     *    P_{k|k} = P_{k|k-1} - K * S * K^T
     */
    template <typename MeasFunc, typename AddOp = decltype(DefaultAdd),
              typename SubOp = decltype(DefaultSubtract)>
    auto update(ZVec const& z, MeasFunc&& h, RMat const& R, AddOp&& x_add_op = DefaultAdd,
                SubOp&& z_sub_op = DefaultSubtract) -> bool {
        static_assert(ObservationModel<MeasFunc, StateDim, ObsDim>,
                      "\n[UKF Error] 观测函数 h(x) 不符合要求！\n"
                      "预期签名: Eigen::Matrix<double, ObsDim, 1> h(const Eigen::Matrix<double, "
                      "StateDim,1>&)\n");

        // 1. 基于预测状态生成 Sigma Points
        auto X_sig = generate_sigma_points(x, P_);

        // 2. 预测观测 Sigma Points
        ZSigmaPoints Z_sig;
        for (int i = 0; i < 2 * StateDim + 1; ++i) {
            Z_sig.col(i) = h(X_sig.col(i));
        }

        // 3. 计算观测均值
        ZVec z_pred = Z_sig * wm_;

        // 4. 计算观测协方差 S 和 互协方差 Pxz
        Eigen::Matrix<double, ObsDim, ObsDim> S = Eigen::Matrix<double, ObsDim, ObsDim>::Zero();
        Eigen::Matrix<double, StateDim, ObsDim> Pxz =
            Eigen::Matrix<double, StateDim, ObsDim>::Zero();

        for (int i = 0; i < 2 * StateDim + 1; ++i) {
            ZVec z_diff = z_sub_op(Z_sig.col(i), z_pred);
            XVec x_diff = X_sig.col(i) - x;

            S += wc_(i) * z_diff * z_diff.transpose();
            Pxz += wc_(i) * x_diff * z_diff.transpose();
        }
        S += R;

        // 5. 计算卡尔曼增益
        auto ldlt = S.ldlt();
        if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
            return false;
        }
        Eigen::Matrix<double, StateDim, ObsDim> K =
            Pxz * ldlt.solve(Eigen::Matrix<double, ObsDim, ObsDim>::Identity());

        // 6. 更新状态
        ZVec y = z_sub_op(z, z_pred);
        x      = x_add_op(x, K * y);

        // 7. 更新协方差
        P_ = P_ - K * S * K.transpose();

        return true;
    }

private:
    PMat P_;
    double alpha_;
    double beta_;
    double kappa_;
    double lambda_;
    WeightVec wm_;  // 均值权重 (Weights for Mean)
    WeightVec wc_;  // 协方差权重 (Weights for Covariance)

    // 初始化权重
    // lambda = alpha^2 * (n + kappa) - n
    // wm_0 = lambda / (n + lambda)
    // wc_0 = lambda / (n + lambda) + (1 - alpha^2 + beta)
    // wm_i = wc_i = 1 / (2 * (n + lambda))  for i=1...2n
    void init_weights() {
        int n   = StateDim;
        lambda_ = alpha_ * alpha_ * (n + kappa_) - n;

        wm_(0) = lambda_ / (n + lambda_);
        wc_(0) = lambda_ / (n + lambda_) + (1 - alpha_ * alpha_ + beta_);

        for (int i = 1; i < 2 * n + 1; ++i) {
            wm_(i) = 1.0 / (2 * (n + lambda_));
            wc_(i) = wm_(i);
        }
    }

    // 生成 Sigma Points
    // X_0 = mean
    // X_i     = mean + (sqrt((n+lambda)*P))_i
    // X_{i+n} = mean - (sqrt((n+lambda)*P))_i
    auto generate_sigma_points(XVec const& mean, PMat const& cov) -> SigmaPoints {
        int n = StateDim;
        SigmaPoints X;

        // (n + lambda) * P
        PMat P_scaled = (n + lambda_) * cov;
        Eigen::LLT<PMat> llt(P_scaled);

        if (llt.info() == Eigen::NumericalIssue) {
            PMat P_mod = P_scaled + PMat::Identity() * 1e-9;
            llt.compute(P_mod);
        }

        PMat L = llt.matrixL();

        X.col(0) = mean;
        for (int i = 0; i < n; ++i) {
            X.col(i + 1)     = mean + L.col(i);
            X.col(i + 1 + n) = mean - L.col(i);
        }
        return X;
    }
};

}  // namespace pingpong_tracker::util