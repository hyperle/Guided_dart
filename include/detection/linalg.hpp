#pragma once

// ============================================================================
// 固定尺寸小矩阵：只服务本层的卡尔曼滤波（6×6 状态协方差 + 3×3 新息协方差）。
//
// 为什么不引 Eigen：板端是静态链接 + 16MB 用户堆（K230D 上 KPU 模型还要占一大块），
// 而我们真正需要的只有"乘、转置、求逆"三件事。模板 100 行足够，还能整体内联 ——
// 6×6 的运算在 -O2 下会被完全展开成常量下标访问，没有动态分配、没有虚函数。
//
// 精度：默认 double。状态量在像素量级（≤1e3），协方差跨 1e-4~1e4，
// 单精度在求逆那一步余量偏小（S 矩阵接近奇异时更明显）；而这里的运算量
// 每帧只有几百次浮点乘加，用 double 的代价可以忽略。
// ============================================================================

#include <cmath>
#include <cstddef>

namespace dart::detection::linalg {

template <int R, int C, typename T = double>
struct Mat {
    T a[R][C]{};

    T       &operator()(int i, int j) { return a[i][j]; }
    const T &operator()(int i, int j) const { return a[i][j]; }

    static Mat identity() {
        static_assert(R == C, "identity 只对方阵有意义");
        Mat m;
        for (int i = 0; i < R; ++i)
            m(i, i) = T(1);
        return m;
    }

    static Mat zero() { return Mat{}; }

    // 对称化：卡尔曼更新里 P -= K·(H·P) 在浮点下会丢掉对称性，
    // 长跑几十万帧后可能出现"负方差"这种物理上不可能的状态。每次更新后对称化一次。
    void symmetrize() {
        static_assert(R == C, "symmetrize 只对方阵有意义");
        for (int i = 0; i < R; ++i) {
            for (int j = i + 1; j < R; ++j) {
                const T s = T(0.5) * (a[i][j] + a[j][i]);
                a[i][j] = s;
                a[j][i] = s;
            }
        }
    }
};

template <int R, int K, int C, typename T>
inline Mat<R, C, T> mul(const Mat<R, K, T> &x, const Mat<K, C, T> &y) {
    Mat<R, C, T> o;
    for (int i = 0; i < R; ++i)
        for (int k = 0; k < K; ++k) {
            const T xv = x(i, k);
            if (xv == T(0))
                continue;
            for (int j = 0; j < C; ++j)
                o(i, j) += xv * y(k, j);
        }
    return o;
}

template <int R, int C, typename T>
inline Mat<C, R, T> transpose(const Mat<R, C, T> &x) {
    Mat<C, R, T> o;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j)
            o(j, i) = x(i, j);
    return o;
}

// Gauss-Jordan + 部分主元求逆。奇异返回 false —— 调用方必须按"这一帧不更新"处理，
// 绝不能返回一堆 NaN 让滤波器悄悄烂掉（板端没法调试一个 NaN 状态）。
template <int N, typename T>
inline bool inverse(const Mat<N, N, T> &src, Mat<N, N, T> *out) {
    Mat<N, N, T> a = src;
    Mat<N, N, T> inv = Mat<N, N, T>::identity();

    for (int col = 0; col < N; ++col) {
        // 选主元：取当前列绝对值最大的行，避免除小数放大误差
        int    piv = col;
        double best = std::fabs(static_cast<double>(a(col, col)));
        for (int r = col + 1; r < N; ++r) {
            const double v = std::fabs(static_cast<double>(a(r, col)));
            if (v > best) {
                best = v;
                piv = r;
            }
        }
        if (best < 1e-12)
            return false; // 奇异（或病态到不可用）：宁可拒更新
        if (piv != col) {
            for (int j = 0; j < N; ++j) {
                const T t = a(col, j);
                a(col, j) = a(piv, j);
                a(piv, j) = t;
                const T u = inv(col, j);
                inv(col, j) = inv(piv, j);
                inv(piv, j) = u;
            }
        }

        const T d = a(col, col);
        for (int j = 0; j < N; ++j) {
            a(col, j) = a(col, j) / d;
            inv(col, j) = inv(col, j) / d;
        }
        for (int r = 0; r < N; ++r) {
            if (r == col)
                continue;
            const T f = a(r, col);
            if (f == T(0))
                continue;
            for (int j = 0; j < N; ++j) {
                a(r, j) -= f * a(col, j);
                inv(r, j) -= f * inv(col, j);
            }
        }
    }

    *out = inv;
    return true;
}

} // namespace dart::detection::linalg
