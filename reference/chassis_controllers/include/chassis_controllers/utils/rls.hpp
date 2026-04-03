//
// Created by aim on 2026/4/2.
//

#ifndef BUILD_RLS_HPP
#define BUILD_RLS_HPP

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <chrono>

template<uint32_t dim>
class RLS {
public:
    using VectorType = Eigen::Matrix<float, dim, 1>;
    using MatrixType = Eigen::Matrix<float, dim, dim>;

    /**
     * @brief Delete the default constructor
     */
    RLS() = delete;

    /**
     * @brief The constructor
     * @param delta_ The initialized non-singular value of the transfer matrix
     * @param lambda_ The forgotten index
     */
    constexpr RLS(float delta_, float lambda_)
        : dimension(dim),
          lambda(lambda_),
          delta(delta_),
          lastUpdate(std::chrono::system_clock::now()),
          updateCnt(0),
          defaultParamsVector(VectorType::Zero()),
          output(0.0f) {
        this->reset();
    }

    RLS(float delta_, float lambda_, const VectorType &initParam)
        : RLS(delta_, lambda_) {
        defaultParamsVector = initParam;
        paramsVector = initParam;
    }

    /**
     * @brief Reset the RLS module
     * @retval None
     */
    void reset() {
        transMatrix = MatrixType::Identity() * delta;
        gainVector = VectorType::Zero();
        paramsVector = defaultParamsVector;
    }

    /**
     * @brief Process a cycle of RLS update
     * @param sampleVector The new samples input expressed in n x 1 dimensional vector form
     * @param actualOutput The actual feedback real output
     * @retval paramsVector
     *
     * RLS 递推公式:
     *   K(n)   = P(n-1) * x(n) / [λ + x(n)^T * P(n-1) * x(n)]
     *   θ(n)   = θ(n-1) + K(n) * [y(n) - x(n)^T * θ(n-1)]
     *   P(n)   = [P(n-1) - K(n) * x(n)^T * P(n-1)] / λ
     */
    const VectorType &update(const VectorType &sampleVector, float actualOutput) {
        VectorType Px = transMatrix * sampleVector;
        float denom = lambda + sampleVector.dot(Px);
        gainVector = Px / denom;
        float priorError = actualOutput - sampleVector.dot(paramsVector);
        paramsVector += gainVector * priorError;
        transMatrix = (transMatrix - gainVector * sampleVector.transpose() * transMatrix) / lambda;
        output = sampleVector.dot(paramsVector);

        updateCnt++;
        lastUpdate = std::chrono::system_clock::now();
        return paramsVector;
    }

    /**
     * @brief Set the default regression parameters
     * @param updatedParams
     * @retval None
     */
    void setParamVector(const VectorType &updatedParams) {
        paramsVector = updatedParams;
        defaultParamsVector = updatedParams;
    }

    /**
     * @brief The getter function of the params vector
     * @retval paramsVector
     */
    const VectorType &getParamsVector() const { return paramsVector; }

    /**
     * @brief The getter function of the output
     * @retval The estimated / filtered output of the RLS module
     */
    float getOutput() const { return output; }

    /**
     * @brief Get the transfer (covariance) matrix
     * @retval transMatrix
     */
    const MatrixType &getTransMatrix() const { return transMatrix; }

    /**
     * @brief Get the gain vector
     * @retval gainVector
     */
    const VectorType &getGainVector() const { return gainVector; }

    /**
     * @brief Get the update count
     * @retval updateCnt
     */
    uint32_t getUpdateCount() const { return updateCnt; }

private:
    uint32_t dimension; // Dimension of the RLS space
    float lambda; // The forget factor (0 < λ ≤ 1)
    float delta; // Initialized value of the transfer matrix

    std::chrono::system_clock::time_point lastUpdate; // Last update tick
    uint32_t updateCnt; // Total update count

    /* RLS relevant matrices */
    MatrixType transMatrix; // Transfer (covariance) matrix P
    VectorType gainVector; // Gain vector K
    VectorType paramsVector; // Parameter vector θ
    VectorType defaultParamsVector; // Default parameter vector
    float output; // Estimated / filtered output
};

#endif // BUILD_RLS_HPP
