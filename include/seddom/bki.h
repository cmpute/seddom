#pragma once

#include "seddom/macros.h"
#include "seddom/bkioctomap.h"
#include <array>

namespace seddom
{
    enum class KernelType : char
    {
        CSM, // Counting Sensor Model
        BGK, // Bayesian Generalized Kernel Inference
        SBGK, // Scaled BGK (for point-line inference)
    };

    /*
     * @brief Bayesian Generalized Kernel Inference on Dirichlet distribution
     * @param Dim dimension of data (2, 3, etc.)
     * @param T data type (float, double, etc.)
     * @ref Nonparametric Bayesian inference on multivariate exponential families
     */
    template <typename T, size_t Dim, size_t NumClass>
    class SemanticBGKInference
    {
    public:
        /// Eigen matrix type for training and test data and kernel
        using MatrixXType = Eigen::Matrix<T, -1, Dim>;
        using MatrixKType = Eigen::Matrix<T, -1, -1>;
        using MatrixYType = Eigen::Matrix<T, -1, NumClass>;

        SemanticBGKInference(T sf2, T ell) : _sf2(sf2), _ell(ell)
        {
#ifndef NDEBUG
            trained = false;
#endif
        }

        /*
         * @brief Fit BGK Model
         * @param x input positions (Nxdim)
         * @param y input labels (Nx1)
         */
        void train(const MatrixXType &x, const Eigen::Matrix<uint32_t, -1, 1> &y);

        /*
         * @brief Fit BGK Model with no labels, probability of all labels will be considered as equal
         * @param x input positions (Nxdim)
         */
        void train(const MatrixXType &x);

        /*
         * @brief Fit BGK Model with label and score (confidence)
         * @param x input positions (Nxdim)
         * @param y input labels (Nx1)
         * @param s input scores (Nx1)
         */
        void train(const MatrixXType &x, const Eigen::Matrix<uint32_t, -1, 1> &y, const MatrixKType &s);

        /*
         * @brief Prediction with BGK Model
         * @param xs query positions (Nxdim)
         */
        template <KernelType KType>
        MatrixYType predict(const MatrixXType &xs) const;

    private:
        T _sf2; // signal variance
        T _ell; // length-scale
        MatrixXType _x; // temporary storage of training data
        MatrixYType _y; // temporary storage of training labels

#ifndef NDEBUG
        bool trained; // true if bgkinference stored training data
#endif
    };

    /*
     * @brief Bayesian Generalized Kernel Inference on Dirichlet distribution with beam line
     * @param Dim dimension of data (2, 3, etc.)
     * @param T data type (float, double, etc.)
     * @ref Nonparametric Bayesian inference on multivariate exponential families
     */
    template <typename T, size_t Dim>
    class BGKLInference
    {
    public:
        /// Eigen matrix type for training and test data and kernel
        using MatrixXType = Eigen::Matrix<T, -1, Dim>;
        using MatrixKType = Eigen::Matrix<T, -1, -1>;
        using MatrixYType = Eigen::Matrix<T, -1, 1>;
        static_assert(Dim == 3); // only works for 3D line by now

        BGKLInference(T sf2, T ell) : _sf2(sf2), _ell(ell)
        {
#ifndef NDEBUG
            trained = false;
#endif
        }
        /*
         * @brief Fit line BGK Model
         * @param x input positions (Nxdim), assuming line origin at (0, 0, 0)
         */
        void train(const MatrixXType &x);

        /*
         * @brief Prediction with BGK Model
         * @param xs query positions (Nxdim), assuming line origin at (0, 0, 0)
         */
        template <KernelType KType>
        MatrixYType predict(const MatrixXType &xs) const;

        // also return the distances from the points to the lines
        template <KernelType KType>
        MatrixYType predict(const MatrixXType &xs, MatrixKType &d, MatrixKType& r) const;

    private:
        T _sf2; // signal variance
        T _ell; // length-scale
        MatrixXType _x; // temporary storage of training data

#ifndef NDEBUG
        bool trained; // true if bgkinference stored training data
#endif
    };

    template <size_t NumClass>
    using SBGKI3f = SemanticBGKInference<float, 3, NumClass>;
    using BGKLI3f = BGKLInference<float, 3>;

}

#include "impl/bki.hpp"
