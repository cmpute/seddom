#pragma once

#include "bki.h"
#include <Eigen/Dense>

#define BGKI_TDECL template <typename T, size_t Dim, size_t NumClass>
#define BGKI_CLASS SemanticBGKInference<T, Dim, NumClass>
#define BGKL_TDECL template <typename T, size_t Dim>
#define BGKL_CLASS BGKLInference<T, Dim>

namespace seddom
{
    // pair-wise Euclidean distance
    template <typename T, size_t Dim> inline Eigen::Matrix<T, -1, -1>
    dist(const Eigen::Matrix<T, -1, Dim> &x, const Eigen::Matrix<T, -1, Dim> &z)
    {
        Eigen::Matrix<T, -1, -1> d(x.rows(), z.rows());
        for (int i = 0; i < x.rows(); ++i)
            d.row(i) = (z.rowwise() - x.row(i)).rowwise().norm();
        return d;
    }

    // pair-wise point-to-line distance, assuming lines have one end at (0,0,0)
    template <typename T, size_t Dim> inline Eigen::Matrix<T, -1, -1>
    dist_pl(const Eigen::Matrix<T, -1, Dim> &l, const Eigen::Matrix<T, -1, Dim> &p)
    {
        const T max_dist = 100; // make sure max_dist > _ell
        Eigen::Matrix<T, -1, -1> d(l.rows(), p.rows());
        for (int i = 0; i < l.rows(); ++i)
        {
            T len = l.row(i).norm();
            auto dl = p.rowwise().cross(l.row(i)).rowwise().norm() / len;
            auto condition = p.rowwise().norm().array() < len; // gating with range circle
            d.row(i) = condition.select(dl, max_dist);
        }
        return d;
    }

    // pair-wise point-to-line distance with precise range cutting
    template <typename T, size_t Dim> inline Eigen::Matrix<T, -1, -1>
    dist_pl(const Eigen::Matrix<T, -1, Dim> &l, const Eigen::Matrix<T, -1, Dim> &p, Eigen::Matrix<T, -1, -1> &r)
    {
        Eigen::Matrix<T, -1, -1> d(l.rows(), p.rows());
        for (int i = 0; i < l.rows(); ++i)
        {
            T len = l.row(i).norm();
            r.row(i) = (p * l.row(i).transpose()) / (len * len);
            d.row(i) = p.rowwise().cross(l.row(i)).rowwise().norm() / len;
        }
        return d;
    }

    template <typename T, size_t Dim> inline Eigen::Matrix<T, -1, -1>
    covMaterniso3(const Eigen::Matrix<T, -1, -1> &d, float sf2)
    {
        return ((1 + d.array()) * exp(-d.array())).matrix() * sf2;
    }
    
    template <typename T, size_t Dim> inline Eigen::Matrix<T, -1, -1>
    covSparse(const Eigen::Matrix<T, -1, -1> &d, float sf2)
    {
        auto Kxz = (((2.0 + (d * DPI).array().cos()) * (1.0 - d.array()) / 3.0) + (d * DPI).array().sin() / DPI).matrix() * sf2;
        return Kxz.cwiseMax(0.0);
    }

    BGKI_TDECL void
    BGKI_CLASS::train(const typename BGKI_CLASS::MatrixXType &x, const Eigen::Matrix<uint32_t, -1, 1> &y)
    {
        _x = x;

        _y = Eigen::Matrix<T, -1, NumClass>(y.rows(), NumClass);
        for (size_t k = 0; k < NumClass; ++k)
            _y.col(k) = (y.array() == k).template cast<T>();

#ifndef NDEBUG
        trained = true;
#endif
    }

    BGKI_TDECL template <KernelType KType> typename BGKI_CLASS::MatrixYType
    BGKI_CLASS::predict(const typename BGKI_CLASS::MatrixXType &xs) const // TODO: add option to early stop updating if Ks = 0
    {
        PROFILE_FUNCTION;
#ifndef NDEBUG
        assert(trained == true && "The inference block has not been trained!");
#endif

        PROFILE_BLOCK("Calculate cov");
        typename BGKI_CLASS::MatrixKType d = dist<T, Dim>(xs, _x), Ks;
        if (KType == KernelType::BGK || KType == KernelType::SBGK)
            Ks = covSparse<T, Dim>(d / _ell, _sf2);
        else if (KType == KernelType::CSM)
            Ks = BGKI_CLASS::MatrixKType::Ones(xs.rows(), _x.rows());
        // else if (KType == KernelType::Materniso3)
        //     Ks = covMaterniso3<T, Dim>(d * SQ3 / _ell, _sf2);
        else
            assert(false);

        PROFILE_SPLIT("Dot product");
        return (Ks * _y);
    }

    BGKL_TDECL void
    BGKL_CLASS::train(const typename BGKL_CLASS::MatrixXType &x)
    {
        _x = x;
#ifndef NDEBUG
        trained = true;
#endif
    }

    BGKL_TDECL template <KernelType KType> typename BGKL_CLASS::MatrixYType
    BGKL_CLASS::predict(const typename BGKL_CLASS::MatrixXType &xs) const
    {
        PROFILE_FUNCTION;
#ifndef NDEBUG
        assert(trained == true && "The inference block has not been trained!");
#endif

        PROFILE_BLOCK("Calculate cov");
        typename BGKL_CLASS::MatrixKType d, Ks;
        if (KType == KernelType::CSM)
            Ks = BGKL_CLASS::MatrixKType::Ones(xs.rows(), _x.rows());
        else if (KType == KernelType::BGK)
        {
            d = dist_pl<T, Dim>(_x, xs).transpose();
            Ks = covSparse<T, Dim>(d / _ell, _sf2);
        }
        else if (KType == KernelType::SBGK)
        {
            typename BGKL_CLASS::MatrixKType r(_x.rows(), xs.rows());
            d = dist_pl<T, Dim>(_x, xs, r).transpose();
            auto cov = covSparse<T, Dim>(d / _ell, _sf2);
            auto rarray = r.transpose().array();
            // Ks = cov.array() * (rarray < 1).select(rarray, 0); // linear weighted
            Ks = cov.array() * (rarray < 0.5).select(rarray, 1 - rarray.cwiseMin(1)); // symmetric linear weighted
        }
        // else if (KType == KernelType::Materniso3)
        //     Ks = covMaterniso3<T, Dim>(d * SQ3 / _ell, _sf2);
        else
            assert(false);

        PROFILE_SPLIT("Dot product");
        return Ks.rowwise().sum(); // TODO: clamp max out to prevent new occupancy not being updated?
    }
}

#undef BGKI_TDECL
#undef BGKI_CLASS
#undef BGKL_TDECL
#undef BGKL_CLASS
