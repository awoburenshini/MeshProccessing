#pragma once
#include <common.h>

class GeometryIO;
typedef Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXR16u;

class HeightMap
{
    friend class GeometryIO;
    typedef Eigen::Matrix<Real, 2, 4> Matrix2x4r;

public:
    HeightMap(){

    };

    void init()
    {

        leftBottom = MAPLEFTBOTTOM;

        unitColLen = 30.;
        unitRowLen = 30.;
    };

    double getHeight(double x, double y) const
    {

        Vector2r p(x, y);
        Vector2r X = p - leftBottom;

        X(0) /= unitColLen;
        X(1) /= unitRowLen;

        Vector2i base = X.cast<int>() - Vector2i::Ones();

        Vector2r fx = X - base.cast<Real>();

        Matrix2x4r w = stdB3(fx);

        Real h = 0;

        for(int i = 0; i < 4; ++i){
            for(int j = 0; j < 4; ++j){
                Vector2i offset({i,j});
                Vector2i coord = base + offset;

                Real Bxij = w.col(i).x() * w.col(j).y();
                Real coef = safeGetCoef(coord);
                h += coef * Bxij;
            }
        }

        return h;
    };
    double getHeight(Vector2r p) const
    {
        return getHeight(p(0), p(1));
    };

    double col_length()
    {
        return unitColLen * m_data.cols();
    };
    double row_length()
    {
        return unitRowLen * m_data.rows();
    };

    Vector2r getLeftBottom()
    {
        return leftBottom;
    };
    Vector2r getRigthTop()
    {
        Vector2r rt = leftBottom;
        rt(0) += col_length();
        rt(1) += row_length();
        return rt;
    };

private:
    // standard cubic spline basis
    Matrix2x4r stdB3(const Vector2r &fx) const
    {
        Matrix2x4r w;
        Vector2r one;
        one.setOnes();
        w.col(0) = 0.16666666667 * (2. * one - fx).cwiseProduct((2. * one - fx));
        w.col(0) = w.col(0).cwiseProduct((2. * one - fx));

        w.col(1) = .5 * (fx - 1. * one).cwiseProduct(fx - 1. * one);
        w.col(1) = w.col(1).cwiseProduct(fx - 3. * one);
        w.col(1) = w.col(1) + 0.66666666666 * one;

        w.col(2) = .5 * (2. * one - fx).cwiseProduct(fx - 2. * one);
        w.col(2) = w.col(2).cwiseProduct(fx);
        w.col(2) = w.col(2) + 0.666666666666 * one;

        w.col(3) = 0.16666666667 * (fx - 1. * one).cwiseProduct(fx - 1. * one);
        w.col(3) = w.col(3).cwiseProduct(fx - 1. * one);

        return std::move(w);
    }

    Real safeGetCoef(const Vector2i& coord) const{

        if(coord(0) < 0 || coord(0) > m_data.rows()){
            return 0.;
        }

        if(coord(1) < 0 || coord(1) > m_data.cols()){
            return 0.;
        }

        return m_data(coord(0),coord(1));
    }

private:
    double unitColLen;
    double unitRowLen;
    Vector2r leftBottom;

    MatrixXR16u m_data;
};
