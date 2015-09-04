//
// Created by Twan Koolen on 9/3/15.
//

#ifndef CRTPDEMO_TYPEDEFS_H
#define CRTPDEMO_TYPEDEFS_H

template<typename DerivedX> using XdotType = Eigen::Matrix<typename DerivedX::Scalar, DerivedX::RowsAtCompileTime, DerivedX::ColsAtCompileTime>;
template<typename DerivedX> using YType = Eigen::Matrix<typename DerivedX::Scalar, Eigen::Dynamic, 1>;

#endif //CRTPDEMO_TYPEDEFS_H
