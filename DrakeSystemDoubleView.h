//
// Created by Twan Koolen on 9/3/15.
//

#ifndef CRTPDEMO_DRAKESYSTEMDOUBLEVIEW_H
#define CRTPDEMO_DRAKESYSTEMDOUBLEVIEW_H

#include "DrakeSystem.h"

class DrakeSystemDoubleView {
public:
  std::function<Eigen::VectorXd(double, const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamics;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> output;

  template <typename Sys>
  DrakeSystemDoubleView(const DrakeSystem<Sys>* system) :
      dynamics([=] (double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {return system->dynamics(t, x, u);}),
      output([=] (const Eigen::VectorXd& x) {return system->output(x);})
  {
  }
};


#endif //CRTPDEMO_DRAKESYSTEMDOUBLEVIEW_H
