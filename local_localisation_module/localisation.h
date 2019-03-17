#ifndef MY_MODULE_H
# define MY_MODULE_H

#include <alcommon/albroker.h>
#include "../ParticleFilter.h"
#include <cmath>
#inlcude <vector>

namespace AL
{
  class ALBroker;
}

class Localisation : public AL::ALModule
{
public:
  Localisation(boost::shared_ptr<AL::ALBroker> broker,
           const std::string &name);

  virtual ~Localisation();

  virtual void init();

  void SetMovement();

  void SetLines(double distance, double angle, double angle_change);

private:
    ParticleFilter* filter;
    odometry control_data;
};
#endif // MY_MODULE_H
