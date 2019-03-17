#ifndef MY_MODULE_H
# define MY_MODULE_H

#include <alcommon/albroker.h>

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

private:

};
#endif // MY_MODULE_H
