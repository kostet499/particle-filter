#include "localisation.h"

#include <iostream>

Localisation::Localisation(boost::shared_ptr<AL::ALBroker> broker,
                   const std::string& name)
  : AL::ALModule(broker, name), tts_(getParentBroker())
{
  setModuleDescription("Localisation module");

  /*
  functionName("LocalisationMethod", getName(), "some method");
  BIND_METHOD(MyModule::LocalisationMethod);
  */

  /**
   * addParam(<attribut_name>, <attribut_descrption>);
   * This enables to document the parameters of the method.
   * It is not compulsory to write this line.
   */

  /**
   * setReturn(<return_name>, <return_description>);
   * This enables to document the return of the method.
   * It is not compulsory to write this line.
   */
}

Localisation::~Localisation()
{
  
}

void Localisation::init()
{

}
