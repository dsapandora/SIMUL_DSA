// File:          AgenteCoche.cpp
// Date:          
// Description:   
// Author:        
// Modifications: 

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
// Note that this class derives Robot and so inherits all its functions
class AgenteCoche : public Robot {
  
  // You may need to define your own functions or variables, like
  //  LED *led;
  
  public:
    
    // AgenteCoche constructor
    AgenteCoche() : Robot() {
      
      // You should insert a getDevice-like function in order to get the
      // instance of a device of the robot. Something like:
      //  led = getLED("ledName");
      
    }

    // AgenteCoche destructor
    virtual ~AgenteCoche() {
      
      // Enter here exit cleanup code
    }
    
    // User defined function for initializing and running
    // the AgenteCoche class
    void run() {
      
      // Main loop:
      // Perform simulation steps of 64 milliseconds
      // and leave the loop when the simulation is over
      while (step(64) != -1) {
        
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = distanceSensor->getValue();
        
        // Process sensor data here
        
        // Enter here functions to send actuator commands, like:
        //  led->set(1);
      }
    }
};

// This is the main program of your controller.
// It creates an instance of your Robot subclass, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
  AgenteCoche* controller = new AgenteCoche();
  controller->run();
  delete controller;
  return 0;
}
