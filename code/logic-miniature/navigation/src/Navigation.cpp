/**
 * Copyright (C) 2016 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include <cstdlib>
#include <iostream>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>
#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/strings/StringToolbox.h>

#include <odvdminiature/GeneratedHeaders_ODVDMiniature.h>

#include "Navigation.h"

namespace opendlv {
namespace logic {
namespace miniature {


/*
  Constant Definitions
*/

//const double Navigation::S_W_SIDE_DETECTION = 1.6;
//const double Navigation::S_W_FRONT_DETECTION = 1.5;
//const double Navigation::S_W_CLOSE_FRONT_DETECTION = 1.1;
//const double Navigation::S_I_SEARCH_MISS = 1.75;
//const double Navigation::S_I_SEARCH_FOUND = 1.7;
//const double Navigation::S_OUT_OF_RANGE = 1.79;



const int32_t Navigation::E_FORWARD = 50000;
const int32_t Navigation::E_ROTATE_RIGHT_L = 50000;
const int32_t Navigation::E_ROTATE_RIGHT_R = -50000;
const int32_t Navigation::E_ROTATE_LEFT_L  = -50000;
const int32_t Navigation::E_ROTATE_LEFT_R  = 50000;
const int32_t Navigation::E_STILL = 0;
const int32_t Navigation::E_DYN_TURN_SPEED = 15000;
//const uint32_t Navigation::E_SEARCH = 55000;


const uint32_t Navigation::UPDATE_FREQ = 50;


/*
  Constructor.
*/
Navigation::Navigation(const int &argc, char **argv)
    : TimeTriggeredConferenceClientModule(argc, argv, "logic-miniature-navigation")
    , m_mutex()
    , m_analogReadings()
    , m_gpioReadings()
    , m_gpioOutputPins()
    , m_pwmOutputPins()
    , m_currentState()
   // , m_s_w_Front(0)
    , m_s_w_FrontLeft(0)
    , m_s_w_FrontRight(0)
  //  , m_dynSpeedLeft(0)
 //   , m_dynSpeedRight(0)
    , m_updateCounter(0)
    , m_debug(true)
{
  m_currentState = navigationState::FORWARD;
}

/*
  Destructor.
*/
Navigation::~Navigation() 
{
}

/* 
  This method reads values from the configuration file. Note that there is only
  one global configuration storage loaded by the central odsupercomponent
  module. If the the configuration file is changed, the odsupercompnent module
  needs to be restarted.
*/
void Navigation::setUp()
{
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
  std::string const gpioPinsString = 
      kv.getValue<std::string>("logic-miniature-navigation.gpio-pins");
  std::vector<std::string> gpioPinsVector = 
      odcore::strings::StringToolbox::split(gpioPinsString, ',');
  for (auto pin : gpioPinsVector) {
    m_gpioOutputPins.push_back(std::stoi(pin)); 
  }

  std::string const pwmPinsString = 
      kv.getValue<std::string>("logic-miniature-navigation.pwm-pins");
  std::vector<std::string> pwmPinsVector = 
      odcore::strings::StringToolbox::split(pwmPinsString, ',');
  for (auto pin : pwmPinsVector) {
    m_pwmOutputPins.push_back(std::stoi(pin));
  }
}

/*
  This method is run automatically when the system is shutting down (before the
  destructor). It is typically used to close log files and de-allocate 
  dynamically allocated memory.
*/
void Navigation::tearDown()
{
}

/* 
  The while loop in this method runs at a predefined (in configuration) 
  frequency.
*/
odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Navigation::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == 
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    // The mutex is required since 'body' and 'nextContainer' competes by
    // reading and writing to the class global maps, see also 'nextContainer'.
    odcore::base::Lock l(m_mutex);

    decodeResolveSensors();

    std::string state = "";
    std::string outState = "";
    std::string comment = "";


    /*
    Logic Handling
    */

    navigationState old_state = m_currentState;
    switch(m_currentState) {
      case navigationState::FORWARD:
        state = "FORWARD";

        if (m_s_w_FrontRight) {
          outState = "TURN_LEFT";
          m_currentState = navigationState::TURN_LEFT;

        } else if (m_s_w_FrontLeft) {
          outState = "TURN_RIGHT";
          m_currentState = navigationState::TURN_RIGHT;
        }
        break; 

      case navigationState::TURN_LEFT:
        state = "TURN_LEFT";
        if (!m_s_w_FrontRight && !m_s_w_FrontLeft) {
            outState = "FORWARD";
            m_currentState = navigationState::FORWARD;
        } else if (m_s_w_FrontRight) {
            outState = "ROTATE_LEFT";
            m_currentState = navigationState::ROTATE_LEFT;
        }
        break;

      case navigationState::TURN_RIGHT:
        state = "TURN_RIGHT";
        if (!m_s_w_FrontRight && !m_s_w_FrontLeft ) {
            outState = "FORWARD";
            m_currentState = navigationState::FORWARD;
        } else if (m_s_w_FrontLeft) {
            outState = "ROTATE_RIGHT";
            m_currentState = navigationState::ROTATE_RIGHT;
        }
        break;


      case navigationState::ROTATE_RIGHT:
        state = "ROTATE_RIGHT";
        if (!m_s_w_FrontRight && !m_s_w_FrontLeft) {
            outState = "FORWARD";
            m_currentState = navigationState::FORWARD;
        }
        break;

      case navigationState::ROTATE_LEFT:
        state = "ROTATE_LEFT";
        if (!m_s_w_FrontRight && !m_s_w_FrontLeft) {
            outState = "FORWARD";
            m_currentState = navigationState::FORWARD;
        }
        break;


      default:
        state = "UNKOWN";
        outState = "FORWARD";
        m_currentState = navigationState::FORWARD;
        break;
    }

    if (m_debug) {
      std::cout << "[" << state << ":" << outState << "]: " << comment << std::endl;
    }


    /*
    Engine Speed update
    */

    if (old_state != m_currentState or m_updateCounter > UPDATE_FREQ) {
      m_updateCounter = 0;


      int32_t leftMotorDuty = E_STILL;
      int32_t rightMotorDuty = E_STILL;

      switch(m_currentState) {
        case navigationState::FORWARD:
          leftMotorDuty = E_FORWARD;
          rightMotorDuty = E_FORWARD;
          break;

        case navigationState::TURN_LEFT:
          leftMotorDuty = E_FORWARD - E_DYN_TURN_SPEED;
          rightMotorDuty = E_FORWARD;
          break;

        case navigationState::TURN_RIGHT:
          leftMotorDuty = E_FORWARD;
          rightMotorDuty = E_FORWARD - E_DYN_TURN_SPEED;
          break;

        case navigationState::ROTATE_RIGHT:
          leftMotorDuty = E_ROTATE_RIGHT_L;
          rightMotorDuty = E_ROTATE_RIGHT_R;
          break;

        case navigationState::ROTATE_LEFT:
          leftMotorDuty = E_ROTATE_LEFT_L;
          rightMotorDuty = E_ROTATE_LEFT_R;
          break;

      }



      opendlv::proxy::PwmRequest request1(0, abs(leftMotorDuty));
      odcore::data::Container c1(request1);
      getConference().send(c1);
            

      opendlv::proxy::PwmRequest request2(0, abs(rightMotorDuty));
      odcore::data::Container c2(request2);
      getConference().send(c2);


//GPIO
      opendlv::proxy::ToggleRequest::ToggleState rightMotorState1;
      opendlv::proxy::ToggleRequest::ToggleState rightMotorState2;

      if (rightMotorDuty > 0) {
        rightMotorState1 = opendlv::proxy::ToggleRequest::On;
        rightMotorState2 = opendlv::proxy::ToggleRequest::Off;
      } else {
        rightMotorState1 = opendlv::proxy::ToggleRequest::Off;
        rightMotorState2 = opendlv::proxy::ToggleRequest::On;
      }

      opendlv::proxy::ToggleRequest requestGpio1(30, rightMotorState1);
      odcore::data::Container c3(requestGpio1);
      getConference().send(c3);
      opendlv::proxy::ToggleRequest requestGpio2(31, rightMotorState2);
      odcore::data::Container c4(requestGpio2);
      getConference().send(c4);

      opendlv::proxy::ToggleRequest::ToggleState leftMotorState1;
      opendlv::proxy::ToggleRequest::ToggleState leftMotorState2;
       if (leftMotorDuty > 0) {
        leftMotorState1 = opendlv::proxy::ToggleRequest::On;
        leftMotorState2 = opendlv::proxy::ToggleRequest::Off;
      } else {
        leftMotorState1 = opendlv::proxy::ToggleRequest::Off;
        leftMotorState2 = opendlv::proxy::ToggleRequest::On;
      }

      opendlv::proxy::ToggleRequest requestGpio3(60, leftMotorState1);
      odcore::data::Container c5(requestGpio3);
      getConference().send(c5);
      opendlv::proxy::ToggleRequest requestGpio4(51, leftMotorState2);
      odcore::data::Container c6(requestGpio4);
      getConference().send(c6);



    } else {
      m_updateCounter += 1;
    }


  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void Navigation::decodeResolveSensors()
{
  m_s_w_FrontRight = m_gpioReadings[49];
  m_s_w_FrontLeft  = m_gpioReadings[48];
  std::cout  << "m_s_w_FrontRight: " << m_s_w_FrontRight << std::endl;
  std::cout  << "m_s_w_FrontLeft: " << m_s_w_FrontLeft << std::endl;

 // m_dynSpeedLeft = 1.8 - m_s_w_FrontLeft;
 // m_dynSpeedRight = 1.9 - m_s_w_FrontRight;

}







/* 
  This method receives messages from all other modules (in the same conference 
  id, cid). Here, the messages AnalogReading and ToggleReading is received
  from the modules interfacing to the hardware.
*/
void Navigation::nextContainer(odcore::data::Container &a_c)
{
  odcore::base::Lock l(m_mutex);

  int32_t dataType = a_c.getDataType();
  if (dataType == opendlv::proxy::AnalogReading::ID()) {
    opendlv::proxy::AnalogReading reading = 
        a_c.getData<opendlv::proxy::AnalogReading>();

    uint16_t pin = reading.getPin();
    float voltage = reading.getVoltage();

    m_analogReadings[pin] = voltage; // Save the input to the class global map.

    std::cout << "[" << getName() << "] Received an AnalogReading: " 
        << reading.toString() << "." << std::endl;

  } else if (dataType == opendlv::proxy::ToggleReading::ID()) {
    opendlv::proxy::ToggleReading reading = 
        a_c.getData<opendlv::proxy::ToggleReading>();

    uint16_t pin = reading.getPin();
    bool state;
    if (reading.getState() == opendlv::proxy::ToggleReading::On) {
      state = true;
    } else {
      state = false;
    }

    m_gpioReadings[pin] = state; // Save the state to the class global map.

    std::cout << "[" << getName() << "] Received a ToggleReading: "
        << reading.toString() << "." << std::endl;
  }
}

}
}
}
