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
#include <opendavinci/odcore/data/TimeStamp.h>

#include <opendavinci/odcore/wrapper/Eigen.h>

#include <odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h>
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


const double Navigation::T_REVERSE = 0.7;
const double Navigation::T_ROTATE_REVERSE = 0.2;
const double Navigation::T_TURN = 1;


const int32_t Navigation::E_FORWARD = 36000;
const int32_t Navigation::E_REVERSE = -35000;
const int32_t Navigation::E_ROTATE_RIGHT_L = 35000;
const int32_t Navigation::E_ROTATE_RIGHT_R = -35000;
const int32_t Navigation::E_ROTATE_LEFT_L  = -35000;
const int32_t Navigation::E_ROTATE_LEFT_R  = 35000;
const int32_t Navigation::E_STILL = 0;
const int32_t Navigation::E_DYN_TURN_SPEED = 15000;
//const uint32_t Navigation::E_SEARCH = 55000;


const uint32_t Navigation::UPDATE_FREQ = 50;

const uint8_t Navigation::WALL_MARGINS = 1;


/*
  Constructor.
*/
Navigation::Navigation(const int &argc, char **argv)
    : TimeTriggeredConferenceClientModule(argc, argv, "logic-miniature-navigation")
    , m_mutex()
    , m_outerWalls()
    , m_innerWalls()
    , m_pointsOfInterest()
    , m_analogReadings()
    , m_gpioReadings()
    , m_gpioOutputPins()
    , m_pwmOutputPins()
    , m_graph()
    , m_path()

    , m_currentState()
    , m_lastState()
    , m_currentModifer()

    , m_t_Current()
    , m_t_Last()
    , m_MotorDuties()

    , m_s_w_FrontLeft(0)
    , m_s_w_FrontLeft_t()
    , m_s_w_FrontRight(0)
    , m_s_w_FrontRight_t()
    , m_updateCounter(0)
    , m_debug(true)
    , m_gpsFix(false)
    , m_posX()
    , m_posY()
    , m_Yaw()
{
  m_lastState =  navigationState::PLAN;
  m_currentState = navigationState::PLAN;
  m_currentModifer = stateModifier::NONE;
  m_t_Current = odcore::data::TimeStamp();
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
  
  std::string const outerWallsString = 
      kv.getValue<std::string>("logic-miniature-navigation.outer-walls");
  std::vector<data::environment::Point3> outerWallPoints = ReadPointString(outerWallsString);
  if (outerWallPoints.size() == 4) {
    m_outerWalls.push_back(data::environment::Line(outerWallPoints[0], outerWallPoints[1]));
    m_outerWalls.push_back(data::environment::Line(outerWallPoints[1], outerWallPoints[2]));
    m_outerWalls.push_back(data::environment::Line(outerWallPoints[2], outerWallPoints[3]));
    m_outerWalls.push_back(data::environment::Line(outerWallPoints[3], outerWallPoints[0]));

    std::cout << "Outer walls 1 - " << m_outerWalls[0].toString() <<  std::endl;
    std::cout << "Outer walls 2 - " << m_outerWalls[1].toString() <<  std::endl;
    std::cout << "Outer walls 3 - " << m_outerWalls[2].toString() <<  std::endl;
    std::cout << "Outer walls 4 - " << m_outerWalls[3].toString() <<  std::endl;
  } else {
    std::cout << "Warning: Outer walls format error. (" << outerWallsString << ")" << std::endl;
  }
  
  std::string const innerWallsString = 
      kv.getValue<std::string>("logic-miniature-navigation.inner-walls");
  std::vector<data::environment::Point3> innerWallPoints = ReadPointString(innerWallsString);
  for (uint32_t i = 0; i < innerWallPoints.size(); i += 2) {
    if (i < innerWallPoints.size() - 1) {
      data::environment::Line innerWall(innerWallPoints[i], innerWallPoints[i+1]);
      m_innerWalls.push_back(innerWall);
      std::cout << "Inner wall - " << innerWall.toString() << std::endl;
    }
  }
  
  std::string const pointsOfInterestString = 
      kv.getValue<std::string>("logic-miniature-navigation.points-of-interest");
  m_pointsOfInterest = ReadPointString(pointsOfInterestString);
  for (uint32_t i = 0; i < m_pointsOfInterest.size(); i++) {
    std::cout << "Point of interest " << i << ": " << m_pointsOfInterest[i].toString() << std::endl;
  }

  createGraph();
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

    //Update the current time
    m_t_Current = odcore::data::TimeStamp();
    // 
    decodeResolveSensors();
    
    navigationState old_state = m_currentState;
    logicHandling();
    std::array<int32_t, 2> motorDuties = engineHandling();

    /*
    Engine Speed update
    */

    // MotorDuties[0] is left Engine
    // MotorDuties[1] is right Engine

    std::cout << "Motor Duty" << m_MotorDuties[0] << ":" <<  m_MotorDuties[1] << std::endl;


    if (motorDuties[0] != m_MotorDuties[0] or 
        motorDuties[1] != m_MotorDuties[1] or 
        old_state != m_currentState or
        m_updateCounter > UPDATE_FREQ) {
      
      m_MotorDuties = motorDuties;

      if (old_state != m_currentState) {
        m_lastState = m_currentState;
        m_t_Last = m_t_Current;
      }
      m_updateCounter = 0;


    
      //Left wheel

      //Power
      opendlv::proxy::PwmRequest request1(0, abs(motorDuties[0]));
      odcore::data::Container c1(request1);
      c1.setSenderStamp(0);
            
      opendlv::proxy::ToggleRequest::ToggleState leftMotorState1;
      opendlv::proxy::ToggleRequest::ToggleState leftMotorState2;
       if (motorDuties[0] > 0) {
        leftMotorState1 = opendlv::proxy::ToggleRequest::On;
        leftMotorState2 = opendlv::proxy::ToggleRequest::Off;
      } else {
        leftMotorState1 = opendlv::proxy::ToggleRequest::Off;
        leftMotorState2 = opendlv::proxy::ToggleRequest::On;
      }

      //Set the direction
      opendlv::proxy::ToggleRequest requestGpio3(60, leftMotorState1);
      odcore::data::Container c5(requestGpio3);
      opendlv::proxy::ToggleRequest requestGpio4(51, leftMotorState2);
      odcore::data::Container c6(requestGpio4);


      //Right wheel

      //Power
      opendlv::proxy::PwmRequest request2(0, abs(motorDuties[1]));
      odcore::data::Container c2(request2);
      c2.setSenderStamp(2);


      // Set direction
      opendlv::proxy::ToggleRequest::ToggleState rightMotorState1;
      opendlv::proxy::ToggleRequest::ToggleState rightMotorState2;

      if (motorDuties[1] > 0) {
        rightMotorState1 = opendlv::proxy::ToggleRequest::On;
        rightMotorState2 = opendlv::proxy::ToggleRequest::Off;
      } else {
        rightMotorState1 = opendlv::proxy::ToggleRequest::Off;
        rightMotorState2 = opendlv::proxy::ToggleRequest::On;
      }

      opendlv::proxy::ToggleRequest requestGpio1(30, rightMotorState1);
      odcore::data::Container c3(requestGpio1);
      opendlv::proxy::ToggleRequest requestGpio2(31, rightMotorState2);
      odcore::data::Container c4(requestGpio2);

      //Send the data
      getConference().send(c1);
      getConference().send(c2);
      getConference().send(c3);
      getConference().send(c4);
      getConference().send(c5);
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
  if (m_s_w_FrontRight) {
    m_s_w_FrontLeft_t = m_t_Current;
  }
  m_s_w_FrontLeft  = m_gpioReadings[48];
  if (m_s_w_FrontLeft) {
    m_s_w_FrontLeft_t = m_t_Current;
  }

}



  void Navigation::logicHandling() 
  {
    std::string state = "";
    std::string outState = "";
    std::string comment = "";

    double t1 = 0;
    double t2 = 0;
    std::vector<double> v;


    switch(m_currentState) { 
      case navigationState::REVERSE:
        state = "REVERSE";


        t1 = static_cast<double>(m_t_Current.toMicroseconds() - m_s_w_FrontLeft_t.toMicroseconds()) / 1000000.0;
        t2 = static_cast<double>(m_t_Current.toMicroseconds() - m_s_w_FrontRight_t.toMicroseconds()) / 1000000.0;
        v.push_back(t1);
        v.push_back(t2);

        if (modifierHandling(v, T_TURN)) 
        {
          if (t1 > t2) {
            outState = "ROTATE_RIGHT";
            m_currentState = navigationState::ROTATE_RIGHT;
          } else {
            outState = "ROTATE_LEFT";
            m_currentState = navigationState::ROTATE_LEFT;
          }
        }
        //TODO: logic for US
        break;

      case navigationState::ROTATE_RIGHT:
        state = "ROTATE_RIGHT";

        if (modifierHandling(m_t_Last, T_TURN))
        {
          if (!m_s_w_FrontRight && !m_s_w_FrontLeft) {
            outState = "FOLLOW";
            m_currentState = navigationState::FOLLOW; 
          } else if (m_s_w_FrontRight && m_s_w_FrontLeft) {
            outState = "REVERSE";
            m_currentState = navigationState::REVERSE; 
          }
        }
        //TODO: logic for US
        break;

      case navigationState::ROTATE_LEFT:
        state = "ROTATE_LEFT";
        if (modifierHandling(m_t_Last, T_TURN))
        {
          if(!m_s_w_FrontRight && !m_s_w_FrontLeft) {
              outState = "FOLLOW";
              m_currentState = navigationState::FOLLOW;
          } else if (m_s_w_FrontRight && m_s_w_FrontLeft) {
            outState = "REVERSE";
            m_currentState = navigationState::REVERSE; 
          }
        }
        //TODO: logic for US
        break;

      case navigationState::PLAN:
        state = "PLAN";
        if (m_updateCounter == 1) {
          calculatePath();

          for (auto node : m_path){
              cout << "Path:" << node.toString() << std::endl; 

          }
        } else if (m_updateCounter > 1) {
          outState = "FOLLOW";
          m_currentState = navigationState::FOLLOW;
        }
        break;

      case navigationState::FOLLOW:
        state = "FOLLOW";

        if (m_s_w_FrontLeft && m_s_w_FrontRight) {
          outState = "REVERSE";
          m_currentState = navigationState::REVERSE;
          m_currentModifer = stateModifier::DELAY;

        } else if (m_s_w_FrontRight) {
          outState = "ROTATE_LEFT";
          m_currentState = navigationState::ROTATE_LEFT;
          m_currentModifer = stateModifier::DELAY;

        } else if (m_s_w_FrontLeft) {
          outState = "ROTATE_RIGHT";
          m_currentState = navigationState::ROTATE_RIGHT;
          m_currentModifer = stateModifier::DELAY;
        }
        //TODO: logic for ultrasound
        //TODO: logic for pathplanning if too far from path
        break;

      default:
        state = "UNKOWN";
        outState = "PLAN";
        m_currentState = navigationState::PLAN;
        break;
    }

    if (m_debug){
      if(m_currentModifer == stateModifier::DELAY) {
        std::cout << "[NAVSTATE:" << state << "(DELAY):" << outState << "]" << std::endl;
      } else {
        std::cout << "[NAVSTATE:" << state << ":" << outState << "]" << std::endl;
      }
    }

  }
  void Navigation::pathPlanning() {
    return;
  }

  /*
  * engineHandling returns the engine values to be set
  *
  * It returns the duty cycle values for the engines
  * depending on the state and other factors
  * 
  */
  std::array<int32_t, 2> Navigation::engineHandling(){
    std::array<int32_t, 2> out;
    out[0] = 0;
    out[1] = 0;

    switch(m_currentState) {
      case navigationState::REVERSE:
        out[0] = E_REVERSE;
        out[1] = E_REVERSE;
        break;

      case navigationState::ROTATE_RIGHT:
        out[0]  = E_ROTATE_RIGHT_L;
        out[1] = E_ROTATE_RIGHT_R;
        break;

      case navigationState::ROTATE_LEFT:
        out[0]  = E_ROTATE_LEFT_L;
        out[1] = E_ROTATE_LEFT_R;
        break;

      case navigationState::FOLLOW:
      if (m_gpsFix)
        return followPreview();
      else
        return forward();
      

      case navigationState::PLAN:
        out[0] = E_STILL;
        out[1] = E_STILL;
        break;
    }

    return out;


  }

  //Handles the engine logic to follow the preview point
  std::array<int32_t,2> Navigation::followPreview() {
    return forward();
  }


  std::array<int32_t,2> Navigation::forward() {
    std::array<int32_t, 2> out;
    out[0] = E_FORWARD;
    out[1] = E_FORWARD;
    return out;
  }



  //Different versions of modifierHandling
  
  bool Navigation::modifierHandling(const std::vector<double> &since, const std::vector<double> &until) 
  {
    if (m_currentModifer == stateModifier::NONE) {
      return true;

    } else if (m_currentModifer == stateModifier::DELAY) {
      if (since.size() != until.size()) {
        return false;
      }

      for(uint i=0; i < since.size(); i++){
        if (since[i] <= until[i]) {
          return false;
        }
      }
      return true;
    }
    return false;
  }

  bool Navigation::modifierHandling(const std::vector<odcore::data::TimeStamp> &since, const std::vector<double> &until) 
  {
    if (m_currentModifer == stateModifier::NONE) {
      return true;

    } else if (m_currentModifer == stateModifier::DELAY) {
      if (since.size() != until.size()) {
        return false;
      }

      for(uint i=0; i < since.size(); i++){
        double t = static_cast<double>(m_t_Current.toMicroseconds() - since[i].toMicroseconds()) / 1000000.0;
        if (t <= until[i]) {
          return false;
        }
      }
      return true;
    }
    return false;
  }
  
  bool Navigation::modifierHandling(const std::vector<double> &since, const double &until)
  {
    if (m_currentModifer == stateModifier::NONE) {
      return true;

    } else if (m_currentModifer == stateModifier::DELAY) {

      for(uint i=0; i < since.size(); i++){
        if (since[i] <= until) {
          return false;
        }
      }
      return true;
    }
    return false;
  }

  bool Navigation::modifierHandling(const std::vector<odcore::data::TimeStamp> &since, const double &until)
  {
    if (m_currentModifer == stateModifier::NONE) {
      return true;

    } else if (m_currentModifer == stateModifier::DELAY) {

      for(uint i=0; i < since.size(); i++){
        double t = static_cast<double>(m_t_Current.toMicroseconds() - since[i].toMicroseconds()) / 1000000.0;
        if (t <= until) {
          return false;
        }
      }
      return true;
    }
    return false;
  }

  bool Navigation::modifierHandling(const odcore::data::TimeStamp &since, const double &until)
  {
    double t = static_cast<double>(m_t_Current.toMicroseconds() - since.toMicroseconds()) / 1000000.0;
    return modifierHandling(t, until);
  }

  bool Navigation::modifierHandling(const double &since, const double &until){
    if (m_currentModifer == stateModifier::NONE) {
      return true;

    } else if (m_currentModifer == stateModifier::DELAY) {
      if (since > until) {
        return true;
      }

    }
    return false;
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

std::vector<data::environment::Point3> Navigation::ReadPointString(std::string const &a_pointsString) const
{
  std::vector<data::environment::Point3> points;
  std::vector<std::string> pointStringVector = 
      odcore::strings::StringToolbox::split(a_pointsString, ';');
  for (auto pointString : pointStringVector) {
    std::vector<std::string> coordinateVector = 
        odcore::strings::StringToolbox::split(pointString, ',');
    if (coordinateVector.size() == 2) {
      double x = std::stod(coordinateVector[0]);
      double y = std::stod(coordinateVector[1]);
      double z = 0.0;
      points.push_back(data::environment::Point3(x, y, z));
    }
  }
  return points;
}

void Navigation::createGraph(void){

    //data::environment::Point3 pointX(WALL_MARGINS, 0, 0);
    //data::environment::Point3 pointY(0, WALL_MARGINS, 0);

    std::vector<std::array<float, 4>> inWallLimits;
    std::array<float, 4> inWallLimit = {{0, 0, 0, 0}};
    std::array<float, 4> outWallLimit = {{0, 0, 0, 0}};
    int t = 0;

    for (auto lineInner : m_innerWalls) {
      inWallLimit[0] = ((lineInner.getA().getX()+WALL_MARGINS) > (lineInner.getB().getX()+WALL_MARGINS)) ? (lineInner.getA().getX()+WALL_MARGINS) : (lineInner.getB().getX()+WALL_MARGINS);
      inWallLimit[1] = ((lineInner.getA().getX()-WALL_MARGINS) < (lineInner.getB().getX()-WALL_MARGINS)) ? (lineInner.getA().getX()-WALL_MARGINS) : (lineInner.getB().getX()-WALL_MARGINS);
      inWallLimit[2] = ((lineInner.getA().getY()+WALL_MARGINS) > (lineInner.getB().getY()+WALL_MARGINS)) ? (lineInner.getA().getY()+WALL_MARGINS) : (lineInner.getB().getY()+WALL_MARGINS);
      inWallLimit[3] = ((lineInner.getA().getY()-WALL_MARGINS) < (lineInner.getB().getY()-WALL_MARGINS)) ? (lineInner.getA().getY()-WALL_MARGINS) : (lineInner.getB().getY()-WALL_MARGINS);

      inWallLimits.push_back(inWallLimit);
      std::cout << "innerWalls:" << inWallLimit[0] << ","<< inWallLimit[1] << ","<< inWallLimit[2] << ","<< inWallLimit[3] << std::endl;

      t++;
    }

    t = 1;

    for (auto lineOuter : m_outerWalls) {
      switch(t){
        case 1:
          outWallLimit[2] = ((lineOuter.getA().getY()+WALL_MARGINS) > (lineOuter.getB().getY()+WALL_MARGINS)) ? (lineOuter.getA().getY()+WALL_MARGINS) : (lineOuter.getB().getY()+WALL_MARGINS);
          break;
        case 2:
          outWallLimit[0] = ((lineOuter.getA().getX()+WALL_MARGINS) > (lineOuter.getB().getX()+WALL_MARGINS)) ? (lineOuter.getA().getX()+WALL_MARGINS) : (lineOuter.getB().getX()+WALL_MARGINS);
          break;
        case 3:
          outWallLimit[3] = ((lineOuter.getA().getY()-WALL_MARGINS) < (lineOuter.getB().getY()-WALL_MARGINS)) ? (lineOuter.getA().getY()-WALL_MARGINS) : (lineOuter.getB().getY()-WALL_MARGINS);
          break;
        case 4:
          outWallLimit[1] = ((lineOuter.getA().getX()-WALL_MARGINS) < (lineOuter.getB().getX()-WALL_MARGINS)) ? (lineOuter.getA().getX()-WALL_MARGINS) : (lineOuter.getB().getX()-WALL_MARGINS);
          break;
        default:
          break;
    }
     
      t++;
    }
      std::cout << "OuterWalls:" << outWallLimit[0] << ","<< outWallLimit[1] << ","<< outWallLimit[2] << ","<< outWallLimit[3] << std::endl;

      t = 1;
      bool blocked = false;
      opendlv::logic::miniature::graph currentGraph;
      data::environment::Point3 currentNode(0, 0, 0);

      for (double yNodes = round((double) outWallLimit[2]+0.5);  yNodes < round((double) outWallLimit[3]+0.5); yNodes += 2){
        for (double xNodes = round((double) outWallLimit[0]+0.5); xNodes < round((double) outWallLimit[1]+0.5); xNodes += 2){
            blocked = false;
            for(auto innerArray : inWallLimits){
              if (xNodes < (double) innerArray[0] && xNodes > (double) innerArray[1] && yNodes < (double) innerArray[2] && yNodes > (double) innerArray[3]){
                  blocked = true;
                  break;
              }
        
            }
            
            if (!blocked){
              currentNode.setX(xNodes);
              currentNode.setY(yNodes);
              currentGraph.node = currentNode;
              currentGraph.dist = 100000;
              m_graph.push_back(currentGraph);

              //std::cout << "Nodes" << currentNode.toString() << "," << currentGraph.dist << std::endl;
              t++;
            }
         }
      }
}

void Navigation::calculatePath(){
    std::vector<graph> graphStorage = m_graph;
    std::vector<graph> graphSearch = m_graph;


    //data::environment::Point3 startNode(round(m_posX/2)*2, round(m_posY/2)*2, 0);


    data::environment::Point3 startNode(round(m_pointsOfInterest.at(3).getX()/2)*2, round(m_pointsOfInterest.at(3).getY()/2)*2, 0);
    data::environment::Point3 stopNode(round(m_pointsOfInterest.at(2).getX()/2)*2, round(m_pointsOfInterest.at(2).getY()/2)*2, 0);
    
    std::vector<data::environment::Point3> neighNodes;

    data::environment::Point3 neighNode(0,0,0);
    data::environment::Point3 currentNode(0,0,0);

    int t = 0;
    for (auto graphs : graphSearch){
      if (graphs.node == startNode){
        graphStorage.at(t).dist = 0;
        graphSearch.at(t).dist = 0;
        cout << "startNode" << graphs.node.toString() << std::endl;
        break;
      }
      t++;
    }
    
    int smallestDistInd = 0;
    int loopIndex = 0;

    while(!graphSearch.empty()){
        loopIndex = 0;
        smallestDistInd = 0;

        for (auto graphs : graphSearch){
          if (graphs.dist < graphSearch.at(smallestDistInd).dist){
            smallestDistInd = loopIndex;
          }
          loopIndex++;
        }
        cout << "SmallestNode" << graphSearch.at(smallestDistInd).node.toString() << std::endl;
        neighNodes.clear();

        neighNode.setX(graphSearch.at(smallestDistInd).node.getX()-2);
        neighNode.setY(graphSearch.at(smallestDistInd).node.getY());
        neighNodes.push_back(neighNode);

        neighNode.setX(graphSearch.at(smallestDistInd).node.getX()+2);
        neighNode.setY(graphSearch.at(smallestDistInd).node.getY());
        neighNodes.push_back(neighNode);

        neighNode.setX(graphSearch.at(smallestDistInd).node.getX());
        neighNode.setY(graphSearch.at(smallestDistInd).node.getY()-2);
        neighNodes.push_back(neighNode);

        neighNode.setX(graphSearch.at(smallestDistInd).node.getX());
        neighNode.setY(graphSearch.at(smallestDistInd).node.getY()+2);
        neighNodes.push_back(neighNode);

        int neighLoops = 0;
        int neigthStoreLoop = 0;
        int tempDist = 0;


        for (auto neighNodePoint : neighNodes){
              neighLoops = 0;
              //cout << "Neighbor" << neighNodePoint.toString() << std::endl;
              loopIndex = 0;
              for (auto graphs : graphSearch){
                  if (graphs.node == neighNodePoint){
                      tempDist = (graphSearch.at(smallestDistInd).dist + 2);

                      if(tempDist < graphSearch.at(loopIndex).dist){
                        //cout << "Dist" << tempDist << std::endl;
                          graphSearch.at(loopIndex).dist = tempDist;

                            neigthStoreLoop = 0;
                            for (auto graphs2 : graphStorage){
                              if (graphs2.node == neighNodePoint){
                                graphStorage.at(neigthStoreLoop).prevPoint = graphSearch.at(smallestDistInd).node;
                                graphStorage.at(neigthStoreLoop).dist = tempDist;
                                //cout << "Neighbor" << graphs2.node.toString() << std::endl;
                                break;
                              }
                              neigthStoreLoop++;
                            }

                      }

                    }
                 loopIndex++;

                }
                neighLoops++;
              }

              graphSearch.erase(graphSearch.begin() + smallestDistInd);
        }
        

        loopIndex = 0;
        int bestIndex = 0;

        for (auto graphs : graphStorage){
          if (graphs.node == stopNode){
            bestIndex = loopIndex;
            break;
          }
          loopIndex++;
        }

        int prevIndex = 0;
        prevIndex = bestIndex;

        //cout << "endNode" << stopNode.toString() << std::endl;

        m_path.clear();

        while(graphStorage.at(prevIndex).node != startNode){
            currentNode = graphStorage.at(prevIndex).node;
            m_path.push_back(currentNode);
            //cout << "Path" << currentNode.toString() << std::endl;


            loopIndex = 0;

            for (auto graphs2 : graphStorage){
                  if (graphs2.node == graphStorage.at(prevIndex).prevPoint){
                      prevIndex = loopIndex;
                      break;
                  }
                  loopIndex++;
            }

          //m_path.push_back()
        }
        m_path.push_back(startNode);

        std::reverse(m_path.begin(), m_path.end());

        




}


}
}
}
