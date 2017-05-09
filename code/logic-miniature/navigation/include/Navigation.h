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

#ifndef LOGIC_MINIATURE_NAVIGATION_H
#define LOGIC_MINIATURE_NAVIGATION_H

#include <map>
#include <memory>

#include <opendavinci/odcore/base/Mutex.h>
#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>

namespace opendlv {
namespace logic {
namespace miniature {


enum class navigationState
{
  FORWARD,
  REVERSE,
  TURN_RIGHT,
  TURN_LEFT,
  ROTATE_RIGHT,
  ROTATE_RIGHT_DELAY,
  ROTATE_RIGHT_REVERSE,
  ROTATE_LEFT,
  ROTATE_LEFT_DELAY,
  ROTATE_LEFT_REVERSE
};


class Navigation : 
  public odcore::base::module::TimeTriggeredConferenceClientModule {
 public:
  Navigation(const int &, char **);
  Navigation(const Navigation &) = delete;
  Navigation &operator=(const Navigation &) = delete;
  virtual ~Navigation();
  virtual void nextContainer(odcore::data::Container &);

 private:

  static const double S_W_SIDE_DETECTION;
  static const double S_W_FRONT_DETECTION;
  static const double S_W_CLOSE_FRONT_DETECTION;
  static const double S_OUT_OF_RANGE;

  static const double T_REVERSE;
  static const double T_ROTATE_REVERSE;
  static const double T_TURN;



  static const int32_t E_FORWARD;
  static const int32_t E_REVERSE;
  static const int32_t E_ROTATE_RIGHT_L;
  static const int32_t E_ROTATE_RIGHT_R;
  static const int32_t E_ROTATE_LEFT_L;
  static const int32_t E_ROTATE_LEFT_R;
  static const int32_t E_STILL;
  static const int32_t E_DYN_TURN_SPEED;
  //static const uint32_t E_SEARCH;

  static const uint32_t UPDATE_FREQ;


  void setUp();
  void tearDown();
  virtual odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();
  void decodeResolveSensors();



  odcore::base::Mutex m_mutex;
  std::map<uint16_t, float> m_analogReadings;
  std::map<uint16_t, bool> m_gpioReadings;
  std::vector<uint16_t> m_gpioOutputPins;
  std::vector<uint16_t> m_pwmOutputPins;
  navigationState m_currentState;
  navigationState m_lastState;
  odcore::data::TimeStamp m_t_Current;
  odcore::data::TimeStamp m_t_Last;

 // double m_s_w_Front;
  bool m_s_w_FrontLeft;
  odcore::data::TimeStamp m_s_w_FrontLeft_t;
  bool m_s_w_FrontRight;
  odcore::data::TimeStamp m_s_w_FrontRight_t;
//  bool m_dynSpeedLeft;
//  bool m_dynSpeedRight;
  uint16_t m_updateCounter;
  bool m_debug;


};

}
}
}

#endif
