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

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>
#include <opendavinci/odcore/data/Container.h>

#include <odvdminiature/GeneratedHeaders_ODVDMiniature.h>

#include "Lps.h"

namespace opendlv {
namespace proxy {
namespace miniature {

Lps::Lps(int32_t const &argc, char **argv)
    : DataTriggeredConferenceClientModule(argc, argv, "proxy-miniature-lps")
    , m_needleMarkerDistances()
    , m_needleNormRoll()
    , m_needleNormPitch()
    , m_needleNormYaw()
    , m_searchMarginHalf()
    , m_frameId()
{
}

Lps::~Lps() 
{
}

void Lps::setUp() 
{
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();

  m_searchMarginHalf = 0.5f * 
    kv.getValue<float>("proxy-miniature-lps.searchMargin");
  m_frameId = kv.getValue<uint16_t>("proxy-miniature-lps.frameId");

   
  float x0 = kv.getValue<float>("proxy-miniature-lps.origoMarkerX");
  float y0 = kv.getValue<float>("proxy-miniature-lps.origoMarkerY");
  float z0 = kv.getValue<float>("proxy-miniature-lps.origoMarkerZ");
  opendlv::model::Cartesian3 origoMarker(x0, y0, z0);

  float x1 = kv.getValue<float>("proxy-miniature-lps.forwardMarkerX");
  float y1 = kv.getValue<float>("proxy-miniature-lps.forwardMarkerY");
  float z1 = kv.getValue<float>("proxy-miniature-lps.forwardMarkerZ");
  opendlv::model::Cartesian3 forwardMarker(x1, y1, z1);

  float x2 = kv.getValue<float>("proxy-miniature-lps.leftwardMarkerX");
  float y2 = kv.getValue<float>("proxy-miniature-lps.leftwardMarkerY");
  float z2 = kv.getValue<float>("proxy-miniature-lps.leftwardMarkerZ");
  opendlv::model::Cartesian3 leftwardMarker(x2, y2, z2);

  std::vector<opendlv::model::Cartesian3> needleMarkers;
  needleMarkers.push_back(origoMarker);
  needleMarkers.push_back(forwardMarker);
  needleMarkers.push_back(leftwardMarker);

  AnalyseNeedle(needleMarkers);
}

void Lps::tearDown() 
{
}

void Lps::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::proxy::QtmFrame::ID()) {
    opendlv::proxy::QtmFrame qtmFrame = 
        a_container.getData<opendlv::proxy::QtmFrame>();
    
    std::vector<opendlv::model::Cartesian3> markers = qtmFrame.getListOfMarkers();
    Search(markers);
  }
}

void Lps::AnalyseNeedle(std::vector<opendlv::model::Cartesian3> a_needleMarkers)
{
  uint8_t const markerCount = a_needleMarkers.size();

  float rollTotal = 0.0f;
  float pitchTotal = 0.0f;
  float yawTotal = 0.0f;

  for (auto marker : a_needleMarkers) {
    float x = marker.getX();
    float y = marker.getY();
    float z = marker.getZ();
    float distance = sqrt(x*x + y*y + z*z);

    m_needleMarkerDistances.push_back(distance);

    float roll = atan2(y, z);
    float pitch = atan2(z, x);
    float yaw = atan2(y, x);

    rollTotal += roll;
    pitchTotal += pitch;
    yawTotal += yaw;
  }

  m_needleNormRoll = rollTotal / markerCount;
  m_needleNormPitch = pitchTotal / markerCount;
  m_needleNormYaw = yawTotal / markerCount;
}

void Lps::Search(std::vector<opendlv::model::Cartesian3> a_haystackMarkers)
{
  uint32_t const haystackMarkerCount = a_haystackMarkers.size();

//  std::cout << "==== SEARCH" << std::endl;

  for (uint32_t i = 0; i < haystackMarkerCount; i++) {

//    std::cout << "I THINK ORIGO IS: " << i << std::endl;

    opendlv::model::Cartesian3 origoCandidate = a_haystackMarkers[i];

    uint32_t const needleMarkerCount = m_needleMarkerDistances.size();
    std::vector<float> foundDistances(needleMarkerCount);
    std::vector<uint32_t> foundIndices(needleMarkerCount);
    for (uint32_t j = 0; j < needleMarkerCount; j++) {
      foundDistances[j] = std::numeric_limits<float>::max();
      foundIndices[j] = -1;
    }

    for (uint32_t j = 0; j < needleMarkerCount; j++) {
      float searchedDistance = m_needleMarkerDistances[j];

//      std::cout << " SEARCH FOR DISTANCE: " << searchedDistance << std::endl;

      for (uint32_t k = 0; k < haystackMarkerCount; k++) {
        if (k == i) {
          continue;
        }

        opendlv::model::Cartesian3 candidate = a_haystackMarkers[k];
        
        float dx = candidate.getX() - origoCandidate.getX();
        float dy = candidate.getY() - origoCandidate.getY();
        float dz = candidate.getZ() - origoCandidate.getZ();
      
        float distance = sqrt(dx*dx + dy*dy + dz*dz);

   //     std::cout << "  IS IT: " << dx << " " << dy << " = " << distance << "?" << std::endl;

        if (searchedDistance > distance - m_searchMarginHalf &&
            searchedDistance < distance + m_searchMarginHalf)
        {
          float prevFoundDistance = foundDistances[j];
          float currentError = std::abs(distance - searchedDistance);
          float prevError = std::abs(prevFoundDistance - searchedDistance);

          if (currentError < prevError) {
            foundDistances[j] = distance;
            foundIndices[j] = k;
//            std::cout << "   YES, " << searchedDistance << " == " << distance << std::endl;
          }
        }
      }

      std::vector<opendlv::model::Cartesian3> needleMarkers;
      needleMarkers.push_back(origoCandidate);

      bool isNeedleFound = true;
      for (int index : foundIndices) {
        if (index == -1) {
          isNeedleFound = false;
          break;
        }
        opendlv::model::Cartesian3 candidate = a_haystackMarkers[index];
        needleMarkers.push_back(candidate);
      }

      // TODO: Check that we don't use the same node twice..

      if (isNeedleFound) {
        std::cout << "    FOUND!!!" << std::endl;
        FindState(needleMarkers);
      }
    }
  }
//  std::cout << "    NOT FOUND!!!" << std::endl;
}

void Lps::FindState(std::vector<opendlv::model::Cartesian3> a_needleMarkers)
{
//  std::cout << "== OBJECT " << a_scene_object->GetName() << std::endl;

  opendlv::model::Cartesian3 origo = a_needleMarkers[0];
  float const x0 = origo.getX();
  float const y0 = origo.getY();
  float const z0 = origo.getZ();
    
//  std::cout << "  origo " << x << " " << y << std::endl;

  uint32_t const haystackMarkerCount = a_needleMarkers.size();

  bool doFlip = true;

  float rollTotal = 0.0f;
  float pitchTotal = 0.0f;
  float yawTotal = 0.0f;

  for (uint32_t i = 1; i < haystackMarkerCount; i++) {
    opendlv::model::Cartesian3 marker = a_needleMarkers[i];
    
    float dx = marker.getX() - x0;
    float dy = marker.getY() - y0;
    float dz = marker.getZ() - z0;

    if (dx > 0.0f) {
      doFlip = false;
    }

    float roll = atan2(dy, dz);
    float pitch = atan2(dz, dx);
    float yaw = atan2(dy, dx);
    
 //   std::cout << "  node " << i << " x " << marker->x << " y " << marker->y << " dx "
 //       << dx << " dy " << dy << " yaw " << yaw << std::endl;

    rollTotal += roll;
    pitchTotal += pitch;
    yawTotal += yaw;
  }

  float const rollMean = yawTotal / (haystackMarkerCount - 1);
  float const pitchMean = yawTotal / (haystackMarkerCount - 1);
  float const yawMean = yawTotal / (haystackMarkerCount - 1);
  
  float const roll = rollMean - m_needleNormRoll;
  float const pitch = pitchMean - m_needleNormPitch;
  float yaw = yawMean - m_needleNormYaw;

  if (doFlip) {
    yaw += 3.14f;
  }

//  std::cout << "  yaw norm " << yaw_norm << " total yaw " << yaw_tot << " yaw mean " << yaw_mean << " final yaw " << yaw << std::endl;

  opendlv::model::Cartesian3 position(x0, y0, z0);
  opendlv::model::Cartesian3 angularDisplacement(roll, pitch, yaw);
  opendlv::model::State state(position, angularDisplacement, m_frameId);

  odcore::data::Container c(state);
  getConference().send(c);
}

}
}
}
