//***************************************************************************
// Copyright 2013-2021 Norwegian University of Science and Technology (NTNU)*
// Department of Engineering Cybernetics (ITK)                              *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Nikolai Lauvås (based on GPS by Ricardo Martins)                 *
//***************************************************************************
 

//TODO: Add feature: position at 5s before tag registration in imc message.
//TODO: May not work without PPS anymore.

// ISO C++ 98 headers.
#include <cstring>
#include <algorithm>
#include <cstddef>
#include <ctime> /* time_t, struct tm, time, mktime */
#include <string>
#include <sstream>
#include <chrono>

// DUNE headers.
#include <DUNE/DUNE.hpp>
//#include <DUNE/Time/Clock.hpp>

namespace Simulators
{
  //! Device driver for ThelmaHydrophone
  namespace RemoteSensorSender
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {

      //! Sync Period;
      double sync_period;
      double lat;
      double lon;
      double alt;
      std::string id;
      std::string data;

    };

    struct Task: public DUNE::Tasks::Task
    {
      IMC::RemoteSensorInfo tagPosition;
      Arguments m_args;
      //! Timer.
      Time::Counter<float> m_sync_timer;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Sync Period", m_args.sync_period)
        .units(Units::Second)
        .defaultValue("10.0")
        .minimumValue("0.0")
        .description("Period between sync messages");


        param("Latitude", m_args.lat)
        .defaultValue("63.33")
        .minimumValue("0.0")
        .description("Period between sync messages");

        param("Longtitude", m_args.lon)
        .defaultValue("10.083333")
        .minimumValue("0.0")
        .description("Period between sync messages");

        param("Altitude", m_args.alt)
        .defaultValue("10.0")
        .minimumValue("0.0")
        .description("Period between sync messages");

        param("Data", m_args.data)
        .defaultValue("10.0")
        .minimumValue("0.0")
        .description("Period between sync messages");

        param("ID", m_args.id)
        .defaultValue("Fish_position_est_1")
        .minimumValue("0.0")
        .description("Period between sync messages");

        bind<IMC::GpsFix>(this);
      }
      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.sync_period))
          m_sync_timer.setTop(m_args.sync_period);
      }
      void
      onResourceAcquisition(void)
      {

      }


      void
      onResourceRelease(void) {
        
      }

      void
      onResourceInitialization(void)
      {
      }

      void
      consume(const IMC::GpsFix* msg)
      {
        m_args.lat = DUNE::Math::Angles::degrees(msg->lat);
        m_args.lon = DUNE::Math::Angles::degrees(msg->lon);
      }
      void sendRemoteSensorInfo() {
        tagPosition.lat = DUNE::Math::Angles::radians(m_args.lat);
        tagPosition.lon = DUNE::Math::Angles::radians(m_args.lon);
        tagPosition.alt = m_args.alt;
        tagPosition.id = m_args.id;
        tagPosition.data = m_args.data;

        dispatch(tagPosition);
      }


      void
      onMain(void)
      {
        while(!stopping()) {

          if(m_sync_timer.overflow())
          {
            m_sync_timer.reset();
            sendRemoteSensorInfo();
            debug("Sent");
          }
          //consumeMessages();
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
