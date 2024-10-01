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
// Author: Nikolai Lauvås                                                   *
//***************************************************************************
#define MODE_STR_NUM 4

// ISO C++ 98 headers.
#include <cstring>
#include <algorithm>
#include <cstddef>

// libgps 
//#include <gps.h>

#include <libgpsmm.h>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Sensors
{
  //! Device driver for NMEA capable %GPS devices.
  namespace GPSD
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;

    };

    struct Task: public Tasks::Task
    {
      //! GPS Fix message.
      IMC::GpsFix m_fix;
      //! Euler angles message.
      IMC::EulerAngles m_euler;
      //! Angular velocity message.
      IMC::AngularVelocity m_agvel;
      //! Task arguments.
      Arguments m_args;
      //! Input watchdog.
      Time::Counter<float> m_wdog;
      //! True if we have angular velocity.
      bool m_has_agvel;
      //! True if we have euler angles.
      bool m_has_euler;
      //! Last initialization line read.
      std::string m_init_line;
      //! Buffer forEntityState
      char m_bufer_entity[64];

      //struct gps_data_t gps_data;
      gpsmm *gps_rec;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx),
        m_has_agvel(false),
        m_has_euler(false),
        gps_rec(nullptr)
      {
        // Define configuration parameters.
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("")
        .description("Serial port device used to communicate with the sensor");

        // Initialize messages.
        clearMessages();

      }



      void
      onResourceAcquisition(void)
      {
        inf("Before");
        gps_rec = new gpsmm("localhost", DEFAULT_GPSD_PORT);
        inf("Between");
        if (gps_rec->stream(WATCH_ENABLE | WATCH_JSON) == nullptr) {
          err("No GPSD running.");
        }
        inf("After");
        setEntityState(IMC::EntityState::ESTA_NORMAL, "");
      }

      void
      onResourceRelease(void)
      {
        if(gps_rec != nullptr) {
          gps_rec->stream(WATCH_DISABLE);
          Memory::clear(gps_rec);
        }

      }

      void
      onResourceInitialization(void)
      {

      }

      void
      clearMessages(void)
      {
        m_euler.clear();
        m_agvel.clear();
        m_fix.clear();
      }

      void
      onMain(void)
      {
        while (!stopping())
        {



  constexpr auto kWaitingTime{1000000};  // 1000000 is 1 second

    if (!gps_rec->waiting(kWaitingTime)) {
      continue;
    }

    struct gps_data_t* gpsd_data;

    if ((gps_rec->read()) == nullptr) {
      std::cerr << "GPSD read error.\n";
      //return 1;
    }

    while (((gpsd_data = gps_rec->read()) == nullptr) || (gpsd_data->fix.mode < MODE_2D)) {
      // Do nothing until fix, block execution for 1 second (busy wait mitigation)
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    int status = gpsd_data->status;
    if (status == 1)
      {
        m_fix.type = IMC::GpsFix::GFT_STANDALONE;
        m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
      }
    else if (status == 2)
      {
        m_fix.type = IMC::GpsFix::GFT_DIFFERENTIAL;
        m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
      }
    else if (status == 5)
      {
        m_fix.type = IMC::GpsFix::GFT_DEAD_RECKONING;
        m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
      }
    else if (status == 7)
      {
        //m_fix.type = IMC::GpsFix::GFT_DIFFERENTIAL;
        m_fix.validity |= IMC::GpsFix::GFV_VALID_DATE;
        m_fix.validity |= IMC::GpsFix::GFV_VALID_TIME;
      }
    else if (status == 8)
      {
        m_fix.type = IMC::GpsFix::GFT_SIMULATION;
        m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
      }
    //m_fix.type = gpsd_data->status;
    tm *gmtm = gmtime(&gpsd_data->fix.time.tv_sec);
    m_fix.utc_year = 1900 + gmtm->tm_year;
    m_fix.utc_month = gmtm->tm_mon;
    m_fix.utc_day = gmtm->tm_mday;
    m_fix.utc_time = gpsd_data->fix.time.tv_sec;
    m_fix.lat = Angles::radians(gpsd_data->fix.latitude);
    m_fix.lon = Angles::radians(gpsd_data->fix.longitude);
    m_fix.height = gpsd_data->fix.altHAE;
    m_fix.satellites = gpsd_data->satellites_used;
    m_fix.cog = gpsd_data->fix.track;
    m_fix.sog = gpsd_data->fix.speed;
    m_fix.hdop = gpsd_data->dop.hdop;
    m_fix.vdop = gpsd_data->dop.vdop;
    m_fix.hacc = gpsd_data->fix.eph;
    m_fix.vacc = gpsd_data->fix.epv;
    dispatch(m_fix);

    m_euler.psi = Angles::normalizeRadian(Angles::radians(gpsd_data->attitude.yaw)); // Or gpsd_data->attitude.heading, confusing
    m_euler.theta = Angles::normalizeRadian(Angles::radians(gpsd_data->attitude.pitch));
    m_euler.phi = Angles::normalizeRadian(Angles::radians(gpsd_data->attitude.roll));
    m_euler.psi_magnetic = Angles::radians(gpsd_data->fix.magnetic_track);
    dispatch(m_euler);

    const auto latitude{gpsd_data->fix.latitude};
    const auto longitude{gpsd_data->fix.longitude};
    const auto hdop{gpsd_data->dop.hdop};
    const auto vdop{gpsd_data->dop.vdop};
    const auto pdop{gpsd_data->dop.pdop};
    const auto s_vis{gpsd_data->satellites_visible};
    const auto s_used{gpsd_data->satellites_used};
    const auto time_str{gpsd_data->fix.time.tv_sec};

    std::cout << std::setprecision(8) << std::fixed;  // set output to fixed floating point, 8 decimal precision
    std::cout << time_str << "," << latitude << "," << longitude << "," << hdop << "," << vdop << "," << pdop << "," << s_vis << "," << s_used << '\n';
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
