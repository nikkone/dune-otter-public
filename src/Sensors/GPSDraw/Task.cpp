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
// Author: Nikolai Lauvås ( Based on GPS task by Ricardo Martins)           *
//***************************************************************************
#define GPS_JSON_RESPONSE_MAX   10240

// ISO C++ 98 headers.
#include <cstring>
#include <algorithm>
#include <cstddef>

// libgps 
//#include <gps.h>

#include <libgpsmm.h>
#include <errno.h>

#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Sensors
{
  //! Device driver for NMEA messages through gpsd.
  namespace GPSDraw
  {
    //! Maximum number of initialization commands.
    static const unsigned c_max_init_cmds = 14;
    //! Timeout for waitReply() function.
    static const float c_wait_reply_tout = 4.0;
    //! Minimum number of fields of PUBX,00 sentence.
    static const unsigned c_pubx00_fields = 21;
    //! Minimum number of fields of GGA sentence.
    static const unsigned c_gga_fields = 15;
    //! Minimum number of fields of VTG sentence.
    static const unsigned c_vtg_fields = 9;
    //! Minimum number of fields of ZDA sentence.
    static const unsigned c_zda_fields = 7;
    //! Minimum number of fields of HDT sentence.
    static const unsigned c_hdt_fields = 3;
    //! Minimum number of fields of HDM sentence.
    static const unsigned c_hdm_fields = 3;
    //! Minimum number of fields of ROT sentence.
    static const unsigned c_rot_fields = 3;
    //! Minimum number of fields of PSATHPR sentence.
    static const unsigned c_psathpr_fields = 7;
    //! Minimum number of fields of ROT sentence.
    static const unsigned c_hev_fields = 2;
    //! Minimum number of fields of PASHR sentence.
    static const unsigned c_pashr_fields = 11;
        //! Minimum number of fields of PASHR sentence.
    static const unsigned c_psatfvi_fields = 31;

    //! Power on delay.
    static const double c_pwr_on_delay = 5.0;


    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! GPSD host
      std::string gpsd_host;
      //! GPSD port
      std::string gpsd_port;
      //! GPSD data timeout
      unsigned gpsd_data_timeout;
      //! Order of sentences.
      std::vector<std::string> stn_order;
    };

    struct Task: public Tasks::Task
    {
      //! GPS Fix message.
      IMC::GpsFix m_fix;
      //! Euler angles message.
      IMC::EulerAngles m_euler;
      //! Angular velocity message.
      IMC::AngularVelocity m_agvel;
      //! Heave message.
      IMC::Heave m_heave;
      //! Task arguments.
      Arguments m_args;
      //! Input watchdog.
      Time::Counter<float> m_wdog;
      //! True if we have angular velocity.
      bool m_has_agvel;
      //! True if we have heave.
      bool m_has_heave;
      //! True if we have euler angles.
      bool m_has_euler;
      //! Last initialization line read.
      std::string m_init_line;
      //! Buffer forEntityState
      char m_bufer_entity[64];

      struct gps_data_t m_gpsdata;
      char m_message_buffer[GPS_JSON_RESPONSE_MAX];

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx),
        m_has_agvel(false),
        m_has_heave(false),
        m_has_euler(false)
      {
        // Define configuration parameters.
        param("GPSD - Host", m_args.gpsd_host)
        .defaultValue("localhost")
        .description("");

        param("GPSD - Port", m_args.gpsd_port)
        .defaultValue("2947")
        .description("");

        param("GPSD - Data Timeout", m_args.gpsd_data_timeout)
        .defaultValue("500")
        .description("");
        // Initialize messages.
        clearMessages();

        param("Sentence Order", m_args.stn_order)
        .defaultValue("")
        .description("Sentence order");
      }

      void
      onResourceAcquisition(void)
      {
          if (gps_open(m_args.gpsd_host.c_str(), m_args.gpsd_port.c_str(), &m_gpsdata) == -1) {
              err("GPSD error code: %d, reason: %s\n", errno, gps_errstr(errno));
              throw RestartNeeded(DTR("Error opening GPSD connection, restarting"), 5);
          }
      }

      void
      onResourceRelease(void)
      {
        (void) gps_stream(&m_gpsdata, WATCH_DISABLE, NULL);
        (void) gps_close (&m_gpsdata);
      }

      void
      onResourceInitialization(void)
      {
        (void) gps_stream(&m_gpsdata, WATCH_ENABLE | WATCH_RAW, NULL);
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      void
      clearMessages(void)
      {
        m_euler.clear();
        m_agvel.clear();
        m_fix.clear();
      }
//////////////////////////////////////////////////////////////////////////////////

      //! Read time from string.
      //! @param[in] str string.
      //! @param[out] dst time.
      //! @return true if successful, false otherwise.
      bool
      readTime(const std::string& str, float& dst)
      {
        unsigned h = 0;
        unsigned m = 0;
        unsigned s = 0;
        double sfp = 0;

        if (std::sscanf(str.c_str(), "%02u%02u%lf", &h, &m, &sfp) != 3)
        {
          if (std::sscanf(str.c_str(), "%02u%02u%02u", &h, &m, &s) != 3)
            return false;
        }

        dst = (h * 3600) + (m * 60) + s + sfp;

        return true;
      }

      //! Read latitude from string.
      //! @param[in] str input string.
      //! @param[in] h either North (N) or South (S).
      //! @param[out] dst latitude.
      //! @return true if successful, false otherwise.
      bool
      readLatitude(const std::string& str, const std::string& h, double& dst)
      {
        int degrees = 0;
        double minutes = 0;

        if (std::sscanf(str.c_str(), "%02d%lf", &degrees, &minutes) != 2)
          return false;

        dst = Angles::convertDMSToDecimal(degrees, minutes);

        if (h == "S")
          dst = -dst;

        return true;
      }

      //! Read longitude from string.
      //! @param[in] str input string.
      //! @param[in] h either West (W) or East (E).
      //! @param[out] dst longitude.
      //! @return true if successful, false otherwise.
      double
      readLongitude(const std::string& str, const std::string& h, double& dst)
      {
        int degrees = 0;
        double minutes = 0;

        if (std::sscanf(str.c_str(), "%03d%lf", &degrees, &minutes) != 2)
          return false;

        dst = Angles::convertDMSToDecimal(degrees, minutes);

        if (h == "W")
          dst = -dst;

        return true;
      }

      //! Read decimal from input string.
      //! @param[in] str input string.
      //! @param[out] dst decimal.
      //! @return true if successful, false otherwise.
      template <typename T>
      bool
      readDecimal(const std::string& str, T& dst)
      {
        unsigned idx = 0;
        while (str[idx] == '0')
          ++idx;

        return castLexical(std::string(str, idx), dst);
      }

      //! Read number from input string.
      //! @param[in] str input string.
      //! @param[out] dst number.
      //! @return true if successful, false otherwise.
      template <typename T>
      bool
      readNumber(const std::string& str, T& dst)
      {
        return castLexical(str, dst);
      }

      //! Process sentence.
      //! @param[in] line line.
      void
      processSentence(const std::string& line)
      {
        // Discard leading noise.
        size_t sidx = 0;
        for (sidx = 0; sidx < line.size(); ++sidx)
        {
          if (line[sidx] == '$')
            break;
        }

        // Discard trailing noise.
        size_t eidx = 0;
        for (eidx = line.size() - 1; eidx > sidx; --eidx)
        {
          if (line[eidx] == '*')
            break;
        }

        if (sidx >= eidx)
          return;

        // Compute checksum.
        uint8_t ccsum = 0;
        for (size_t i = sidx + 1; i < eidx; ++i)
          ccsum ^= line[i];

        // Validate checksum.
        unsigned rcsum = 0;
        if (std::sscanf(&line[0] + eidx + 1, "%02X", &rcsum) != 1)
        {
          trace("No checksum found, will not parse sentence.");
          return;
        }

        if (ccsum != rcsum)
        {
          trace("Checksum field does not match computed checksum, will not "
                "parse sentence.");
          return;
        }

        // Split sentence
        std::vector<std::string> parts;
        String::split(line.substr(sidx + 1, eidx - sidx - 1), ",", parts);

        if (std::find(m_args.stn_order.begin(), m_args.stn_order.end(), parts[0]) != m_args.stn_order.end())
          interpretSentence(parts);
      }


      //! Interpret given sentence.
      //! @param[in] parts vector of strings from sentence.
      void
      interpretSentence(std::vector<std::string>& parts)
      {
        if (parts[0] == m_args.stn_order.front())
        {
          clearMessages();
          m_fix.setTimeStamp();
          m_euler.setTimeStamp(m_fix.getTimeStamp());
          m_agvel.setTimeStamp(m_fix.getTimeStamp());
          m_heave.setTimeStamp(m_fix.getTimeStamp());
        }

        if (hasNMEAMessageCode(parts[0], "ZDA"))
        {
          interpretZDA(parts);
        }
        else if (hasNMEAMessageCode(parts[0], "GGA"))
        {
          interpretGGA(parts);
        }
        else if (hasNMEAMessageCode(parts[0], "VTG"))
        {
          interpretVTG(parts);
        }
        else if (parts[0] == "PSAT")
        {
          if (parts[1] == "HPR")
            interpretPSATHPR(parts);
        }
        else if (parts[0] == "PASHR")
        {
          interpretPASHR(parts);
        }
        else if (parts[0] == "PUBX")
        {
          if (parts[1] == "00")
            interpretPUBX00(parts);
        }
        else if (hasNMEAMessageCode(parts[0], "HDM"))
        {
          interpretHDM(parts);
        }
        else if (hasNMEAMessageCode(parts[0], "HDT"))
        {
          interpretHDT(parts);
        }
        else if (hasNMEAMessageCode(parts[0], "ROT"))
        {
          interpretROT(parts);
        }
        else if (hasNMEAMessageCode(parts[0], "HEV"))
        {
          interpretHEV(parts);
        }

        if (parts[0] == m_args.stn_order.back())
        {
          m_wdog.reset();
          dispatch(m_fix);

          if (m_has_euler)
          {
            dispatch(m_euler);
            m_has_euler = false;
          }
          
          if (m_has_heave)
          {
            dispatch(m_heave);
            m_has_heave = false;
          }
          
          if (m_has_agvel)
          {
            dispatch(m_agvel);
            m_has_agvel = false;
          }

          std::memset(&m_bufer_entity, '\0', sizeof(m_bufer_entity));
          if (m_fix.validity & IMC::GpsFix::GFV_VALID_POS)
          {
            std::sprintf(m_bufer_entity, "active - hdop: %.2f , Sat: %d", m_fix.hdop, m_fix.satellites);
            setEntityState(IMC::EntityState::ESTA_NORMAL, Utils::String::str(DTR(m_bufer_entity)));
          }
          else
          {
            std::sprintf(m_bufer_entity, "wait gps fix - hdop: %.2f , Sat: %d", m_fix.hdop, m_fix.satellites);
            setEntityState(IMC::EntityState::ESTA_NORMAL, Utils::String::str(DTR(m_bufer_entity)));
          }
          //spew("Messages Sent.");
        }
      }

      bool
      hasNMEAMessageCode(const std::string& str, const std::string& code)
      {
        return String::startsWith(str, "G") && String::endsWith(str, code);
      }

      //! Interpret HEV sentence (rate of turn).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretHEV(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_hev_fields)
        {
          war(DTR("invalid HEV sentence"));
          return;
        }

        if (readNumber(parts[1], m_heave.value))
        {
          m_has_heave = true;
        }
      }

      //! Interpret ZDA sentence (UTC date and time).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretZDA(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_zda_fields)
        {
          war(DTR("invalid ZDA sentence"));
          return;
        }

        // Read time.
        if (readTime(parts[1], m_fix.utc_time))
          m_fix.validity |= IMC::GpsFix::GFV_VALID_TIME;

        // Read date.
        if (readDecimal(parts[2], m_fix.utc_day)
            && readDecimal(parts[3], m_fix.utc_month)
            && readDecimal(parts[4], m_fix.utc_year))
        {
          m_fix.validity |= IMC::GpsFix::GFV_VALID_DATE;
        }
      }

      //! Interpret GGA sentence (GPS fix data).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretGGA(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_gga_fields)
        {
          war(DTR("invalid GGA sentence"));
          return;
        }

        int quality = 0;
        readDecimal(parts[6], quality);
        if (quality == 1)
        {
          m_fix.type = IMC::GpsFix::GFT_STANDALONE;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }
        else if (quality == 2)
        {
          m_fix.type = IMC::GpsFix::GFT_DIFFERENTIAL;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }

        if (readLatitude(parts[2], parts[3], m_fix.lat)
            && readLongitude(parts[4], parts[5], m_fix.lon)
            && readNumber(parts[9], m_fix.height)
            && readDecimal(parts[7], m_fix.satellites))
        {
          // Convert altitude above sea level to altitude above ellipsoid.
          double geoid_sep = 0;
          if (readNumber(parts[11], geoid_sep))
            m_fix.height += geoid_sep;

          // Convert coordinates to radians.
          m_fix.lat = Angles::radians(m_fix.lat);
          m_fix.lon = Angles::radians(m_fix.lon);
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }
        else
        {
          m_fix.validity &= ~IMC::GpsFix::GFV_VALID_POS;
        }

        if (readNumber(parts[8], m_fix.hdop))
          m_fix.validity |= IMC::GpsFix::GFV_VALID_HDOP;
      }

      //! Interpret PUBX00 sentence (navstar position).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretPUBX00(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_pubx00_fields)
        {
          war(DTR("invalid PUBX,00 sentence"));
          return;
        }

        if (parts[8] == "G3" || parts[8] == "G2")
        {
          m_fix.type = IMC::GpsFix::GFT_STANDALONE;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }
        else if (parts[8] == "D3" || parts[8] == "D2")
        {
          m_fix.type = IMC::GpsFix::GFT_DIFFERENTIAL;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }

        if (readLatitude(parts[3], parts[4], m_fix.lat)
            && readLongitude(parts[5], parts[6], m_fix.lon)
            && readNumber(parts[7], m_fix.height)
            && readDecimal(parts[18], m_fix.satellites))
        {
          // Convert coordinates to radians.
          m_fix.lat = Angles::radians(m_fix.lat);
          m_fix.lon = Angles::radians(m_fix.lon);
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }
        else
        {
          m_fix.validity &= ~IMC::GpsFix::GFV_VALID_POS;
        }

        if (readNumber(parts[9], m_fix.hacc))
          m_fix.validity |= IMC::GpsFix::GFV_VALID_HACC;

        if (readNumber(parts[10], m_fix.vacc))
          m_fix.validity |= IMC::GpsFix::GFV_VALID_VACC;

        if (readNumber(parts[15], m_fix.hdop))
          m_fix.validity |= IMC::GpsFix::GFV_VALID_HDOP;

        if (readNumber(parts[16], m_fix.vdop))
          m_fix.validity |= IMC::GpsFix::GFV_VALID_VDOP;
      }

      //! Interpret VTG sentence (course over ground).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretVTG(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_vtg_fields)
        {
          war(DTR("invalid VTG sentence"));
          return;
        }

        if (readNumber(parts[1], m_fix.cog))
        {
          m_fix.cog = Angles::normalizeRadian(Angles::radians(m_fix.cog));
          m_fix.validity |= IMC::GpsFix::GFV_VALID_COG;
        }

        if (readNumber(parts[7], m_fix.sog))
        {
          m_fix.sog *= 1000.0f / 3600.0f;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_SOG;
        }
      }

      //! Interpret HDT sentence (true heading).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretHDT(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_hdt_fields)
        {
          war(DTR("invalid HDT sentence"));
          return;
        }

        if (readNumber(parts[1], m_euler.psi))
          m_euler.psi = Angles::normalizeRadian(Angles::radians(m_euler.psi));
      }

      //! Interpret HDM sentence (Magnetic heading of
      //! the vessel derived from the true heading calculated).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretHDM(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_hdm_fields)
        {
          war(DTR("invalid HDM sentence"));
          return;
        }

        if (readNumber(parts[1], m_euler.psi_magnetic))
        {
          m_euler.psi_magnetic = Angles::normalizeRadian(Angles::radians(m_euler.psi_magnetic));
          m_has_euler = true;
        }
      }

      //! Interpret ROT sentence (rate of turn).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretROT(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_rot_fields)
        {
          war(DTR("invalid ROT sentence"));
          return;
        }

        if (readNumber(parts[1], m_agvel.z))
        {
          m_agvel.z = Angles::radians(m_agvel.z) / 60.0;
          m_has_agvel = true;
        }
      }

      //! Interpret PSATHPR sentence (Proprietary NMEA message that
      //! provides the heading, pitch, roll, and time in a single message).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretPSATHPR(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_psathpr_fields)
        {
          war(DTR("invalid PSATHPR sentence"));
          return;
        }
        // Read time.
        readNumber(parts[2], m_euler.time);

        if (readNumber(parts[3], m_euler.psi))
        {
          m_euler.psi = Angles::normalizeRadian(Angles::radians(m_euler.psi));
          m_has_euler = true;
        }

        if (readNumber(parts[4], m_euler.theta))
        {
          m_euler.theta = Angles::normalizeRadian(Angles::radians(m_euler.theta));
          m_has_euler = true;
        }

        if (readNumber(parts[5], m_euler.phi))
        {
          m_euler.phi = Angles::normalizeRadian(Angles::radians(m_euler.phi));
          m_has_euler = true;
        }
      }
/*
      //! Interpret PSATFVI sentence (Proprietary NMEA message that
      //! provides the heading, pitch, roll, and time in a single message).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretPSATFVI(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_psatfvi_fields)
        {
          war(DTR("invalid PSATFVI sentence"));
          return;
        }
        // Read time.
        readNumber(parts[2], m_euler.time);

        if (readNumber(parts[9], m_euler.psi))
        {
          m_euler.psi = Angles::normalizeRadian(Angles::radians(m_euler.psi));
          m_has_euler = true;
        }

        if (readNumber(parts[11], m_euler.theta))
        {
          m_euler.theta = Angles::normalizeRadian(Angles::radians(m_euler.theta));
          m_has_euler = true;
        }

        if (readNumber(parts[13], m_euler.phi))
        {
          m_euler.phi = Angles::normalizeRadian(Angles::radians(m_euler.phi));
          m_has_euler = true;
        }

        int quality = 0;
        readDecimal(parts[27], quality);
        if (quality == 1)
        {
          m_fix.type = IMC::GpsFix::GFT_STANDALONE;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }
        else if (quality == 2)
        {
          m_fix.type = IMC::GpsFix::GFT_DIFFERENTIAL;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }


        if()
        if (readLatitude(parts[2], parts[3], m_fix.lat)
            && readLongitude(parts[4], parts[5], m_fix.lon)
            && readNumber(parts[9], m_fix.height)
            && readDecimal(parts[7], m_fix.satellites))
        {
          // Convert altitude above sea level to altitude above ellipsoid.
          double geoid_sep = 0;
          if (readNumber(parts[24], geoid_sep))
            m_fix.height += geoid_sep;

          // Convert coordinates to radians.
          m_fix.lat = Angles::radians(m_fix.lat);
          m_fix.lon = Angles::radians(m_fix.lon);
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }
        else
        {
          m_fix.validity &= ~IMC::GpsFix::GFV_VALID_POS;
        }

      }
*/
      //! Interpret PASHR sentence (Proprietary NMEA message that
      //! provides the heave(and standard deviation),heading, pitch(and standard deviation), roll(and standard deviation), and time in a single message).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretPASHR(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_pashr_fields)
        {
          war(DTR("invalid PASHR sentence"));
          return;
        }
        // Read time.
        readNumber(parts[1], m_euler.time);

        if (readNumber(parts[2], m_euler.psi))
        {
          m_euler.psi = Angles::normalizeRadian(Angles::radians(m_euler.psi));
          m_has_euler = true;
        }

        if (readNumber(parts[5], m_euler.theta))
        {
          m_euler.theta = Angles::normalizeRadian(Angles::radians(m_euler.theta));
          m_has_euler = true;
        }

        if (readNumber(parts[4], m_euler.phi))
        {
          m_euler.phi = Angles::normalizeRadian(Angles::radians(m_euler.phi));
          m_has_euler = true;
        }
        if (readNumber(parts[6], m_heave.value))
        {
          m_has_heave = true;
        }
      }


//////////////////////////////////////////////////////////////////////////////////

      void
      onMain(void)
      {
        while (!stopping())
        {

          consumeMessages(); // Needed because task status querys are consumed (inherrited from tasks)
          if (gps_waiting (&m_gpsdata, 500)) {
            spew("After gps_waiting");
            *m_message_buffer = '\0';
            if (gps_read (&m_gpsdata, m_message_buffer, sizeof(m_message_buffer)) != -1) {
              spew("After gps_read");
              std::string line(m_message_buffer);
              processSentence(line);
              spew("Line processed: %s", line.c_str());
            } else {
              spew("Could not read GPSD m_gpsdata");
            }
          }

        }
      }
    };
  }
}

DUNE_TASK
