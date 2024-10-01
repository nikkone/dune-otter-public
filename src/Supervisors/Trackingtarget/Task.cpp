//***************************************************************************
// Copyright 2007-2020 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
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

// DUNE headers.
#include <DUNE/DUNE.hpp>

// SQLITE3 headers.
#include <sqlite3/sqlite3.h>

namespace Supervisors
{
  //! This task automatically starts the formation tracking for given estimators
  //! @author Nikolai Lauvås
  namespace Trackingtarget
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {


      //! Sync Period;
      double timeout;
      //! Vector with the names of all estimators to consider folowing
      std::vector<std::string> monitoredEstimators;
      //! Only used if the db is not used
      double fishtag_min_interval;
      //! Only used if the db is not used
      double fishtag_max_interval;
      //! Location of the sqlite dbfile containing information on the tagged fish
      std::string taglistDBpath;

      //! 
      double ref_send_interval;
      //!
      double formation_rotation_step;
      //!
      double ref_timeout;

      //! Minimum Speed.
      fp32_t minspeed;
      //! Maximum Speed.
      fp32_t maxspeed;
      //! Minimum Radius.
      fp32_t minradius;
      //! Maximum Radius.
      fp32_t maxradius;
      //! Formation Participants.
      std::string participants;
      //! Position filter prefix
      std::string positionFilterPrefix;

      bool enableToggle;

      bool keepFormation;
    };
    /// @brief 
    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
        //! Timer that is reset when a RemoteSensorInfo for the following entity is received. Top is m_minTagInterval
        Time::Counter<float> m_last_tag_timer;
        //!
        IMC::RemoteSensorInfo m_last_rs_msg;

        std::set<std::string> m_monitoredEstimators;

        IMC::otterFormation m_of_msg;

        bool m_formation_started;

        std::string m_tracked_id;
        //! Database containing tag information
        sqlite3 *m_db;

        bool m_db_used;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_formation_started(false),
        m_db_used(false)
      {

        param("Enable toggle", m_args.enableToggle)
        .defaultValue("true")
        .description("Activate sending.");

        param("Timeout Formation Keep", m_args.keepFormation)
        .defaultValue("true")
        .description("Keep Formation After Timeout, else formation is stopped");

        param("Formation Timeout", m_args.timeout)
            .units(Units::Second)
            .defaultValue("600.0")
            .minimumValue("0.0")
            .description("Period after which the tracker considers a fish tag lost");

        param("Taglist DB Path", m_args.taglistDBpath)
        .defaultValue("")
        .description("Path for DB with taglist.");

        param("Position Filter Prefix", m_args.positionFilterPrefix)
        .defaultValue("MultiReceiverXKF")
        .description("Position filter prefix.");


        param("Minimum Speed", m_args.minspeed)
        .units(Units::MeterPerSecond)
        .defaultValue("0.5")
        .description("Minimum Speed");

        param("Maximum Speed", m_args.maxspeed)
        .units(Units::MeterPerSecond)
        .defaultValue("1")
        .description("Maximum Speed.");

        param("Minimum Radius", m_args.minradius)
        .units(Units::Meter)
        .defaultValue("30")
        .description("Minimum Radius");

        param("Maximum Radius", m_args.maxradius)
        .units(Units::Meter)
        .defaultValue("60")
        .description("Maximum Radius.");

        param("Participants", m_args.participants)
        .defaultValue("")
        .description("Participants.");

        param("Formation Ref Interval", m_args.ref_send_interval)
        .units(Units::Second)
        .defaultValue("5.0")
        .minimumValue("0.0")
        .description("Participants.");

        param("Formation Rotation Step", m_args.formation_rotation_step)
        .defaultValue("0.0")
        .description("Participants.");

        param("Fotmation Ref Timeout", m_args.ref_timeout)
        .defaultValue("60")
        .description("Participants.");

// Only used in non-db mode
        param("Monitored Estimators", m_args.monitoredEstimators)
        .defaultValue("MultiReceiverEKF36,Fish_position_est_1")
        .description("Activate sending.");

        param("FishTag min interval", m_args.fishtag_min_interval)
            .units(Units::Second)
            .defaultValue("30.0")
            .minimumValue("0.0")
            .description("Period between sync messages");

        param("FishTag max interval", m_args.fishtag_max_interval)
            .units(Units::Second)
            .defaultValue("90.0")
            .minimumValue("0.0")
            .description("Period between sync messages");
        // Initialize entity state.
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        bind<IMC::RemoteSensorInfo>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.timeout)) 
          m_last_tag_timer.setTop(m_args.timeout);
        if(paramChanged(m_args.minspeed))
          m_of_msg.minspeed = m_args.minspeed; 
        if(paramChanged(m_args.maxspeed))
          m_of_msg.maxspeed = m_args.maxspeed; 
        if(paramChanged(m_args.minradius))
          m_of_msg.minradius = m_args.minradius; 
        if(paramChanged(m_args.maxradius))
          m_of_msg.maxradius = m_args.maxradius; 
        if(paramChanged(m_args.participants))
          m_of_msg.participants = m_args.participants; 

        if(paramChanged(m_args.monitoredEstimators)) {
          m_monitoredEstimators.clear();
          for(auto &estimate : m_args.monitoredEstimators) {
            m_monitoredEstimators.insert(estimate);
          }
        }
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        if(!m_args.taglistDBpath.empty()) {
          if(sqlite3_open_v2(m_args.taglistDBpath.c_str(), &m_db,SQLITE_OPEN_READONLY,0) != SQLITE_OK){
            err("Can't open database: %s\n", sqlite3_errmsg(m_db));
            sqlite3_close(m_db);
          } else {
            m_db_used = true;
          }
        } else {
          war("Not using database for tag information");
        }
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        m_of_msg.speed_units = IMC::SUNITS_METERS_PS;
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        sqlite3_close(m_db);
      }

      void sendFormationStart(const std::string &id) {
        m_of_msg.target = id;
        m_of_msg.msg_type = IMC::otterFormation::MessageTypeEnum::T_start;
        m_formation_started = true;
        m_tracked_id = m_of_msg.target;
        dispatch(m_of_msg);
      }

      void sendFormationStop() {
        m_of_msg.msg_type = IMC::otterFormation::MessageTypeEnum::T_stop;
        m_formation_started = false;
        dispatch(m_of_msg);
      }

      void consume(const IMC::RemoteSensorInfo *msg)
      {
        if(m_args.enableToggle) {
          bool stopPrevious = false;
          if(m_formation_started && m_args.keepFormation && m_last_tag_timer.overflow()) {
            stopPrevious = true;
            spew("StopPrevious true");
          }
          if(!m_formation_started || stopPrevious) {
            if(m_db_used) {
              size_t pos_end = msg->id.find(m_args.positionFilterPrefix);
              if(pos_end != std::string::npos) {
                m_of_msg.custom.clear();

                uint16_t minTagInterval = 0;
                uint16_t maxTagInterval = 0;
                try {
                  //if(tagInDB(std::stoi(msg->id.substr(pos_end+m_args.positionFilterPrefix.length())), m_of_msg)) {
                  if(getTagInfoFromDB(std::stoi(msg->id.substr(pos_end+m_args.positionFilterPrefix.length())),minTagInterval,maxTagInterval)) {
                    m_of_msg.custom = generateCustomParameters(m_args.formation_rotation_step,minTagInterval, maxTagInterval, m_args.ref_timeout, m_args.ref_send_interval);
                    spew("Found tag");
                    if(stopPrevious) {
                      sendFormationStop();
                    }
                    sendFormationStart(msg->id);
                  } else {
                    spew("Did not find tag \"%s\"", msg->id.substr(pos_end+m_args.positionFilterPrefix.length()).c_str());
                  }
                } catch (...) {
                  err("Could not solve tag \"%s\" in db", msg->id.substr(pos_end+m_args.positionFilterPrefix.length()).c_str());
                }
              }
            } else if(m_monitoredEstimators.find(msg->id) != m_monitoredEstimators.end()) {
                spew("Monitored tag found from parameter");
                if(stopPrevious) {
                  sendFormationStop();
                }
                m_of_msg.custom = generateCustomParameters(m_args.formation_rotation_step, m_args.fishtag_min_interval, m_args.fishtag_max_interval, m_args.ref_timeout, m_args.ref_send_interval);
                sendFormationStart(msg->id);
            }
          }
          if(m_formation_started) {
            if(m_tracked_id == msg->id) {
              m_last_rs_msg = *msg;
              m_last_tag_timer.reset();
            }
          }
        }
      }

      //! Checks if a tag is in the taglist DB and fills relevant fields in the otterformation message
      bool getTagInfoFromDB(const uint32_t transId, uint16_t& minTagInterval, uint16_t& maxTagInterval) {
        std::string query = "select minInterval, maxInterval from taglist where ID=" + std::to_string(transId);
        
        sqlite3_stmt* db_handle = nullptr;

        if (sqlite3_prepare_v2(m_db, query.c_str(), query.length(), &db_handle, 0) == SQLITE_OK) {
          if(sqlite3_step(db_handle) == SQLITE_ROW) {
            try{
              minTagInterval = sqlite3_column_int(db_handle, 0);
              maxTagInterval = sqlite3_column_int(db_handle, 1);
            } catch (...) {
              sqlite3_finalize(db_handle);
              return false;
            }
            sqlite3_finalize(db_handle);
            return true;
          }
        }
        sqlite3_finalize(db_handle);
        return false;
      }

      std::string generateCustomParameters(double rotation_dist, double minTagInterval, double maxTagInterval, double timeout, double FollowRefInterval) {
        return 
        "r=" + std::to_string(rotation_dist) +
        ";i=" + std::to_string(minTagInterval) +
        ";x=" + std::to_string(maxTagInterval) +
        ";t=" + std::to_string(timeout) +
        ";f=" + std::to_string(FollowRefInterval) +
        ";";
      }


      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);
          if(m_formation_started && !m_args.keepFormation && m_last_tag_timer.overflow()) {
            war("Formation stopped due to timeout");
            sendFormationStop();
          }
        }
      }
    };
  }
}

DUNE_TASK
