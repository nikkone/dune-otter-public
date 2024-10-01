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

// DUNE headers.
#include <DUNE/DUNE.hpp>
namespace Monitors
{
  //! Insert explanation on task behaviour here.
  //! @author Nikolai Lauvås
  namespace SpeedOfSoundInWater
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
// Initial Parameters for calculating Speed of Sound
      //! Initial Speed of Sound in water
      float init_c_sound;
      //! Initial temperature used for calculating Speed of Sound in water
      float init_temperature;
      //! Initial salinity used for calculating Speed of Sound in water
      float init_salinity;
      //! Initial depth used for calculating Speed of Sound in water
      float init_depth;
// Parameters for updating Speed of Sound
      //! Should the Speed of Sound in water be updated with water temperature
      bool update_c_sound_temp;
      //! Entity delivering water temperatures for updating the Speed of Sound in water
      std::string entity_c_sound_temp;
      //! Should the Speed of Sound in water be updated from measurement
      bool update_c_sound;
      //! Entity providing the Speed of Sound in water
      std::string entity_c_sound;
      //! Should the Speed of Sound in water be updated with water salinity measurement
      bool update_c_sound_salinity;
      //! Entity delivering water salinity for updating the Speed of Sound in water
      std::string entity_c_sound_salinity;

    };
    struct Task: public DUNE::Tasks::Periodic
    {
      //! Task arguments.
      Arguments m_args;
      //! Current Speed of sound in water
      float m_c_speed;
      //! Current temperature used for calculating Speed of sound in water
      float m_c_speed_temp;
      //! Current salinity used for calculating Speed of sound in water
      float m_c_speed_salinity;
      //! Current depth used for calculating Speed of sound in water
      float m_c_speed_depth;      
      //! Temperature entity label.
      int m_temp_eid;
      //! Speed of sound provider entity label.
      int m_c_sound_eid;
      //! Salinity provider entity label.
      int m_salinity_eid;

      IMC::SoundSpeed m_c_speed_msg;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx)
      {
// Initial Parameters for calculating Speed of Sound
        param("Initial Speed Of Sound", m_args.init_c_sound)
        .units(Units::MeterPerSecond)
        .description("The ID of the tracked fish tag")
        .defaultValue("1485.0");

        param("Initial Temperature", m_args.init_temperature)
        .units(Units::DegreeCelsius)
        .description("The ID of the tracked fish tag")
        .defaultValue("10.0");

        param("Initial Salinity", m_args.init_salinity)
        .description("Initial value of salinity in PPM")
        .defaultValue("25.0");

        param("Initial Depth", m_args.init_depth)
        .units(Units::Meter) 
        .description("Initial depth used to calculate Speed of Sound in water")
        .defaultValue("1.0");  
// Parameters for updating Speed of Sound
        param("Use Temperature For Speed Of Sound", m_args.update_c_sound_temp)
        .description("If any temperature measurements are to be used for updating the speed of sound.")
        .defaultValue("false");

         param("Temperature - Entity", m_args.entity_c_sound_temp)
        .description("The entity delivering water temperatures for updating the Speed of Sound in water")
        .defaultValue("Hydrophone");

        param("Use Salinity For Speed Of Sound", m_args.update_c_sound_salinity)
        .description("If any salinity measurements are to be used for updating the speed of sound.")
        .defaultValue("false");

         param("Salinity - Entity", m_args.entity_c_sound_salinity)
        .description("The entity delivering salinity for updating the Speed of Sound in water")
        .defaultValue("CTD");

        param("Use Speed Of Sound Measurement", m_args.update_c_sound)
        .description("If speed of sound is provided through IMC messages.")
        .defaultValue("false");

         param("Speed Of Sound - Entity", m_args.entity_c_sound)
        .description("The entity delivering the Speed of Sound in water")
        .defaultValue("CTD");        

        bind<IMC::Temperature>(this);
        bind<IMC::Salinity>(this);
        bind<IMC::SoundSpeed>(this);
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.init_c_sound))
          m_c_speed=m_args.init_c_sound;

        if(paramChanged(m_args.init_salinity))
          m_c_speed_salinity = m_args.init_salinity;

        if(paramChanged(m_args.init_temperature))
          m_c_speed_temp = m_args.init_temperature;

        if(paramChanged(m_args.init_depth))
          m_c_speed_depth = m_args.init_depth;


        if(!m_args.update_c_sound) {
          if(m_args.update_c_sound_salinity || m_args.update_c_sound_temp) {
            m_c_speed=calculateSpeedOfSound(m_c_speed_temp, m_c_speed_salinity, m_c_speed_depth);
            m_c_speed_msg.value = m_c_speed;
            dispatch(m_c_speed_msg);
            spew("Calculated c_speed: %f", m_c_speed);
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
        try
          {
            m_c_sound_eid = resolveEntity(m_args.entity_c_sound);
          }
          catch (...)
          {
            if(m_args.update_c_sound) {
              err("Could not find entity: %s", m_args.entity_c_sound.c_str());
            }
            m_c_sound_eid = 0;
          }
        try
          {
            m_temp_eid = resolveEntity(m_args.entity_c_sound_temp);
          }
          catch (...)
          {
            if(m_args.update_c_sound_temp) {            
              err("Could not find entity: %s", m_args.entity_c_sound_temp.c_str());
            }
            m_temp_eid = 0;

          }
        try
          {
            m_salinity_eid = resolveEntity(m_args.entity_c_sound_salinity);
          }
          catch (...)
          {
            if(m_args.update_c_sound_salinity) {            
              err("Could not find entity: %s", m_args.entity_c_sound_salinity.c_str());
            }
            m_salinity_eid = 0;
          }
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
      }


      void
      consume(const IMC::SoundSpeed* msg)
      {
        if(msg->getSourceEntity() == m_c_sound_eid) {
          if(m_args.update_c_sound) {
            m_c_speed = msg->value;
            m_c_speed_msg.value = m_c_speed;
            dispatch(m_c_speed_msg);
            spew("Setting c_sound to: %f", msg->value);
          }
        }
      }

      void
      consume(const IMC::Temperature* msg)
      {
        if(msg->getSourceEntity() == m_temp_eid) {
          if(m_args.update_c_sound_temp) {
            m_c_speed_temp = msg->value;
            m_c_speed = calculateSpeedOfSound(m_c_speed_temp, m_c_speed_salinity, m_c_speed_depth);
            m_c_speed_msg.value = m_c_speed;
            dispatch(m_c_speed_msg);
            spew("Recalculating c_sound with temperature: %f, Result: %f", msg->value, m_c_speed);
          }
        }
      }

      void
      consume(const IMC::Salinity* msg)
      {
        if(msg->getSourceEntity() == m_salinity_eid) {
          if(m_args.update_c_sound_salinity) {
            m_c_speed_salinity = msg->value;
            m_c_speed = calculateSpeedOfSound(m_c_speed_temp, m_c_speed_salinity, m_c_speed_depth);
            m_c_speed_msg.value = m_c_speed;
            dispatch(m_c_speed_msg);
            spew("Recalculating c_sound with salinity: %f, Result: %f", msg->value, m_c_speed);
          }
        }
      }

      //! Leroys formula for calculating the speed of sound in water
      //! Temperature given in Celsius
      //! Salinity given in PPM
      //! Depth given in Meters
      double calculateSpeedOfSound(double temperature, double salinity, double depth) {
        double c_speed = 1492.9;
        c_speed += 3*(temperature -10.0);
        c_speed -= (6*pow(temperature-10.0, 2.0))/1000;
        c_speed -= (4*pow(temperature-18.0, 2.0))/100;
        c_speed += 1.2*(salinity-35);
        c_speed -= ((temperature-18-0)*(salinity-35))/100;
        c_speed += depth/61;
        return c_speed;
      }
      //! Initialize resources.
      void
      onResourceInitialization(void)
      {

      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }


      //! Main loop.
      void
      task(void)
      {
        consumeMessages();
        m_c_speed_msg.value = m_c_speed;
        dispatch(m_c_speed_msg);
      }
    };
  }
}

DUNE_TASK
