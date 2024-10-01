//***************************************************************************
// Copyright 2013-2022 Norwegian University of Science and Technology (NTNU)*
// Department of Engineering Cybernetics (ITK)                              *
//***************************************************************************
// This file is an extention to DUNE: Unified Navigation Environment.       *
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
//Author: Nikolai Lauvås (Reusing some code by Ricardo Gomes and José Braga)*
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <USER/DUNE.hpp>
#include <Ottermodel/thruster.hpp>
namespace Control
{
  namespace ASV
  {
    namespace HeadingAndSpeedForce
    {
      //! Tolerance for very low meters per second speed.
      static const float c_mps_tol = 0.1;
      //! Tolerance for heading error.
      static const float c_yaw_tol = 0.2;

      using DUNE_NAMESPACES;

      struct Arguments
      {
        //! Maximum Motor thrust.
        float act_max;
        //! Ramp actuation limit when the value is rising in actuation per second
        float act_ramp;
        //! End of scale value for RPM's at 100% of thurst
        float rpm_eos;

        //! Maximum acceleration step to smooth speed ramp in mps control
        int16_t max_force_accel;
        //! Maximum heading error to thrust.
        float yaw_max;
        //! PID gains for heading controller.
        std::vector<float> yaw_gains;
        //! Control logic for saturation.
        bool share;
        //! Log the size of each PID parcel
        bool log_parcels;

        //! MPS to force controller feedforward gain
        float mps_force_ffgain;
        //! Minimum value admissible for desired thrust force
        float min_force;
        //! Maximum value admissible for desired thrust force
        float max_force;
        //! Limit for the integral term, mps to force
        float mps_force_max_int;
        //! PID gains for mps to force controller.
        std::vector<float> mps_force_gains;
        //! GPS entity label.
        std::string elabel_gps;
        //! Min speed to switch to course control
        double minCourseSpeed;
        //! Minimum timestep accepted
        double min_timestep_accepted;
      };

      struct Task: public Tasks::Task
      {
        //! MPS(speed) to thrustforce PID controller
        USER::Control::DiscretePID m_mps_force_pid;
        //! YAW(Heading) PID controller
        USER::Control::DiscretePID m_yaw_pid;
        //! Control Parcels for meters per second controller returning force
        IMC::ControlParcel m_parcel_mps_force;
        //! Control Parcels for yaw controller
        IMC::ControlParcel m_parcel_yaw;
        //! Desired heading.
        float m_desired_yaw;
        //! Desired speed.
        float m_desired_speed;
        //! Desired speed units.
        uint8_t m_speed_units;
        //! Time of last estimated state message.
        Delta m_delta;
        //! Current motor actuation.
        IMC::SetThrusterActuation m_act[2];
        //! Last motor actuation.
        IMC::SetThrusterActuation m_last_act[2];
        //! previous value of the desired force
        float m_previous_force;
        //! Apply common actuation.
        bool m_common;
        //! GPS entity eid.
        int m_gps_eid;
        //! Control loops last reference
        uint32_t m_scope_ref;
        //! Task arguments.
        Arguments m_args;
        //! Current course from GPS
        double m_current_course;

        Task(const std::string& name, Tasks::Context& ctx):
          Tasks::Task(name, ctx),
          m_common(false),
          m_scope_ref(0)
        {
          param("Maximum Thrust Actuation", m_args.act_max)
          .defaultValue("1.0")
          .description("Maximum Motor Command");

          param("Yaw PID Gains", m_args.yaw_gains)
          .defaultValue("")
          .size(3)
          .description("PID gains for YAW controller");

          param("Maximum Heading Error to Thrust", m_args.yaw_max)
          .defaultValue("30.0")
          .description("Maximum admissable heading error to thrust");

          param("Share Saturation", m_args.share)
          .defaultValue("false")
          .description("Share saturation");

          param("Ramp Actuation Limit", m_args.act_ramp)
          .defaultValue("0.0")
          .description("Ramp actuation limit when the value is rising in actuation per second");

          param("MPS Force PID Gains", m_args.mps_force_gains)
          .defaultValue("200.0, 5.0, 0.0")
          .size(3)
          .description("PID Force gains for MPS controller");

          param("MPS Force Feedforward Gain", m_args.mps_force_ffgain)
          .defaultValue("0.0")
          .description("MPS Force controller feedforward gain");

          param("MPS Force Integral Limit", m_args.mps_force_max_int)
          .defaultValue("-1.0")
          .description("Limit for the integral term mps to Force");

          param("Minimum Force Limit", m_args.min_force)
          .defaultValue("-135")
          .units(Units::Newton)
          .description("Minimum value admissible for desired Force");

          param("Maximum Force Limit", m_args.max_force)
          .defaultValue("239.364")
          .units(Units::Newton)
          .description("Maximum value admissible for desired Force");

          param("Maximum force Acceleration", m_args.max_force_accel)
          .defaultValue("70")
          .units(Units::Newton)
          .description("Maximum acceleration step to smooth speed ramp in mps control");

          param("RPMs at Maximum Thrust", m_args.rpm_eos)
          .defaultValue("2500")
          .units(Units::RPM)
          .description("End of scale value for RPM's at 100% of thurst");

          param("Entity Label - GPS", m_args.elabel_gps)
          .defaultValue("GPS")
          .description("Entity label of 'GpsFix' messages");
          
          param("Minimum speed to use course", m_args.minCourseSpeed)
          .defaultValue("-1")
          .description("Minimum SOG reading required before using course. Else uses heading. Uses heading if -1.");

          param("Log PID Parcels", m_args.log_parcels)
          .defaultValue("false")
          .description("Log the size of each PID parcel");

          param("Minimum timestep accepted", m_args.min_timestep_accepted)
          .defaultValue("0.5")
          .minimumValue("0.0")
          .maximumValue("5.0")
          .description("Log the size of each PID parcel");
          m_desired_speed = 0.0;
          m_speed_units = IMC::SUNITS_PERCENTAGE;

          // Initialize entity state.
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);

          // Register handler routines.
          bind<IMC::Abort>(this);
          bind<IMC::EstimatedState>(this);
          bind<IMC::DesiredHeading>(this);
          bind<IMC::DesiredSpeed>(this);
          bind<IMC::ControlLoops>(this);
          bind<IMC::GpsFix>(this);
        }

        void
        onUpdateParameters(void)
        {
          if (paramChanged(m_args.yaw_max))
            m_args.yaw_max = Angles::radians(m_args.yaw_max);

          if (paramChanged(m_args.yaw_gains) ||
              paramChanged(m_args.mps_force_gains) ||
              paramChanged(m_args.mps_force_ffgain) ||
              paramChanged(m_args.mps_force_max_int) ||
              paramChanged(m_args.log_parcels))
          {
            reset();
            setup();
          }
        }

        //! Reserve entities.
        void
        onEntityReservation(void)
        {
          if (m_args.log_parcels)
          {
            std::string label = getEntityLabel();
            m_parcel_mps_force.setSourceEntity(reserveEntity(label + " - MPS Parcel"));
            m_parcel_yaw.setSourceEntity(reserveEntity(label + " - Yaw Parcel"));
          }
        }

        //! Resolve entities.
        void
        onEntityResolution(void)
        {
          try
          {
            m_gps_eid = resolveEntity(m_args.elabel_gps);
          }
          catch (...)
          {
            m_gps_eid = 0;
          }
        }

        //! On activation
        void
        onActivation(void)
        {
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        }

        //! On deactivation
        void
        onDeactivation(void)
        {
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
        }

        //! Reset PIDs and actuation references.
        void
        reset(void)
        {
          m_yaw_pid.reset();
          m_mps_force_pid.reset();

          m_previous_force = 0;

          for (uint8_t i = 0; i < 2; i++)
          {
            m_act[i].id = i;
            m_act[i].value = 0.0;
            m_last_act[i].id = i;
            m_last_act[i].value = 0.0;
            dispatch(m_act[i]);
          }
        }

        //! Setup PIDs.
        void
        setup(void)
        {

          // Do not set MPS PID output limits since we use a feedforward gain.
          m_mps_force_pid.setGains(m_args.mps_force_gains);
          m_mps_force_pid.setIntegralLimits(m_args.mps_force_max_int);

          m_yaw_pid.setGains(m_args.yaw_gains);

          // Log parcels.
          if (m_args.log_parcels)
          {
            m_mps_force_pid.enableParcels(this, &m_parcel_mps_force);
            m_yaw_pid.enableParcels(this, &m_parcel_yaw);
          }
        }

        void
        onResourceInitialization(void)
        {
          reset();
        }

        void
        consume(const IMC::Abort* msg)
        {
          if (msg->getDestination() != getSystemId())
            return;

          // This works as redundancy, in case everything else fails
          reset();
          debug("disabling");
        }

        void
        consume(const IMC::GpsFix* msg)
        {
          if (msg->getSource() != getSystemId())
            return;
          if (msg->getSourceEntity() != m_gps_eid)
            return;
          m_current_course = msg->cog;
        }

        void
        consume(const IMC::EstimatedState* msg)
        {
          if (msg->getSource() != getSystemId())
            return;

          double direction;
          if( (m_args.minCourseSpeed != -1.0) && (msg->u > m_args.minCourseSpeed) ) {
            direction = m_current_course;
          } else {
            direction = msg->psi;
          }
          // Compute time delta.
          double tstep = m_delta.getDelta();
          if (!isActive())
          {
            m_desired_yaw = direction;
            m_desired_speed = msg->u;
            return;
          }
          if(tstep > m_args.min_timestep_accepted) {
            reset();
            debug("disabling due to timestep too large");
            setEntityState(IMC::EntityState::ESTA_ERROR, "disabling due to timestep too large");
            IMC::Abort abort;
            abort.setDestination(getSystemId());
            dispatch(abort);
            // TODO: Consider milder action, such as a roof on delta.
          }
          // Check if we have a valid time delta.
          if (tstep < 0.0)
            return;

          float thrust_com = 0;
          float err_yaw = Angles::normalizeRadian(m_desired_yaw - direction);

          // Yaw controller.
          float thrust_diff = m_yaw_pid.step(tstep, err_yaw);


          // Thrust forward.
          if (thrustForward(err_yaw))
          {
            // Velocity controller.
            switch (m_speed_units)
            {
              case IMC::SUNITS_PERCENTAGE:
                thrust_com = (m_desired_speed / 100.0);
                break;
              case IMC::SUNITS_METERS_PS:
                thrust_com = mpsToForce(m_desired_speed, msg->u, tstep);
                break;

              case IMC::SUNITS_RPM:
                thrust_com = rpmToThrust(m_desired_speed);
                break;
              default:
                break;
            }
                                        
          }

            float force[2] = {thrust_com + thrust_diff, thrust_com - thrust_diff};
            //spew("Thrust com: %f, diff: %f", thrust_com, thrust_diff);

            // Positive saturation
            if(force[0] > m_args.max_force) {
              thrust_com -= force[0] - m_args.max_force;
              force[0] = thrust_com + thrust_diff;
              force[1] = thrust_com - thrust_diff;
            }
            if(force[1] > m_args.max_force) {
              thrust_com -= force[1] - m_args.max_force;
              force[0] = thrust_com - thrust_diff;
              force[1] = thrust_com + thrust_diff;
            }

            // Negative saturation
            if(force[0] < m_args.min_force) {
              force[0] = m_args.min_force;
              force[1] = -m_args.min_force;
            }
            else if(force[1] < m_args.min_force) {
              force[0] = -m_args.min_force;
              force[1] = m_args.min_force;
            }
            //spew("Force: %f, %f", force[0], force[1]);

            m_act[0].value = Ottermodel::Thruster::forceToThrust(force[0]);
            m_act[1].value = Ottermodel::Thruster::forceToThrust(force[1]);

            m_act[0].value = Math::trimValue(m_act[0].value, -1.0, 1.0);

            m_act[1].value = Math::trimValue(m_act[1].value, -1.0, 1.0);

            //spew("act: %f, %f", m_act[0].value, m_act[1].value);

            // TODO: Hva viss negativ går i metning?
            // TODO: 

          shareSaturation();

          dispatchThrust(m_act[0].value, tstep, 0);
          dispatchThrust(m_act[1].value, tstep, 1);
        }

        void
        consume(const IMC::DesiredHeading* msg)
        {
          if (!isActive())
            return;

          m_desired_yaw = msg->value;
        }

        void
        consume(const IMC::DesiredSpeed* msg)
        {
          if (!isActive())
            return;

          m_desired_speed = msg->value;
          m_speed_units = msg->speed_units;
        }

        void
        consume(const IMC::ControlLoops* msg)
        {
          if (!(msg->mask & (IMC::CL_YAW | IMC::CL_SPEED)))
            return;
          inf("msg->scope_ref: %u", msg->scope_ref);
          if (msg->scope_ref < m_scope_ref)
            return;

          m_scope_ref = msg->scope_ref;

          if (msg->enable == isActive())
            return;

          if (msg->enable)
            requestActivation();
          else
            requestDeactivation();

          debug(isActive() ? DTR("enabling") : DTR("disabling"));

          if (!isActive())
            reset();
        }

        //! Convert rpm value to thrust actuation
        //! @param[in] rpm value of rpms currently in the motor
        //! @param[in] desired_rpm desired rpms for the motor
        //! @param[in] timestep amount of time since last control step
        //! @return common thrust actuation.
        float
        rpmToThrust(float desired_rpm)
        {
          return desired_rpm / m_args.rpm_eos;
        }

        //! Runs PID speed controler taking in reference in meters per second
        //! and returns the desired force value.
        //! @param[in] vel absolute ground velocity.
        //! @param[in] timestep amount of time since last control step.
        //! @return desired force value.        
        float mpsToForce(float desiredMPS, float measuredMPS, double timestep) {
          // if desired speed is too low just turn off motor
          if (desiredMPS < c_mps_tol)
          {
            m_previous_force = 0.0;
            return 0.0;
          }
          // cannot let the timestep be zero
          if (timestep <= 0.0)
            return 0.0;

          m_parcel_mps_force.a = desiredMPS * m_args.mps_force_ffgain;
          float force = m_parcel_mps_force.a;
          force += m_mps_force_pid.step(timestep, desiredMPS - measuredMPS);


          // trim acceleration in force
          /*force = Math::trimValue(force, m_previous_force - m_args.max_force_accel * timestep,
                                m_previous_force + m_args.max_force_accel * timestep);
          */

          // trim force value
          force = Math::trimValue(force, m_args.min_force, m_args.max_force);
          m_previous_force = force;
          return force;
        }

        //! Dispatch to bus SetThrusterActuation message
        //! @param[in] value set thrust actuation value
        //! @param[in] timestep amount of time since last control step
        void
        dispatchThrust(float value, double timestep, uint8_t id)
        {
          if ((value > m_last_act[id].value) && (m_args.act_ramp > 0.0))
          {
            value = m_last_act[id].value + trimValue((value - m_last_act[id].value) / timestep,
                                                     0.0, m_args.act_ramp * timestep);
          }

          m_act[id].value = trimValue(value, -m_args.act_max, m_args.act_max);
          dispatch(m_act[id]);

          m_last_act[id].value = m_act[id].value;
        }

        //! Check if we are facing our waypoint to thrust.
        //! @param[in] yaw_err yaw error.
        //! @return true to thrust forward, false otherwise.
        bool
        thrustForward(float yaw_err)
        {
          // Check if we can thrust.
          if (m_common)
          {
            // Do not thrust forward if heading error is too large.
            if (std::fabs(yaw_err) > m_args.yaw_max * (1 + c_yaw_tol))
              m_common = false;
          }
          else
          {
            if (std::fabs(yaw_err) < m_args.yaw_max)
              m_common = true;
          }

          return m_common;
        }

        //! Distribute actuation references if over-saturated.
        void
        shareSaturation(void)
        {
          // New control logic when saturation occurs
          if (!m_args.share)
            return;

          for(uint8_t i = 0; i < 2; i++)
          {
            if (m_act[i].value > m_args.act_max)
            {
              float delta = m_act[i].value - m_args.act_max;
              m_act[i].value = m_args.act_max;
              m_act[(i + 1) % 2].value -= delta;
            }
            else if (m_act[i].value < -m_args.act_max)
            {
              float delta = m_act[i].value + m_args.act_max;
              m_act[i].value = -m_args.act_max;
              m_act[(i + 1) % 2].value -= delta;
            }
          }
        }

        void
        onMain(void)
        {
          while (!stopping())
          {
            waitForMessages(1.0);
          }
        }
      };
    }
  }
}

DUNE_TASK
