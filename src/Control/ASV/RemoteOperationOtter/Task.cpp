//***************************************************************************
// Copyright 2007-2021 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: Ricardo Martins                                                  *
//***************************************************************************

// ISO C++ 98 headers.
#include <algorithm>

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <Ottermodel/thruster.hpp>

namespace Control
{
  namespace ASV
  {
    namespace RemoteOperationOtter
    {
      using DUNE_NAMESPACES;
      typedef enum RCmode
      {
          RCM_NORMAL   = 0,
          RCM_FORCE    = 1,
          RCM_SKEWED   = 2,
          RCM_AUTOPILOT= 3,
          RCM_PATH     = 4
      } RCmode_t;
      //! Task arguments.
      struct Arguments
      {
        //! Thrust scaling.
        double scale;
        //! Factor to reduce forward thrust when differential thrusting
        float skewed_motor_force_factor;
        u_int8_t rcmode;

        //! Minimum value admissible for desired thrust force
        float min_force;
        //! Maximum value admissible for desired thrust force
        float max_force;
        float autopilot_max_speed;
        float autopilot_max_yaw;
        float speed_step;
        float yaw_step;
      };

      struct Task: public DUNE::Control::BasicRemoteOperation
      {
        //! Motor commands.
        IMC::SetThrusterActuation m_thrust[2];
        //! DesiredSpeed reference
        IMC::DesiredSpeed msg_speed;
        //! Outgoing desired heading message.
        IMC::DesiredHeading msg_heading;
        //! Task arguments.
        Arguments m_args;
        //! Control loops last reference
        uint32_t m_scope_ref;
        double m_speed;
        double m_heading;
        //! Control loops message
        IMC::ControlLoops m_cloops;
        //! Active loops
        uint32_t m_aloops;
        float m_autopilot_max_yaw;
        float m_yaw_step;
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::BasicRemoteOperation(name, ctx),
          m_scope_ref(0),
          m_speed(0),
          m_heading(0)
        {
          param("Thrust Scale", m_args.scale)
          .defaultValue("1.0");

          param("Skewed Motor Force Factor", m_args.skewed_motor_force_factor)
          .defaultValue("0.0")
          .description("Factor to reduce forward thrust when differential thrusting.");

          param("RC mode", m_args.rcmode)
          .defaultValue("0")
          .description("");

          param("Minimum Force Limit", m_args.min_force)
          .defaultValue("-135")
          .units(Units::Newton)
          .description("Minimum value admissible for desired Force");

          param("Maximum Force Limit", m_args.max_force)
          .defaultValue("239.364")
          .units(Units::Newton)
          .description("Maximum value admissible for desired Force");

          param("Autopilot Max Speed", m_args.autopilot_max_speed)
          .defaultValue("2.3")
          .units(Units::MeterPerSecond)
          .description("Maximum value admissible given to desiredSpeed in Autopilot mode");
          
          param("Autopuilot Max Yaw", m_args.autopilot_max_yaw)
          .defaultValue("180")
          .units(Units::Degree)
          .description("Maximum value admissible given to desiredHeading in Autopilot mode");

          param("Speed step", m_args.speed_step)
          .defaultValue("0.05")
          .units(Units::Percentage)
          .description("Step added/subtracted when changing speed");

          param("Yaw step", m_args.yaw_step)
          .defaultValue("30")
          .units(Units::Degree)
          .description("Step added/subtracted when changing yaw");    

          // Add remote actions.
          addActionAxis("Port Motor");
          addActionAxis("Starboard Motor");
          addActionButton("Accelerate");
          addActionButton("Decelerate");
          addActionButton("Left");
          addActionButton("Right");
          addActionAxis("Heading");
          addActionAxis("Throttle");
          addActionButton("Stop");

          // Initialize SetThrusterActuation messages.
          m_thrust[0].id = 0;
          m_thrust[1].id = 1;
          msg_speed.speed_units = IMC::SUNITS_METERS_PS;

          bind<IMC::Abort>(this);
          bind<IMC::ControlLoops>(this);
        }

        void
        onUpdateParameters(void)
        {
          if(isActive()) {
            setup();
          }
          
          if(paramChanged(m_args.autopilot_max_yaw)) {
            m_autopilot_max_yaw = DUNE::Math::Angles::radians(m_args.autopilot_max_yaw);
          }
          if(paramChanged(m_args.yaw_step)) {
            m_yaw_step = DUNE::Math::Angles::radians(m_args.yaw_step);
          }
        }

        void
        onActivation(void)
        {
          setup();
        }

        void
        onDeactivation(void)
        {
          reset();
        }

        void
        onConnectionTimeout(void)
        {
          reset();
        }

        double
        applyScale(int value)
        {
          return Math::trimValue((value / 127.0) * m_args.scale, -1.0, 1.0);
        }
        void setup() {
          if(m_args.rcmode == RCM_AUTOPILOT) {
            enableControlLoops(IMC::CL_SPEED);
            enableControlLoops(IMC::CL_YAW);
          } else {
            if(m_aloops != 0) {
              disableControlLoops(IMC::CL_SPEED);
              disableControlLoops(IMC::CL_YAW);
            }
          }
          m_thrust[0].value = 0;
          m_thrust[1].value = 0;
          m_speed = m_heading = 0;
          actuate();
        }
        void reset() {
          if(m_args.rcmode == RCM_AUTOPILOT) {
            disableControlLoops(IMC::CL_SPEED);
            disableControlLoops(IMC::CL_YAW);
          }
          m_thrust[0].value = 0;
          m_thrust[1].value = 0;
          m_speed = m_heading = 0;
          actuate();
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
        consume(const IMC::ControlLoops* msg)
        {
          m_cloops.scope_ref = msg->scope_ref;
          debug("Update ref %u", m_cloops.scope_ref);
        }

        //! Enable control loops.
        //! @param mask control loop mask
        void
        enableControlLoops(uint32_t mask)
        {
          configureControlLoops(IMC::ControlLoops::CL_ENABLE, mask);
        }

        //! Disable control loops (need to use only if
        //! control mode changes during path control, not on deactivation).
        //! @param mask control loop mask
        void
        disableControlLoops(uint32_t mask)
        {
          configureControlLoops(IMC::ControlLoops::CL_DISABLE, mask);
        }
        void
        configureControlLoops(uint8_t enable, uint32_t mask)
        {
          if (enable)
          {
            if ((mask & m_aloops) == mask)
              return;

            m_aloops |= mask;
          }
          else
          {
            if ((mask & ~m_aloops) == mask)
              return;

            m_aloops &= ~mask;
          }

          m_cloops.enable = enable;
          m_cloops.mask = mask;
          dispatch(m_cloops);
        }
        void
        onRemoteActions(const IMC::RemoteActions* msg)
        {
          TupleList tuples(msg->actions);

          m_thrust[0].value = applyScale(tuples.get("Port Motor", 0));
          m_thrust[1].value = applyScale(tuples.get("Starboard Motor", 0));

          if (m_thrust[0].value == 0 && m_thrust[1].value == 0)
          {
              double throttle = applyScale(tuples.get("Throttle", 0));
              //inf("%f", throttle);
              if (tuples.get("Decelerate", 0))
                m_speed -= m_args.speed_step;
              else if (tuples.get("Accelerate", 0))
                m_speed += m_args.speed_step;
              else if (throttle) {
                m_speed = throttle;
              }
                

              m_speed = Math::trimValue(m_speed, -1.0 , 1.0);
              double hdng = (tuples.get("Heading", 0)) / 127.0;
              if (tuples.get("Left", 0)) {
                m_heading -= m_yaw_step;
              } else if (tuples.get("Right", 0)) {
                m_heading += m_yaw_step;
              }
              double leftThrust = m_speed;
              double rightThrust = m_speed;


              switch(m_args.rcmode) {
                case RCM_NORMAL:
                  leftThrust *= 1+hdng*2;
                  rightThrust *= 1-hdng*2;
                break;
                case RCM_FORCE:
                  {
                    float thrust_com = Ottermodel::Thruster::thrustToForce(m_speed);
                    float thrust_diff = Ottermodel::Thruster::thrustToForce(hdng*2);
                    float force[2] = {thrust_com + thrust_diff, thrust_com - thrust_diff};

                    // Positive saturation
                    if(force[0] > m_args.max_force) {
                      thrust_com -= force[0] - m_args.max_force;
                      force[0] = thrust_com + thrust_diff;
                      force[1] = thrust_com - thrust_diff;
                    }
                    if(force[1] > m_args.max_force) {
                      thrust_com -= force[1] - m_args.max_force;
                      force[0] = thrust_com + thrust_diff;
                      force[1] = thrust_com - thrust_diff;
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
                    leftThrust  = Ottermodel::Thruster::forceToThrust(force[0]);
                    rightThrust = Ottermodel::Thruster::forceToThrust(force[1]);
                  }
                break;
                case RCM_SKEWED:
                  if(hdng>0) {
                    leftThrust  *= 1+(hdng*2*m_args.skewed_motor_force_factor);
                    rightThrust *= 1-hdng*2;
                  } else if(hdng<0) {
                    rightThrust *= 1-(hdng*2*m_args.skewed_motor_force_factor);
                    leftThrust  *= 1+hdng*2;
                  }
                break;
                case RCM_AUTOPILOT:
                  msg_speed.value = m_args.autopilot_max_speed*m_speed;
                  dispatch(msg_speed);
                  msg_heading.value = DUNE::Math::Angles::normalizeRadian(m_autopilot_max_yaw*hdng + m_heading);
                  dispatch(msg_heading);
                break;
                case RCM_PATH:
                break;
                default:
                  spew("Invalid mode %u, changing to normal mode.", m_args.rcmode);
                  m_args.rcmode = RCM_NORMAL;
              };
              //spew("Heading: %f", hdng);

              m_thrust[0].value = Math::trimValue(leftThrust, -1.0, 1.0);
              m_thrust[1].value = Math::trimValue(rightThrust, -1.0, 1.0);

              if (tuples.get("Stop", 0))
                m_speed = m_thrust[0].value = m_thrust[1].value = m_heading = 0;
          }
          else {
            m_speed = m_heading = 0;
          }
        }

        void
        actuate(void)
        {
          if(m_args.rcmode == RCM_AUTOPILOT) {
            return;
          }
          debug("%0.2f %0.2f %u", m_thrust[0].value, m_thrust[1].value, m_aloops);
//
          dispatch(m_thrust[0]);
          dispatch(m_thrust[1]);
        }
      };
    }
  }
}

DUNE_TASK
