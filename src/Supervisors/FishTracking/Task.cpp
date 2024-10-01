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
// written agreement between you and the Department of Engineering          *
// Cybernetics at the Norwegian University of Science and Technology        *
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

namespace Supervisors
{
  //! 
  //! @author Nikolai Lauvås
  namespace FishTracking
  {
    using DUNE_NAMESPACES;

    //! Enumeration tracking states
    enum state_t {S_INACTIVE,
                  S_TRANSIT,
                  S_SEARCHING,
                  S_ASSEMBLING_TRACKING_FORMATION,
                  S_TRACKING_SINGLE_VEHICLE,
                  S_TRACKING_FORMATION};
    enum event_t {
    E_NONE,
    E_ACTIVATE,
    E_DEACTIVATE,
    E_PLAN_FINISHED,
    E_SEARCH_PLAN_FINISHED,
    E_TAG_DETECTED_SELF,
    E_TAG_DETECTED_OTHER,
    E_ARRIVED,
    E_NEAR_OTHER_VEHICLE
    };
    struct Arguments
    {
      //! Toggle
      float toggled_time;
      std::vector<double> search_start_point;
    };

    struct Task: public DUNE::Tasks::Periodic
    {
      Arguments m_args;
      std::vector<event_t> m_eventBuffer;
      state_t m_currentState;
      //! Current Lat and Lon of vehicle.
      fp64_t m_current_lat, m_current_lon;
      uint8_t prevOutcome;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_currentState(S_INACTIVE), 
        m_current_lat(0.0),
        m_current_lon(0.0),
        prevOutcome(IMC::PlanControlState::LastPlanOutcomeEnum::LPO_NONE)
      {
        param("Search Start", m_args.search_start_point)
        .units(Units::Degree)
        .size(2);

        bind<IMC::EstimatedState>(this);
        bind<IMC::PlanControlState>(this);
        bind<IMC::TBRFishTag>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
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

      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        m_eventBuffer.push_back(E_ACTIVATE);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {

      }
      void
      consume(const IMC::EstimatedState* msg) {
        //inf("Received EstimatedState");
        m_current_lat=msg->lat;
        m_current_lon=msg->lon;
      }
      void
      consume(const IMC::PlanControlState* pcs)
      {
        if(prevOutcome != pcs->last_outcome) {
          if (pcs->last_outcome == IMC::PlanControlState::LastPlanOutcomeEnum::LPO_SUCCESS) {
            spew("pcs added");
            m_eventBuffer.push_back(E_PLAN_FINISHED);
          }

          prevOutcome = pcs->last_outcome;
        }
      }

      void
      consume(const IMC::TBRFishTag* msg) {
        if (msg->getSource() != getSystemId()) {
          m_eventBuffer.push_back(E_TAG_DETECTED_SELF);
        } else {
          m_eventBuffer.push_back(E_TAG_DETECTED_OTHER);
        }
      }
      state_t handleActivate() {
          IMC::PlanProbSpec spec;
          spec.setDestination(getSystemId());
          spec.vehicle = getSystemId();
          spec.problem_type = IMC::PlanProbSpec::TypeEnum::PPT_fpath;
          spec.start_lat = m_current_lat;
          spec.start_lon = m_current_lon;
          spec.end_lat = Math::Angles::radians(m_args.search_start_point[0]);
          spec.end_lon = Math::Angles::radians(m_args.search_start_point[1]);
          spec.speed = 2;
          spec.speed_units = IMC::SUNITS_METERS_PS;
          spec.custom = "t=5.0;p=1;a=1;";


          IMC::PolygonVertex south_west;
          south_west.lat = std::min(m_current_lat, Math::Angles::radians(m_args.search_start_point[0])) - 0.0001;
          south_west.lon = std::min(m_current_lon, Math::Angles::radians(m_args.search_start_point[1])) - 0.0001;
          IMC::PolygonVertex north_east;
          north_east.lat = std::max(m_current_lat, Math::Angles::radians(m_args.search_start_point[0])) + 0.0001;
          north_east.lon = std::max(m_current_lon, Math::Angles::radians(m_args.search_start_point[1])) + 0.0001;
          spec.area.push_back(south_west);
          spec.area.push_back(north_east);

          dispatch(spec);
          return S_TRANSIT;
      }
/*        state_t handleEvent(event_t &event) {
            switch(m_currentState) {
                case S_TRANSIT:
                  if(event == E_PLAN_FINISHED) {
                    inf("Arrived");
                    m_currentState = S_SEARCHING;
                  }
                    // Find plan, perform plan
                    // If no plan found, retry x times, then go to INACTIVE while giving an error message
                    // If TagDetection, then
                    // If goal reached, go to S_SEARCHING
                    break;
                case S_SEARCHING:
                  inf("Starting search");
                    // Get seraching assignment execute x times
                    // If Tag detection on any vessel, go to S_ASSEMBLING_TRACKING_FORMATION
                    // If search finished, either loiter, station keeping or return to base.
                    break;
                case S_ASSEMBLING_TRACKING_FORMATION:
                    // If tag detection on this vessel, go to S_TRACKING_SINGLE_VEHICLE
                    // If Tag detection on other vessel and not on this, get assigned location and enter S_TRANSIT
                    break;
                case S_TRACKING_SINGLE_VEHICLE:
                    break;
                case S_TRACKING_FORMATION:
                    break;
                case S_INACTIVE:
                  inf("Inactive, waiting for E_ACTIVATE event");
                  if(event == E_ACTIVATE) {
                    inf("Activated, changing state to transit.");
                    m_currentState = handleActivate();
                  }
                    break;
                default:
                    err("FSM error, state should not be reached");
            }
          return m_currentState;
        }*/

        void onInactiveState(const event_t &event) {
            switch(event) {
              case E_ACTIVATE:
                  inf("Activated, changing state to transit.");
                  m_currentState = handleActivate();
                break;
              case E_DEACTIVATE:
              case E_SEARCH_PLAN_FINISHED:
              case E_TAG_DETECTED_SELF:
              case E_TAG_DETECTED_OTHER:
              case E_ARRIVED:
              case E_NEAR_OTHER_VEHICLE:
              case E_PLAN_FINISHED:
              default:
                spew("Event has no effect in state: S_INACTIVE");
            }
        }

        void onTransitState(const event_t &event) {
            switch(event) {
              case E_PLAN_FINISHED:
                inf("Arrived");
                m_currentState = S_SEARCHING;
                m_eventBuffer.push_back(E_ARRIVED);
                break;
              case E_DEACTIVATE:
              case E_ACTIVATE:
              case E_SEARCH_PLAN_FINISHED:
              case E_TAG_DETECTED_SELF:
              case E_TAG_DETECTED_OTHER:
              case E_ARRIVED:
              case E_NEAR_OTHER_VEHICLE:
              default:
                spew("Event has no effect in state: S_TRANSIT");
            }
        }
        void onSearcingState(const event_t &event) {
            switch(event) {
              case E_ARRIVED:
                spew("Starting search");
                break;
              case E_ACTIVATE:
              case E_DEACTIVATE:
              case E_SEARCH_PLAN_FINISHED:
              case E_TAG_DETECTED_SELF:
              case E_TAG_DETECTED_OTHER:
              case E_NEAR_OTHER_VEHICLE:
              case E_PLAN_FINISHED:
              default:
                spew("Event has no effect in state: S_SEARCHING");
            }
        }
        void onAssmblingState(const event_t &event) {
            switch(event) {
              case E_ACTIVATE:
              case E_DEACTIVATE:
              case E_SEARCH_PLAN_FINISHED:
              case E_TAG_DETECTED_SELF:
              case E_TAG_DETECTED_OTHER:
              case E_ARRIVED:
              case E_NEAR_OTHER_VEHICLE:
              case E_PLAN_FINISHED:
              default:
                spew("Event has no effect in state: S_ASSEMBLING_TRACKING_FORMATION");
            }
        }
        void onSingleTrackingState(const event_t &event) {
            switch(event) {
              case E_ACTIVATE:
              case E_DEACTIVATE:
              case E_SEARCH_PLAN_FINISHED:
              case E_TAG_DETECTED_SELF:
              case E_TAG_DETECTED_OTHER:
              case E_ARRIVED:
              case E_NEAR_OTHER_VEHICLE:
              case E_PLAN_FINISHED:
              default:
                spew("Event has no effect in state: S_TRACKING_SINGLE_VEHICLE");
            }
        }
        void onFormationTrackingState(const event_t &event) {
            switch(event) {
              case E_ACTIVATE:
              case E_DEACTIVATE:
              case E_SEARCH_PLAN_FINISHED:
              case E_TAG_DETECTED_SELF:
              case E_TAG_DETECTED_OTHER:
              case E_ARRIVED:
              case E_NEAR_OTHER_VEHICLE:
              case E_PLAN_FINISHED:
              default:
                spew("Event has no effect in state: S_TRACKING_FORMATION");
            }
        }

      void stateAction(const state_t &state, const event_t &event) {
        switch(state) {
            case S_TRANSIT:
              spew("S_TRANSIT");
              onTransitState(event);
                break;
            case S_SEARCHING:
              spew("S_SEARCHING");
              onSearcingState(event);
                break;
            case S_ASSEMBLING_TRACKING_FORMATION:
              spew("S_ASSEMBLING_TRACKING_FORMATION");
              onAssmblingState(event);
                break;
            case S_TRACKING_SINGLE_VEHICLE:
              spew("S_TRACKING_SINGLE_VEHICLE");
              onSingleTrackingState(event);
                break;
            case S_TRACKING_FORMATION:
              spew("S_TRACKING_FORMATION");
              onFormationTrackingState(event);
                break;
            case S_INACTIVE:
              spew("S_INACTIVE");
              onInactiveState(event);
                break;
            default:
                err("FSM error, state should not be reached");
        }
      }
      //! Main loop.
      void
      task(void)
      {
        consumeMessages();
        if(!m_eventBuffer.empty()) {
          stateAction(m_currentState, m_eventBuffer.back());
          m_eventBuffer.pop_back();
        } else {
          stateAction(m_currentState, event_t::E_NONE);
        }

      }
    };
  }
}

DUNE_TASK
