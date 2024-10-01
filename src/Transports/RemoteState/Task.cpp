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

namespace Transports
{
  //! 
  //! @author Nikolai Lauvås
  namespace RemoteState
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Toggle
      float toggled_time;
      std::vector<double> search_start_point;
    };

    struct Task: public DUNE::Tasks::Periodic
    {
      Arguments m_args;
      IMC::RemoteState m_rs;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx)
      {
        //param("Search Start", m_args.search_start_point)
        //.units(Units::Degree)
        //.size(2);

        bind<IMC::EstimatedState>(this);
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
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {

      }
      void
      consume(const IMC::EstimatedState* msg) {
        m_rs.lat = msg->lat;
        m_rs.lon = msg->lon;
        m_rs.psi = msg->psi;
        m_rs.speed = msg->u;
        m_rs.depth = msg->depth;
      }

 
      //! Main loop.
      void
      task(void)
      {
        consumeMessages();
        dispatch(m_rs);
      }
    };
  }
}

DUNE_TASK
