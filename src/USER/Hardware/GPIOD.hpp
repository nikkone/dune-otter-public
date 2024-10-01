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
// Author: Nikolai Lauvås (Based on old interface by Ricardo Martins)       *
//***************************************************************************

#ifndef DUNE_HARDWARE_GPIOD_HPP_INCLUDED_
#define DUNE_HARDWARE_GPIOD_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <string>
#include <gpiod.h>
// DUNE headers.
#include <DUNE/Config.hpp>

namespace DUNE
{
  namespace Hardware
  {
    // Export symbol.
    class DUNE_DLL_SYM GPIOD;

    class GPIOD
    {
    public:
      enum Direction
      {
        //! GPIO is used as input.
        GPIOD_DIR_INPUT,
        //! GPIO is used as output.
        GPIOD_DIR_OUTPUT
      };

      //! Initialize GPIO.
      //! @param[in] number GPIO number.
      GPIOD(unsigned int number, std::string chipname = "gpiochip0");
      //! Default destructor.
      ~GPIOD(void);

      //! Set GPIO direction.
      //! @param[in] direction GPIO direction.
      void
      setDirection(Direction direction);

      //! Set GPIO direction.
      //! @param[in] direction "input" or "output".
      void
      setDirection(const std::string& direction);

      //! Set GPIO value.
      //! @param[in] value pin value (false = off, true = on).
      void
      setValue(bool value);

      //! Get GPIO value.
      //! @return pin value (false = off, true = on).
      bool
      getValue(void);

    private:
      //! Disallow copy constructor.
      GPIOD(const GPIOD&);

      //! Disallow copy assignment.
      GPIOD& operator=(const GPIOD&);

      //! GPIO number.
      //! GPIO direction.
      Direction m_direction;

#if defined(DUNE_OS_LINUX)
      //! GPIO Chip.
      std::string chipname;
      //! GPIO number.
      unsigned int line_num;
      struct gpiod_chip *chip;
      struct gpiod_line *line;


#endif
    };
  }
}

#endif
