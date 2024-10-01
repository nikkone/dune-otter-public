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

#ifndef	CONSUMER
#define	CONSUMER	"Consumer"
#endif

// ISO C++ 98 headers.
#include <cerrno>

// DUNE headers.
#include <DUNE/System/Error.hpp>
#include <DUNE/Streams/Terminal.hpp>
#include <DUNE/Utils/String.hpp>
#include <USER/Hardware/GPIOD.hpp>


//#include <gpiod.h>

namespace DUNE
{
  namespace Hardware
  {
    using System::Error;
    using Utils::String;
    
    GPIOD::GPIOD(unsigned int number, std::string device_chipname):
      m_direction(GPIOD_DIR_INPUT),
      chipname(device_chipname),
      line_num(number)
    {
      // Linux 2.6 implementation.
#if defined(DUNE_OS_LINUX)
	chip = gpiod_chip_open_by_name(chipname.c_str());
	if (!chip) {
		throw Error("Open chip failed", chipname.c_str());
	}

	line = gpiod_chip_get_line(chip, line_num);
	if (!line) {
		throw Error("Get line failed", std::to_string(line_num));
	}
      // Lacking implementation.
#else
      throw Error("unimplemented feature", "USER::Hardware::GPIOD");
#endif
    }

    GPIOD::~GPIOD(void)
    {
      // Linux 2.6 implementation.
#if defined(DUNE_OS_LINUX)
      try
      {
        gpiod_line_release(line);
        gpiod_chip_close(chip);
      }
      catch (std::exception& e)
      {
        DUNE_ERR("USER::Hardware::GPIOD", e.what());
      }
#endif
    }

    void
    GPIOD::setDirection(Direction direction)
    {
      if (direction == GPIOD_DIR_INPUT)
      {
#if defined(DUNE_OS_LINUX)
	int ret = gpiod_line_request_input(line, CONSUMER);
	if (ret < 0) {
        gpiod_line_release(line);
        throw Error("Request line as input failed", std::to_string(line_num));
	}
#endif
      }
      else
      {
#if defined(DUNE_OS_LINUX)
	int ret = gpiod_line_request_output(line, CONSUMER, 0);
	if (ret < 0) {
        gpiod_line_release(line);
        throw Error("Request line as output failed", std::to_string(line_num));
	}
#endif
      }

      m_direction = direction;
    }

    void
    GPIOD::setDirection(const std::string& direction)
    {
      if (direction == "input")
        setDirection(GPIOD_DIR_INPUT);
      else if (direction == "output")
        setDirection(GPIOD_DIR_OUTPUT);
      else
        throw Error(direction, "GPIO direction must be either 'input' or 'output'");
    }

    void
    GPIOD::setValue(bool value)
    {
      if (m_direction != GPIOD_DIR_OUTPUT)
        throw Error("GPIO is not configured as output", String::str(line_num));

#if defined(DUNE_OS_LINUX)
		int ret = gpiod_line_set_value(line, value ? 1 : 0);
		if (ret < 0) {
            gpiod_line_release(line);
            throw Error("Set line output failed", std::to_string(line_num));
		    
		}
#else
      (void)value;
#endif
    }

    bool
    GPIOD::getValue(void)
    {
      if (m_direction != GPIOD_DIR_INPUT)
        throw Error("GPIO is not configured as input", String::str(line_num));

#if defined(DUNE_OS_LINUX)
      int val = gpiod_line_get_value(line);
      if (val < 0) {
          gpiod_line_release(line);
          throw Error("unable to read GPIO value", std::to_string(line_num));
      }
      return val == 1;
#endif

      return false;
    }

  }
}
