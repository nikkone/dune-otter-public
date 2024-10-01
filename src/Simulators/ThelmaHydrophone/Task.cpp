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


#include <fstream>

namespace Simulators
{
  //! This task simulates the behavior of the Sensors/ThelmaHydrophone task. It currently only supports simulating tag detections.
  //! The tag detections are created by reading a textfile where each line imitates the message received over RS-485 from Thelma Biotel Hydrophones.
  //!
  //! @author Nikolai Lauvås
  namespace ThelmaHydrophone
  {
    using DUNE_NAMESPACES;
    //! Number of fields in fish tag message
    static const unsigned c_tag_fields = 11;
    //! Number of fields in TBR-700RT sensor reading
    static const unsigned c_tbr_sensor_fields = 8;
    //! Task arguments
    struct Arguments
    {
      //! Path to DB file
      std::string msg_path;
      //! Fixed tag delay
      float msg_delay;
      //! Tag delay according to difference in timestamp
      bool realTimeDelay;
      //! Real time delay multiplier
      float realTimeMultiplier;
    };
    struct Task: public DUNE::Tasks::Task
    {
      // Task arguments
      Arguments m_args;

      IMC::TBRFishTag m_tag_msg;

      IMC::RemoteSensorInfo m_tag_position;

      //! UNIX Timestamp.
      uint32_t m_prev_unix_timestamp;
      //! Milliseconds part.
      uint32_t m_prev_millis;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_prev_unix_timestamp(0)
      {
        param("Messagefile Path", m_args.msg_path)
        .defaultValue("")
        .description("Path to file with messages");

        param("Delay Between Tag Messages", m_args.msg_delay)
        .units(Units::Millisecond)
        .defaultValue("0.0")
        .description("How long to wait between Tag messages.");
        
        param("Use Real Time Delay", m_args.realTimeDelay)
        .defaultValue("true")
        .description("Enable/disable tag delay according to difference in timestamp.");  

        param("Real Time Delay Multiplyer", m_args.realTimeMultiplier)
        .defaultValue("1.0")
        .description("Allows to change speed of real time tag sending while keeping the correct intervals.");   
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

      //! Process sentence.
      //! @param[in] line line.
      int
      processSentence(const std::string& line)
      {
        spew(DTR("Process"));
        if (line.find("ack01") != std::string::npos) {
          spew(DTR("Sensor clock diciplined"));
        } if(line.find("ack02") != std::string::npos) {
          spew(DTR("Sensor timestamp set"));
        } if (line.find("$") != std::string::npos) {

          // Discard leading noise.
          size_t sidx = 0;
          for (sidx = 0; sidx < line.size(); ++sidx)
          {
            if (line[sidx] == '$')
              break;
          }

          // Split sentence
          std::vector<std::string> parts;
          try {
            spew(DTR("try"));
            String::split(line.substr(sidx + 1, line.size()), ",", parts);
          } catch(const std::exception& ex) {
            err(DTR("Invalid argument: %s"), ex.what());
            return 0;
          }
          spew("Fields %lu", parts.size());
          if(parts.size() >= 3) {
            if(parts[2]  == "TBR Sensor") {
              interpretSensorReading(parts);
              return 1;
            } else if(parts.size() == c_tag_fields) {
              interpretTagDetection(parts);
              return 2;
            }
          }
        }
        return 0;
      }
      //! Read int from input string.
      //! @param[in] str input string.
      //! @param[out] dst number.
      //! @return true if successful, false otherwise.
      bool readIntFromString(const std::string& str, int& dst) {
        try {
          dst = std::stoi(str);
          return true;
        }
        catch (const std::invalid_argument& ia) {
          err(DTR("Invalid argument: %s"), ia.what());
          return false;
        }
        return true;
      }
      //! Interpret fishtag sentence.
      //! @param[in] parts vector of strings from sentence.
      void
      interpretTagDetection(const std::vector<std::string>& parts)
      {
        spew(DTR("Interpret tag"));
        if (parts.size() < c_tag_fields)
        {
          war(DTR("invalid tag sentence"));
          return;
        }
        
        int serial_no = 0;
        int unix_timestamp = 0;
        int millis = 0;
        int trans_protocol = 0;
        int trans_id = 0;
        int trans_data = 0;
        int SNR = 0;
        int trans_freq = 0;
        int recv_mem_addr = 0;

        if (readIntFromString(parts[0], serial_no))
        {
          // Receiver serial number
          spew(DTR("Serial number: %u"), serial_no);
        }
        if (readIntFromString(parts[1], unix_timestamp))
        {
          //UTC UNIX timestamp
          spew(DTR("UTC UNIX timestamp: %u"), unix_timestamp);
        }
        if (readIntFromString(parts[2], millis))
        {
          //Millisecond timestamp
          spew(DTR("Millisecond timestamp: %u"), millis);
        }

        //Transmit protocol
        if(parts[3] == "R256")
          trans_protocol = IMC::TBRFishTag::TBR_R256;
        else if(parts[3] == "R04K")
          trans_protocol = IMC::TBRFishTag::TBR_R04K;
        else if(parts[3] == "S64K")
          trans_protocol = IMC::TBRFishTag::TBR_S64K;
        else if(parts[3] == "R64K")
          trans_protocol = IMC::TBRFishTag::TBR_R64K;
        else if(parts[3] == "R01M")
          trans_protocol = IMC::TBRFishTag::TBR_R01M;
        else if(parts[3] == "S256")
          trans_protocol = IMC::TBRFishTag::TBR_S256;
        else if(parts[3] == "HS256")
          trans_protocol = IMC::TBRFishTag::TBR_HS256;
        else if(parts[3] == "DS256")
          trans_protocol = IMC::TBRFishTag::TBR_DS256;
        spew(DTR("Transmit protocol: %s, enum: %i"), parts[3].c_str(), trans_protocol);


        if (readIntFromString(parts[4], trans_id))
        {
          // Tag ID number
          spew(DTR("Tag ID: %u"), trans_id);
        }
        if (readIntFromString(parts[5], trans_data))
        {
          // Tag raw data
          spew(DTR("Tag raw data: %u"), trans_data);
        }
        if (readIntFromString(parts[6], SNR))
        {
          // Signal to noise ratio
          spew(DTR("SNR: %u"), SNR);
        }
        if (readIntFromString(parts[7], trans_freq))
        {
          // Signal frequency
          spew(DTR("Signal frequency: %u"), trans_freq);
        }
        if (readIntFromString(parts[8], recv_mem_addr))
        {
          // Receiver memory address
          spew(DTR("Receiver memory address: %u"), recv_mem_addr);
        }
        spew("%f, %f", std::stof(parts[9]), std::stof(parts[10]));

        m_prev_unix_timestamp = m_tag_msg.unix_timestamp;
        m_prev_millis = m_tag_msg.millis;

        m_tag_msg.serial_no = serial_no;
        m_tag_msg.unix_timestamp = unix_timestamp;
        m_tag_msg.millis = millis;
        m_tag_msg.trans_protocol = trans_protocol;
        m_tag_msg.trans_id = trans_id;
        m_tag_msg.trans_data = trans_data;
        m_tag_msg.snr = SNR;
        m_tag_msg.trans_freq = trans_freq;
        m_tag_msg.recv_mem_addr = recv_mem_addr;
        m_tag_msg.lat = DUNE::Math::Angles::radians(std::stof(parts[9]));
        m_tag_msg.lon = DUNE::Math::Angles::radians(std::stof(parts[10]));
        
        // Compile message output to Neptus/DUNE log
        
        m_tag_position.lat = m_tag_msg.lat;
        m_tag_position.lon = m_tag_msg.lon;
        //m_tag_position.data = std::to_string(m_tagDetection[0].unix_timestamp) + std::to_string(m_tagDetection[0].millis) + "," + std::to_string(m_tagDetection[1].unix_timestamp) + std::to_string(m_tagDetection[1].millis) + "," + std::to_string(m_tagDetection[2].unix_timestamp) + std::to_string(m_tagDetection[2].millis);
        // + "," + m_xkf.stage3.innov.norm_p(2) + "," << m_xkf.stage3.PHat.norm_p(2) + "," << m_xkf.stage3.PHat.trace();
        m_tag_position.id = "Receiver" + std::to_string(serial_no);
        
      }

      //! Interpret SensorReading sentence.
      //! @param[in] parts vector of strings from sentence.
      void
      interpretSensorReading(const std::vector<std::string>& parts) {
        spew(DTR("Interpret SensorReading"));
        if (parts.size() < c_tbr_sensor_fields)
        {
          war(DTR("invalid SensorReading sentence"));
          return;
        }

        int serial_no = 0;
        int unix_timestamp = 0;
        int temperature = 0;
        int avg_noise_level = 0;
        int peak_noise_level = 0;
        int recv_listen_freq = 0;
        int recv_mem_addr = 0;
        float temp_C = 0.0;

        if (readIntFromString(parts[0], serial_no))
        {
          // Receiver serial number
          spew(DTR("Serial number: %u"), serial_no);
        }
        if (readIntFromString(parts[1], unix_timestamp))
        {
          //UTC UNIX timestamp
          spew(DTR("UTC UNIX timestamp: %u"), unix_timestamp);
        }
        if (readIntFromString(parts[3], temperature))
        {
          // Temperature
          temp_C = float(temperature-50)/10.0;
          spew(DTR("Temperature(C): %f"), temp_C);
          IMC::Temperature temp_msg;
          temp_msg.setSourceEntity(255);
          temp_msg.value = fp32_t(temp_C);
          dispatch(temp_msg);
        }
        if (readIntFromString(parts[4], avg_noise_level))
        {
          // Average Noise Level
          spew(DTR("Average Noise Level: %u"), avg_noise_level);
        }
        if (readIntFromString(parts[5], peak_noise_level))
        {
          // Peak noise level
          spew(DTR("Peak noise level: %u"), peak_noise_level);
        }
        if (readIntFromString(parts[6], recv_listen_freq))
        {
          // Noise logging frequency
          spew(DTR("Noise logging frequency: %u"), recv_listen_freq);
        }
        if (readIntFromString(parts[7], recv_mem_addr))
        {
          // Receiver memory address
          spew(DTR("Receiver memory address: %u"), recv_mem_addr);
        }
        IMC::TBRSensor sensor_msg;
        sensor_msg.serial_no = serial_no;
        sensor_msg.unix_timestamp = unix_timestamp;
        sensor_msg.temperature = fp32_t(temp_C);
        sensor_msg.avg_noise_level = avg_noise_level;
        sensor_msg.peak_noise_level = peak_noise_level;
        sensor_msg.recv_listen_freq = recv_listen_freq;
        sensor_msg.recv_mem_addr = recv_mem_addr;
        dispatch(sensor_msg);
      }
      //! Main loop.
      void
      onMain(void)
      {
        Delay::waitMsec(2000);
        std::ifstream infile(m_args.msg_path);

        std::string line;
        while (std::getline(infile, line) && !stopping())
        {
            if(processSentence(line) == 2) { // Decode the line and enter if statement for tag messages
              if(m_args.realTimeDelay) {
                if(m_prev_unix_timestamp == 0) {
                  m_prev_unix_timestamp = m_tag_msg.unix_timestamp;
                } else {
                  Delay::waitMsec(m_args.realTimeMultiplier*((m_tag_msg.unix_timestamp - m_prev_unix_timestamp)*1000+(m_tag_msg.millis-m_prev_millis)));
                }
                
              } else {
                if(m_prev_unix_timestamp + 1 < m_tag_msg.unix_timestamp) {
                  Delay::waitMsec(m_args.msg_delay);
                }
              }
              debug("Dispatching TagID %u with original timestamp %u.%u", m_tag_msg.trans_id, m_tag_msg.unix_timestamp, m_tag_msg.millis);
              dispatch(m_tag_msg);
              dispatch(m_tag_position);
            }
        }

        while (!stopping())
        {
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
