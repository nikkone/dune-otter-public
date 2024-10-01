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
// Author: Nikolai Lauvås (based on GPS by Ricardo Martins)                 *
//***************************************************************************
 

//TODO: Add feature: position at 5s before tag registration in imc message.
//TODO: May not work without PPS anymore.

// ISO C++ 98 headers.
#include <cstring>
#include <algorithm>
#include <cstddef>
#include <ctime> /* time_t, struct tm, time, mktime */
#include <string>
#include <sstream>
#include <inttypes.h>
#include <chrono>

// DUNE headers.
#include <DUNE/DUNE.hpp>
//#include <DUNE/Time/Clock.hpp>

// Local headers.
#include "Reader.hpp"
#include "commands.hpp"

// External Libraries Headers
#include <boost/circular_buffer.hpp>

namespace Sensors
{
  //! Device driver for ThelmaHydrophone
  namespace ThelmaHydrophone
  {
    using DUNE_NAMESPACES;

    //! Maximum number of initialization commands.
    static const unsigned c_max_init_cmds = 14;
    //! Timeout for waitReply() function.
    static const float c_wait_reply_tout = 4.0;
    //! Power on delay.
    static const double c_pwr_on_delay = 5.0;
    //! Number of fields in fish tag message
    static const unsigned c_tag_fields = 9;
    //! Number of fields in TBR-700RT sensor reading
    static const unsigned c_tbr_sensor_fields = 8;
    //! Message used to sync Thelma hydrophones
    static const std::string syncString = "(+)";
    //! Message used to sync Thelma hydrophones
    static const unsigned c_GpsFix_buffer_size = 11;

    struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;
      //! Input timeout in seconds.
      float inp_tout;
      //! Initialization commands.
      std::string init_cmds[c_max_init_cmds];
      //! Initialization replies.
      std::string init_rpls[c_max_init_cmds];
      //! Power channels.
      std::vector<std::string> pwr_channels;
      //! Write full unix timestamp every timestamp_send_divider times task is run.
      unsigned int timestamp_send_divider;
      //! Sync Period;
      double sync_period;
      //! Sync Period ACK timeout
      double sync_period_ack_timeout;

      unsigned transmission_time;

      bool usingPPS;

      std::string log_folder;

    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Serial port handle.
      IO::Handle* m_handle;
      //! Task arguments.
      Arguments m_args;
      //! Last initialization line read.
      std::string m_init_line;
      //! TBRReader thread.
      TBRReader* m_TBRReader;
      //! How often the full unix timestamp is sent, in executions % counter
      unsigned int timestamp_send_counter;
      //! Timer.
      Time::Counter<float> m_sync_timer;
      //! Timer.
      Time::Counter<float> m_syncack_timer;
      //! Buffer storing recent GpsFix messages
      boost::circular_buffer<IMC::GpsFix> *m_GPSBuffer;
      //!
      bool waitingForAck3;
      //! Filename for rawlog
      std::string filename;
      //! Most recent "Number of strings sent since power up" received
      uint16_t recent_received_string_nr;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_handle(NULL),
        m_TBRReader(NULL),
        recent_received_string_nr(0)
      {
        // Define configuration parameters.
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("")
        .description("Serial port device used to communicate with the sensor");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("115200")
        .description("Serial port baud rate");

        param("Sync Period", m_args.sync_period)
        .units(Units::Second)
        .defaultValue("10.0")
        .minimumValue("0.0")
        .description("Period between sync messages");

        param("Sync Period Ack Timeout", m_args.sync_period_ack_timeout)
        .units(Units::Second)
        .defaultValue("5.0")
        .minimumValue("0.0")
        .description("Ack should return within before timeout, or error will be output.");

        param("Write full timestamp divider", m_args.timestamp_send_divider)
        .defaultValue("6")
        .minimumValue("1")
        .description("Write full unix timestamp every timestamp_send_divider times task is run. (Not used in PPS mode.)");

        param("Input Timeout", m_args.inp_tout)
        .units(Units::Second)
        .defaultValue("4.0")
        .minimumValue("0.0")
        .description("Input timeout");

        param("Transmission Time", m_args.transmission_time)
        .units(Units::Second)
        .defaultValue("3")
        .minimumValue("0")
        .description("Transmission Time used for finding position");

        param("Use PPS", m_args.usingPPS)
        .defaultValue("false")
        .description("If the sensor uses PPS instead of sync messages.");

        param("Power Channel - Names", m_args.pwr_channels)
        .defaultValue("")
        .description("Device's power channels");

        for (unsigned i = 0; i < c_max_init_cmds; ++i)
        {
          std::string cmd_label = String::str("Initialization String %u - Command", i);
          param(cmd_label, m_args.init_cmds[i])
          .defaultValue("");

          std::string rpl_label = String::str("Initialization String %u - Reply", i);
          param(rpl_label, m_args.init_rpls[i])
          .defaultValue("");
        }

        param("Rawlogfile folder", m_args.log_folder)
        .defaultValue("log/")
        .description("The folder to place the raw logfiles in.");

        bind<IMC::DevDataText>(this);
        bind<IMC::IoEvent>(this);
        bind<IMC::GpsFix>(this);
      }
      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.sync_period)) {
          //sendFullTimestamp();
          m_sync_timer.setTop(m_args.sync_period);
        }
          

        if(paramChanged(m_args.sync_period_ack_timeout)) {
          m_syncack_timer.setTop(m_args.sync_period_ack_timeout);
          waitingForAck3 = false;
        }
      }
      void
      onResourceAcquisition(void)
      {
        m_GPSBuffer = new boost::circular_buffer<IMC::GpsFix>(c_GpsFix_buffer_size);

        if (m_args.pwr_channels.size() > 0)
        {
          IMC::PowerChannelControl pcc;
          pcc.op = IMC::PowerChannelControl::PCC_OP_TURN_ON;
          for (size_t i = 0; i < m_args.pwr_channels.size(); ++i)
          {
            pcc.name = m_args.pwr_channels[i];
            dispatch(pcc);
          }
        }

        Counter<double> timer(c_pwr_on_delay);
        while (!stopping() && !timer.overflow())
          waitForMessages(timer.getRemaining());

        try
        {
          if (!openSocket())
            m_handle = new SerialPort(m_args.uart_dev, m_args.uart_baud);

          m_TBRReader = new TBRReader(this, m_handle);
          m_TBRReader->start();
        }
        catch (...)
        {
          throw RestartNeeded(DTR("1"), 5);
        }

      }

      bool
      openSocket(void)
      {
        char addr[128] = {0};
        unsigned port = 0;

        if (std::sscanf(m_args.uart_dev.c_str(), "tcp://%[^:]:%u", addr, &port) != 2)
          return false;

        TCPSocket* sock = new TCPSocket;
        sock->connect(addr, port);
        m_handle = sock;
        return true;
      }

      void
      onResourceRelease(void)
      {
        if (m_TBRReader != NULL)
        {
          m_TBRReader->stopAndJoin();
          delete m_TBRReader;
          m_TBRReader = NULL;
        }

        Memory::clear(m_handle);
      }

      void
      onResourceInitialization(void)
      {
        filename = m_args.log_folder + "tbr_ " + std::to_string(std::time(nullptr)) + ".thelma";
        bool configuration_mode = false;
        for (unsigned i = 0; i < c_max_init_cmds; ++i)
        {
          if (m_args.init_cmds[i].empty())
            continue;
          // Try to enter the command mode. Only executed on the first command
          if(!configuration_mode) {
            slowTbrSend(TBCMD_ENTER_COMMAND_MODE);
            if (!waitForReply(TBCMD_ENTER_COMMAND_MODE_RESPONSE)) {
              war("%s: %s", DTR("Could not enter command mode with "), TBCMD_ENTER_COMMAND_MODE);
              war("Trying a command to see if the device is already in command mode.");
            }
            configuration_mode = true;
          }
          // Send initialization string and wait for answer
          std::string cmd = String::unescape(m_args.init_cmds[i]);
          m_handle->writeString(cmd.c_str());

          if (!m_args.init_rpls[i].empty())
          {
            std::string rpl = String::unescape(m_args.init_rpls[i]);
            if (!waitForReply(rpl))
            {
              err("%s: %s", DTR("No/Unexpected reply to command"), m_args.init_cmds[i].c_str());
              throw std::runtime_error(DTR("Failed to setup device"));
            }
          }
        }
        // If the command mode command was sent the this returns the device to listening mode
        if(configuration_mode) {
            commandTbrSend(TBCMD_EXIT_COMMAND_MODE);
            if (!waitForReply(TBCMD_EXIT_COMMAND_MODE_RESPONSE))
            {
              err("%s: %s", DTR("No/Unexpected reply to command"), TBCMD_EXIT_COMMAND_MODE);
              throw std::runtime_error(DTR("Failed to leave command mode"));
            }
            configuration_mode = true;
        }

        
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        if(m_args.usingPPS) {
          debug("Using PPS");
          //sendFullTimestamp();
        } else {
          debug("Waiting to start timer to dividable by 10.");
          while(std::time(0) % 10 != 0);
          m_sync_timer.setTop(m_args.sync_period);
          sendTbrTimestampSync();
          debug("Finished waiting to start timer to dividable by 10.");
        }
      }

      void
      consume(const IMC::DevDataText* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        spew("%s", sanitize(msg->value).c_str());

      std::ofstream logOutStream;
      logOutStream.open(filename, std::fstream::app);
      if (logOutStream.good()) {
        logOutStream.precision(15);
          logOutStream << msg->value << std::endl;
          logOutStream.close();
      } else {
        war("Could not write to rawlog: %s", filename.c_str());
      }

        if (getEntityState() == IMC::EntityState::ESTA_BOOT)
          m_init_line = msg->value;
        else
          processSentence(msg->value);
      }

      void
      consume(const IMC::IoEvent* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        if (msg->type == IMC::IoEvent::IOV_TYPE_INPUT_ERROR)
          throw RestartNeeded(msg->error, 5);
      }

      void
      consume(const IMC::GpsFix* msg)
      {
        if (msg->getSource() != getSystemId())
          return;
        m_GPSBuffer->push_back(*msg);
        //inf("%lu", m_GPSBuffer->size());
      }

      //! Wait reply to initialization command.
      //! @param[in] stn string to compare.
      //! @return true on successful match, false otherwise.
      bool
      waitForReply(const std::string& stn)
      {
        Counter<float> counter(c_wait_reply_tout);
        while (!stopping() && !counter.overflow())
        {
          waitForMessages(counter.getRemaining());
          if (m_init_line == stn)
          {
            m_init_line.clear();
            return true;
          }
        }

        return false;
      }

      void sendFullTimestamp() {
        std::time_t timestamp = std::time(nullptr) + 1;
        std::string UTCUnixTimestamp = "UT=" + std::to_string(timestamp);
        m_syncack_timer.reset(); // Resused to set an upper limit to waiting
        while((std::time(nullptr) !=timestamp) && !m_syncack_timer.overflow());
        if(!m_syncack_timer.overflow())
          slowTbrSend(UTCUnixTimestamp);
        else {
          war("Something went wrong when sending timestamp, retrying.");
          sendFullTimestamp();
        }

        //printf("%" PRIu64 "\n", ram);
        m_syncack_timer.reset();
        waitingForAck3 = true;

      }

      uint8_t calcLuhnVerifDigit(uint32_t timestamp) // From TB Live datasheet, fw1.0.1 rev.1
      { 
        uint16_t digitSum = 0;
        uint32_t digit = 0;
        for(uint8_t i = 0; i < 9; i ++) {
          timestamp /= 10;
          digit = timestamp % 10;
          if( (i % 2) == 0 ) {
            digit *= 2;
          }
          if(digit > 9) {
            digit -= 9 ;
          }
          digitSum += digit;
        }
        uint8_t luhnsCheckDigit = ( digitSum * 9) % 10;
        return luhnsCheckDigit;
      }

      void sendTbrTimestampSync() {
        // Get timestamp from system clock
        std::time_t timestamp = std::time(nullptr);
        // Remove last digit
        std::string UTCUnixTimestamp = std::to_string(timestamp/10);
        // Add Luhn verification number
        UTCUnixTimestamp += std::to_string(calcLuhnVerifDigit(timestamp));
        // Add preamble and send
        slowTbrSend(syncString + UTCUnixTimestamp);
      }

      void sendTbrSync() {
        slowTbrSend(syncString);
      }
      void commandTbrSend(std::string cmd) {
        m_handle->write(cmd.c_str(), cmd.size());
      }

      void slowTbrSend(std::string cmd) {
        // Send to uart slowly, because ThelmaHydrophone processes max 1 char per millisecond
        char a[1] = {'0'};
        for(char& c : cmd) {
            a[0] = c;
            m_handle->write(a, 1);
            Delay::waitMsec(1);
        }
        uint64_t microseconds_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        spew("Sent: \"%s\" at \"%" PRIu64 "\n", cmd.c_str(),microseconds_since_epoch);
        //spew(DTR("Sent: \"%s\" at \"%ld\""), cmd.c_str(), std::time(0)); 
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

      //! Process sentence.
      //! @param[in] line line.
      void
      processSentence(const std::string& line)
      {
        spew(DTR("Process"));
        if (line.find("ack01") != std::string::npos) {
          trace(DTR("Sensor clock diciplined."));
        } if(line.find("ack02") != std::string::npos) {
          trace(DTR("Sensor timestamp set."));
        } if(line.find("ack03") != std::string::npos) {
          if(waitingForAck3) {
            waitingForAck3 = false;
            trace(DTR("Expected sensor timestamp in PPS mode set."));
          } else {
            war("Unexpected ack03 received.");
          }
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
            return;
          }

          interpretSentence(parts);
        }
      }


      bool runningNrCheck(uint16_t running_nr) {
        if(recent_received_string_nr+1 != running_nr) {
          err("Unexpected message running number, expected: %u, received %u", recent_received_string_nr+1, running_nr);
          recent_received_string_nr = running_nr;
          return false;
        }
          recent_received_string_nr = running_nr;
          return true;
      }

      //! Interpret given sentence.
      //! @param[in] parts vector of strings from sentence.
      void
      interpretSentence(std::vector<std::string>& parts)
      {
        spew(DTR("Interpret"));
        
        if(parts.size() >= 3) {
          if(parts[2]  == "TBR Sensor") {
            interpretSensorReading(parts);
          } else {
            interpretTagDetection(parts);
          }
        }
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
          runningNrCheck(recv_mem_addr);
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
        else if(parts[3] == "S64K") {
          trans_protocol = IMC::TBRFishTag::TBR_S64K;
          if (readIntFromString(parts[5], trans_data))
          {
            // Tag raw data
            spew(DTR("Tag raw data: %u"), trans_data);
          }
        }
        else if(parts[3] == "R64K")
          trans_protocol = IMC::TBRFishTag::TBR_R64K;
        else if(parts[3] == "R01M")
          trans_protocol = IMC::TBRFishTag::TBR_R01M;
        else if(parts[3] == "S256") {
          trans_protocol = IMC::TBRFishTag::TBR_S256;
          if (readIntFromString(parts[5], trans_data))
          {
            // Tag raw data
            spew(DTR("Tag raw data: %u"), trans_data);
          }
        }
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

        // Only read sensor for tags with sensor, done in transmit protocol part

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
          runningNrCheck(recv_mem_addr);
        }
        IMC::TBRFishTag tag_msg;
        tag_msg.serial_no = serial_no;
        tag_msg.unix_timestamp = unix_timestamp;
        tag_msg.millis = millis;
        tag_msg.trans_protocol = trans_protocol;
        tag_msg.trans_id = trans_id;
        tag_msg.trans_data = trans_data;
        tag_msg.snr = SNR;
        tag_msg.trans_freq = trans_freq;
        tag_msg.recv_mem_addr = recv_mem_addr;

        unsigned buffer_index = findPosition(unix_timestamp, m_args.transmission_time);
        tag_msg.lat = (*m_GPSBuffer)[buffer_index].lat;
        tag_msg.lon = (*m_GPSBuffer)[buffer_index].lon;
        dispatch(tag_msg);
      }

      unsigned findPosition(unsigned tagTimestamp, unsigned delay) {
        std::chrono::seconds offset= std::chrono::seconds(delay);
        std::chrono::seconds tagtime= std::chrono::seconds(tagTimestamp)-offset;

        //std::chrono::seconds tagtime= std::chrono::duration_cast<std::chrono::seconds>(std::chrono::seconds((int)(*m_GPSBuffer)[m_GPSBuffer->size()-1].getTimeStamp()));
        //inf("%li",tagtime.count());
        // Initialize with oldest tag detection.
        int minDiff = abs(std::chrono::duration_cast<std::chrono::seconds>(tagtime-std::chrono::seconds((int)(*m_GPSBuffer)[c_GpsFix_buffer_size-m_GPSBuffer->size()].getTimeStamp()) -offset).count());

        // Iterate from newest to second oldest (high index to low)
          for(unsigned index = c_GpsFix_buffer_size-1;index >c_GpsFix_buffer_size-m_GPSBuffer->size();--index) {
          //inf("finding %i, %f", index, (*m_GPSBuffer)[index].getTimeStamp());

          std::chrono::seconds gpstime= std::chrono::seconds((int)(*m_GPSBuffer)[index].getTimeStamp());
          auto diff = std::chrono::duration_cast<std::chrono::seconds>(tagtime-gpstime);
          //inf("Absdiff: %li, mindiff: %i", abs(diff.count()), minDiff);

          if(abs(diff.count()) < minDiff) {
            minDiff=abs(diff.count());
          }
          else {
            if (diff.count()>0) { // If older detection is farther away in time, return the one step newer
              //spew("Found closest to be %f", (*m_GPSBuffer)[index +1].getTimeStamp());
              return index +1;
            }
          }
        }

        war("Could not find better position than oldest");
        return c_GpsFix_buffer_size-m_GPSBuffer->size();
      }

      void
      onMain(void)
      {
        sendFullTimestamp();
        while(!stopping()) {

          if(m_sync_timer.overflow())
          {
            m_sync_timer.reset();
            if(m_args.usingPPS) {
              sendFullTimestamp();
            } else {
              if(timestamp_send_counter >= m_args.timestamp_send_divider) {
                sendTbrTimestampSync();
                timestamp_send_counter = 0;
              } else {
                sendTbrSync();
              }
              //spew("C: %ld", std::time(0));
              spew("Sending duration: %f", m_sync_timer.getElapsed());
              timestamp_send_counter++;
            }
          }
          if(waitingForAck3) {
            if(m_syncack_timer.overflow()) {
              err("ack03 not received within %f", m_args.sync_period_ack_timeout);
              waitingForAck3 = false;
            }
          }
          //consumeMessages();
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
