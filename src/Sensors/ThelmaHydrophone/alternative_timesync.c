/*
      void send_sync_msg(int32_t timestamp){
        spew(DTR("time: %i"), timestamp);
        uint8_t counter;
        uint8_t digit;
        uint8_t timestamp_array[9];
        uint8_t luhn_key;
        timestamp=timestamp/10; //last number is always zero, so it should not be sent
        for(counter=9; counter>=1;counter--){
          digit=timestamp%10;
          timestamp=timestamp/10;
          timestamp_array[counter-1]=digit;
        }
        std::string out = "";
        for(counter=0;counter<9;counter++){
          out += char((timestamp_array[counter])+'0');
        }
        luhn_key=create_luhn_number(timestamp_array);
        out += char(luhn_key+'0');
        spew(DTR("Sendsync: %s"), out.c_str());
      }

      uint8_t create_luhn_number(uint8_t timestamp_array2[]){
        uint32_t rx_timestamp = 0;
        uint8_t i = 0;
        uint32_t luhn_sum=0;
        for(i=0;i<9;i++){
          if(i%2==0){
            timestamp_array2[i]+=timestamp_array2[i];
          }
          if(timestamp_array2[i]>=10){
            luhn_sum+=timestamp_array2[i]%10+1;
          }
          else
            luhn_sum+=timestamp_array2[i];
        }
        char luhn_key=(luhn_sum*9)%10;
        
        return luhn_key;
      }

        //
        //uint8_t l2 = create_luhn_number((uint8_t*)UTCUnixTimestamp.c_str());
        //spew(DTR("Send: %u"), l2);
        #include <time.h>
        send_sync_msg((unsigned)time(NULL));
        //
        
*/
/*
      int calcLuhnVerifDigit(const char *number)
      {
          int i, sum, ch, num, twoup, len;

          len = std::strlen(number);
          sum = 0;
          twoup = 1;
          for (i = len - 1; i >= 0; --i) {
              ch = number[i];
              num = (ch >= '0' && ch <= '9') ? ch - '0' : 0;
              if (twoup) {
                  num += num;
                  if (num > 9) num = (num % 10) + 1;
              }
              sum += num;
              twoup = ++twoup & 1;
          }
          sum = 10 - (sum % 10);
          if (sum == 10) sum = 0;
          return sum;
      }
*/
