#ifndef _DECODEFRAME_H_
#define _DECODEFRAME_H_
#include <stdint.h>
#include <string.h>
#include "serial/serial.h"

class DecodeFrame{
    public:

        DecodeFrame();
        //void serail_sub_callback(const );
        void CB_write_rs232_cycle(const ros::TimerEvent& e);
        void serial_sub_callback();
    private:
        uint8_t frameHead[3] = {0xa5,0x55,0x5a}; //define the frame head as: 0b0110 0101 0101 0101 0101 0110
        uint8_t frameEnd = 0xff;
};



#endif

