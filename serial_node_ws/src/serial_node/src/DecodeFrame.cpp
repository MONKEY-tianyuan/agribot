#include <stdint.h>
#include <string.h>
#include "serial/serial.h"
#include "DecodeFrame.hpp"

extern serial::Serial ser;
extern unsigned char s_buffer;e

void DecodeFrame::DecodeFrame(){
}

void DecodeFrame::CB_write_rs232_cycle(const ros::TimerEvent& e){

    ser.write(s_buffer,7);
    s_buffer[3] = 0;
    s_buffer[4] = 0;
    
}
