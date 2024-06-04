#ifndef _AGRIBOT_COM_HPP_
#define _AGRIBOT_COM_HPP_

#include <iostream>
#include <string>
#include <sstream>
using namespace std;


#include <stdint.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "serial/serial.h"
#include "serial_node/ros_to_32.h"
#include "serial_node/stm2ros_time.h"
#include "serial_node/stm2ros_tao.h"
#include "serial_node/stm_to_ros.h"
#include "serial_node/ros_hub_vel.h"
#include "serial_node/ros_step_pav.h"
#include "serial_node/ros_turn.h"
#include "stdbool.h"

typedef union{
    float data;
    uint32_t data_long;
    uint8_t data8[4];
    int32_t sdata_long;
    int16_t data_short;
    uint16_t data_ushort;
}data_u;

// typedef struct{
//     float x;
//     float y;
//     float z;
//     float yaw;
//     float pitch;
//     float roll;
// }Vel_Pos_DATA;

typedef struct{
    int16_t encoder3;
    int16_t encoder4;
    int16_t encoder_step1;
    int16_t encoder_step2;
}ENCODER_DATA;  

class agribot_com{
    public:
        agribot_com(); //constructor: to open serial and turn on machine
        ~agribot_com();//destructor: to close serial and stop machine
        void Control();
        serial::Serial Stm32_ser;

    private:
        //callback function 
        //multithread
        void Cmd_hubVel_callback(const serial_node::ros_hub_vel::ConstPtr& msg); //when successfully subscribing ros_to_32 msg, serial write hub motor velocity to STM32
        void Cmd_stepPos_callback(const serial_node::ros_step_pav::ConstPtr& msg); //when successfully subscribing ros_to_32 msg, serial write step motro position to STM32
        void Cmd_turn_callback(const serial_node::ros_turn::ConstPtr& msg);
        void curtain_up_callback(const std_msgs::StringConstPtr& msg);
        void curtain_down_callback(const std_msgs::StringConstPtr& msg);
        void curtain_stay_callback(const std_msgs::StringConstPtr& msg);

        void Steering_plane(float rotate_radius,float back_mid_vel);
        void Publish_ENCODER();
        void Publish_Test();
        unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode); //to check
        // Vel_Pos_DATA Robot_Pos_GPS;
        // Vel_Pos_DATA Robot_Vel_GPS;
        ENCODER_DATA encoder_data;

        int serial_baudrate;
        string usart_port_name, robot_frame_id;
        ros::NodeHandle n;
        ros::Time _Now, _Last_Time;
        float Sampling_Time;
        ros::Subscriber hub_Vel_Sub, step_Pos_Sub,turn_Sub,curtain_up_Sub,curtain_down_Sub,curtain_stay_Sub;
        ros::Publisher encoder_Pub ,test_Pub;
        uint8_t tmp_rec_buffer[1024] = {0};
        uint8_t Receve_Buffer[7] = {0};
        uint8_t Send_Data[25]={0};
        uint8_t rFrameHead = 0xDC;
        uint8_t rFrameTail = 0x0D;
        uint8_t rFrameTail2 = 0xBA;
        uint8_t sFrameHead = 0xCC;
        uint8_t sFrameTail = 0xDD;
        //uint8_t sFrameHead_step = 0xEE;
        //uint8_t sFrameTail_step = 0xFF;
        data_u hubVel1;
        data_u hubVel2;
        data_u stepPos1;
        data_u stepPos2;
        data_u stepScaler1;
        data_u stepScaler2;
        
        data_u encoder1_stm = {0};
        data_u encoder2_stm = {0};
        data_u encoder3_stm={0};
        data_u encoder4_stm={0};

        data_u encoder_step1={0};
        data_u encoder_step2={0};

        data_u DAC_value1 = {0};
        data_u DAC_value2 = {0};
        volatile bool step1_finish_flag = true;
        volatile bool step2_finish_flag = true;
        //update: 12/5/2022 
        //robot mechanical parameters

        //serial_node::stm2ros_time encoder_msg;
        uint32_t seq;
        uint8_t rec_index = 0;
        float dist_vert = 1.2319; //unit: m
        float dist_hori = 0.8636;
        //take center of back wheels as reference calculating point
        


};


#endif