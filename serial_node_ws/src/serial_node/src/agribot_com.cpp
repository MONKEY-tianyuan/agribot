#include "agribot_com.hpp"

#include "math.h"

#define STEP_RATIO 20
#define STEP_RESLN 400
#define TIM_SCALER 10500
#define pi 3.1415926535

int main(int agrc,char** agrgv){
    ros::init(agrc,agrgv,"agribot_com");
    ROS_INFO("agribot has turned on");
    agribot_com AGRIBOT_COM;
    AGRIBOT_COM.Control();
    return 0;

}

agribot_com::agribot_com():Sampling_Time(0){
    //memset(Robot_Pos_GPS,0,sizeof(Robot_Pos_GPS));
    //memset(Robot_Vel_GPS,0,sizeof(Robot_Pos_GPS));
    memset(Send_Data,0,sizeof(Send_Data));
    memset(&hubVel1,0,sizeof(hubVel1));
    memset(&hubVel2,0,sizeof(hubVel2));
    memset(&stepPos1,0,sizeof(stepPos1));
    memset(&stepPos2,0,sizeof(stepPos2));
    memset(&encoder_data,0,sizeof(encoder_data));
    ros::NodeHandle private_param("~");
    private_param.param<std::string>("usart_port_name",usart_port_name,"/dev/ttyUSB0");
    private_param.param<int>("seVel1ial_baudrate",serial_baudrate,115200);
    private_param.param<std::string>("robot_frame_id",robot_frame_id,"agribot");

    test_Pub = n.advertise<serial_node::ros_to_32>("/agribot_com",100);

    encoder_Pub = n.advertise<serial_node::stm2ros_tao>("/encoder",1000);
    //hub_Vel_Sub = n.subscribe("/agribot_com",10,&agribot_com::Cmd_hubVel_callback,this);
    hub_Vel_Sub = n.subscribe("/cmd_hub_vel",10,&agribot_com::Cmd_hubVel_callback,this);
    curtain_up_Sub = n.subscribe("/curtain_up",10,&agribot_com::curtain_up_callback,this);
    curtain_down_Sub = n.subscribe("/curtain_down",10,&agribot_com::curtain_down_callback,this);
    curtain_stay_Sub = n.subscribe("/curtain_stay",10,&agribot_com::curtain_stay_callback,this);

    step_Pos_Sub = n.subscribe("/cmd_step_pos",10,&agribot_com::Cmd_stepPos_callback,this);
    turn_Sub = n.subscribe("/cmd_turn",10,&agribot_com::Cmd_turn_callback,this);
    //step_Pos_Sub = n.subscribe("/agribot_com",10,&agribot_com::Cmd_stepPos_callback,this);
    ROS_INFO_STREAM("Ready...");
    try
        {
            {/* code */
            Stm32_ser.setPort(usart_port_name);
            Stm32_ser.setBaudrate(serial_baudrate);
            serial::Timeout to=serial::Timeout::simpleTimeout(1000);
            Stm32_ser.setTimeout(to);
            Stm32_ser.open();}
        }
        catch(serial::IOException &e)
        {
            ROS_INFO_STREAM("Failed to open port ");
        
        }
        if (Stm32_ser.isOpen()){
            ROS_INFO_STREAM("Succeed to open port ");
            }
        else{
            
        }

}

agribot_com::~agribot_com(){



}

void agribot_com::Control(){
    _Last_Time = ros::Time::now();
    seq = 1;
    while(ros::ok()){
        _Now = ros::Time::now();
        Sampling_Time = (_Now - _Last_Time).toSec();
        if(Stm32_ser.available()){
            //read serial port data
            size_t bytes_read = Stm32_ser.read(tmp_rec_buffer,Stm32_ser.available());
            // for (int i =0 ;i<bytes_read;i++){
            //     ROS_INFO("i: %x ",tmp_rec_buffer[i]);
            // }

            if(rec_index == 0 && tmp_rec_buffer[0] == rFrameHead){
                //rec_index = 0;
                
                for(int i=0;i<bytes_read;i++){
                    Receve_Buffer[rec_index++] = tmp_rec_buffer[i];
                    
                }
                //ROS_INFO("find head,%x,index: %d", Receve_Buffer[0],rec_index);
            }
            else if(rec_index != 0 && rec_index<7){
                for(int i=0;i<bytes_read;i++){
                    Receve_Buffer[rec_index++] = tmp_rec_buffer[i];  
                }
                //ROS_INFO("rec_index: %d",rec_index);
                

            }
            else if(rec_index >= 7){
                rec_index = 0;
                //ROS_INFO("Done to receive!");
                //ROS_INFO("%x",Receve_Buffer[9]);
                if(Receve_Buffer[5] == rFrameTail && Receve_Buffer[6] == rFrameTail2){
                    ROS_INFO("heard");
                    encoder3_stm.data8[0] = Receve_Buffer[1];
                    encoder3_stm.data8[1] = Receve_Buffer[2];
                    encoder3_stm.data8[2] = 0;
                    encoder3_stm.data8[3] = 0;   

                    encoder4_stm.data8[0] = Receve_Buffer[3];
                    encoder4_stm.data8[1] = Receve_Buffer[4];
                    encoder4_stm.data8[2] = 0;
                    encoder4_stm.data8[3] = 0;



                    // encoder_step1.data8[0] = Receve_Buffer[5];
                    // encoder_step1.data8[1] = Receve_Buffer[6];
                    // encoder_step1.data8[2] = 0;
                    // encoder_step1.data8[3] = 0;

                    // encoder_step2.data8[0] = Receve_Buffer[7];
                    // encoder_step2.data8[1] = Receve_Buffer[8];
                    // encoder_step2.data8[2] = 0;
                    // encoder_step2.data8[3] = 0;

                    

                    
                    encoder_data.encoder3 = encoder3_stm.data_short;
                    encoder_data.encoder4 = encoder4_stm.data_short;
                    encoder_data.encoder_step1 = encoder_step1.data_short;
                    encoder_data.encoder_step2 = encoder_step2.data_short;
                    //ROS_INFO("%d,%d",encoder_data.encoder3,encoder_data.encoder4);
                    //ROS_INFO("receive: %x %x %x %x %x %x",Receve_Buffer[0],Receve_Buffer[2],Receve_Buffer[3],Receve_Buffer[4],Receve_Buffer[5],Receve_Buffer[6]);
                    Publish_ENCODER();
                }
            }
            
        
        }
    //Publish_Test();
    while(Sampling_Time<0.01){
        _Now = ros::Time::now();
        Sampling_Time = (_Now - _Last_Time).toSec();
    }
    
    ros::spinOnce();
    }


}


void agribot_com::curtain_up_callback(const std_msgs::StringConstPtr& msg){
    uint8_t cnt = 0;
    stepScaler1.data_short = 1;
    
    Send_Data[cnt++] = hubVel1.data8[0];
    Send_Data[cnt++] = hubVel1.data8[1];
    Send_Data[cnt++] = hubVel1.data8[2];
    Send_Data[cnt++] = hubVel1.data8[3];

    Send_Data[cnt++] = hubVel2.data8[0];
    Send_Data[cnt++] = hubVel2.data8[1];
    Send_Data[cnt++] = hubVel2.data8[2];
    Send_Data[cnt++] = hubVel2.data8[3];

    Send_Data[cnt++] = stepPos1.data8[0];
    Send_Data[cnt++] = stepPos1.data8[1];
    Send_Data[cnt++] = stepPos1.data8[2];
    Send_Data[cnt++] = stepPos1.data8[3];

    Send_Data[cnt++] = stepPos2.data8[0];
    Send_Data[cnt++] = stepPos2.data8[1];
    Send_Data[cnt++] = stepPos2.data8[2];
    Send_Data[cnt++] = stepPos2.data8[3];

    Send_Data[cnt++] = stepScaler1.data8[0];
    Send_Data[cnt++] = stepScaler1.data8[1];
    Send_Data[cnt++] = stepScaler1.data8[2];
    Send_Data[cnt++] = stepScaler1.data8[3];

    Send_Data[cnt++] = stepScaler2.data8[0];
    Send_Data[cnt++] = stepScaler2.data8[1];
    Send_Data[cnt++] = stepScaler2.data8[2];
    Send_Data[cnt++] = stepScaler2.data8[3];

    Send_Data[cnt++] = 0x01;


    //Send_Data[cnt++] = sFrameTail;
    //uint8_t temp[3]={Send_Data[0],Send_Data[1],Send_Data[17]};
    //Stm32_ser.write(temp,sizeof(temp));
    Stm32_ser.write(Send_Data,25);
    ROS_INFO("Done %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %d",Send_Data[0],Send_Data[1],Send_Data[2],Send_Data[3],Send_Data[4],Send_Data[5],Send_Data[6],Send_Data[7],Send_Data[8],Send_Data[9],Send_Data[10],Send_Data[11],Send_Data[12],Send_Data[13],Send_Data[14],Send_Data[15],Send_Data[16],Send_Data[17],Send_Data[18],Send_Data[19],Send_Data[20],Send_Data[21],Send_Data[22],Send_Data[23],Send_Data[24],sizeof(Send_Data));
    //ROS_INFO_STREAM("Done Transmitting ... ");
    for (int i=0; i<25;i++){
            Send_Data[i]=0;
        }

}


void agribot_com::curtain_down_callback(const std_msgs::StringConstPtr& msg){
    
    uint8_t cnt = 0;
    stepScaler1.data_short = -1;
    
    Send_Data[cnt++] = hubVel1.data8[0];
    Send_Data[cnt++] = hubVel1.data8[1];
    Send_Data[cnt++] = hubVel1.data8[2];
    Send_Data[cnt++] = hubVel1.data8[3];

    Send_Data[cnt++] = hubVel2.data8[0];
    Send_Data[cnt++] = hubVel2.data8[1];
    Send_Data[cnt++] = hubVel2.data8[2];
    Send_Data[cnt++] = hubVel2.data8[3];

    Send_Data[cnt++] = stepPos1.data8[0];
    Send_Data[cnt++] = stepPos1.data8[1];
    Send_Data[cnt++] = stepPos1.data8[2];
    Send_Data[cnt++] = stepPos1.data8[3];

    Send_Data[cnt++] = stepPos2.data8[0];
    Send_Data[cnt++] = stepPos2.data8[1];
    Send_Data[cnt++] = stepPos2.data8[2];
    Send_Data[cnt++] = stepPos2.data8[3];

    Send_Data[cnt++] = stepScaler1.data8[0];
    Send_Data[cnt++] = stepScaler1.data8[1];
    Send_Data[cnt++] = stepScaler1.data8[2];
    Send_Data[cnt++] = stepScaler1.data8[3];

    Send_Data[cnt++] = stepScaler2.data8[0];
    Send_Data[cnt++] = stepScaler2.data8[1];
    Send_Data[cnt++] = stepScaler2.data8[2];
    Send_Data[cnt++] = stepScaler2.data8[3];

    Send_Data[cnt++] = 0x01;


    //Send_Data[cnt++] = sFrameTail;
    //uint8_t temp[3]={Send_Data[0],Send_Data[1],Send_Data[17]};
    //Stm32_ser.write(temp,sizeof(temp));
    Stm32_ser.write(Send_Data,25);
    ROS_INFO("Done %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %d",Send_Data[0],Send_Data[1],Send_Data[2],Send_Data[3],Send_Data[4],Send_Data[5],Send_Data[6],Send_Data[7],Send_Data[8],Send_Data[9],Send_Data[10],Send_Data[11],Send_Data[12],Send_Data[13],Send_Data[14],Send_Data[15],Send_Data[16],Send_Data[17],Send_Data[18],Send_Data[19],Send_Data[20],Send_Data[21],Send_Data[22],Send_Data[23],Send_Data[24],sizeof(Send_Data));
    //ROS_INFO_STREAM("Done Transmitting ... ");
    for (int i=0; i<25;i++){
            Send_Data[i]=0;
        }

}

void agribot_com::curtain_stay_callback(const std_msgs::StringConstPtr& msg){
    uint8_t cnt = 0;
    stepScaler1.data_short = 0;
    
    Send_Data[cnt++] = hubVel1.data8[0];
    Send_Data[cnt++] = hubVel1.data8[1];
    Send_Data[cnt++] = hubVel1.data8[2];
    Send_Data[cnt++] = hubVel1.data8[3];

    Send_Data[cnt++] = hubVel2.data8[0];
    Send_Data[cnt++] = hubVel2.data8[1];
    Send_Data[cnt++] = hubVel2.data8[2];
    Send_Data[cnt++] = hubVel2.data8[3];

    Send_Data[cnt++] = stepPos1.data8[0];
    Send_Data[cnt++] = stepPos1.data8[1];
    Send_Data[cnt++] = stepPos1.data8[2];
    Send_Data[cnt++] = stepPos1.data8[3];

    Send_Data[cnt++] = stepPos2.data8[0];
    Send_Data[cnt++] = stepPos2.data8[1];
    Send_Data[cnt++] = stepPos2.data8[2];
    Send_Data[cnt++] = stepPos2.data8[3];

    Send_Data[cnt++] = stepScaler1.data8[0];
    Send_Data[cnt++] = stepScaler1.data8[1];
    Send_Data[cnt++] = stepScaler1.data8[2];
    Send_Data[cnt++] = stepScaler1.data8[3];

    Send_Data[cnt++] = stepScaler2.data8[0];
    Send_Data[cnt++] = stepScaler2.data8[1];
    Send_Data[cnt++] = stepScaler2.data8[2];
    Send_Data[cnt++] = stepScaler2.data8[3];

    Send_Data[cnt++] = 0x01;


    //Send_Data[cnt++] = sFrameTail;
    //uint8_t temp[3]={Send_Data[0],Send_Data[1],Send_Data[17]};
    //Stm32_ser.write(temp,sizeof(temp));
    Stm32_ser.write(Send_Data,25);
    ROS_INFO("Done %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %d",Send_Data[0],Send_Data[1],Send_Data[2],Send_Data[3],Send_Data[4],Send_Data[5],Send_Data[6],Send_Data[7],Send_Data[8],Send_Data[9],Send_Data[10],Send_Data[11],Send_Data[12],Send_Data[13],Send_Data[14],Send_Data[15],Send_Data[16],Send_Data[17],Send_Data[18],Send_Data[19],Send_Data[20],Send_Data[21],Send_Data[22],Send_Data[23],Send_Data[24],sizeof(Send_Data));
    //ROS_INFO_STREAM("Done Transmitting ... ");
    for (int i=0; i<25;i++){
            Send_Data[i]=0;
        }

}

void agribot_com::Cmd_hubVel_callback(const serial_node::ros_hub_vel::ConstPtr& msg){
    uint8_t cnt = 0;

    hubVel1.data = msg->hub_vel1;
    hubVel2.data = msg->hub_vel2;
    Send_Data[cnt++] = hubVel1.data8[0];
    Send_Data[cnt++] = hubVel1.data8[1];
    Send_Data[cnt++] = hubVel1.data8[2];
    Send_Data[cnt++] = hubVel1.data8[3];

    Send_Data[cnt++] = hubVel2.data8[0];
    Send_Data[cnt++] = hubVel2.data8[1];
    Send_Data[cnt++] = hubVel2.data8[2];
    Send_Data[cnt++] = hubVel2.data8[3];

    Send_Data[cnt++] = stepPos1.data8[0];
    Send_Data[cnt++] = stepPos1.data8[1];
    Send_Data[cnt++] = stepPos1.data8[2];
    Send_Data[cnt++] = stepPos1.data8[3];

    Send_Data[cnt++] = stepPos2.data8[0];
    Send_Data[cnt++] = stepPos2.data8[1];
    Send_Data[cnt++] = stepPos2.data8[2];
    Send_Data[cnt++] = stepPos2.data8[3];

    Send_Data[cnt++] = stepScaler1.data8[0];
    Send_Data[cnt++] = stepScaler1.data8[1];
    Send_Data[cnt++] = stepScaler1.data8[2];
    Send_Data[cnt++] = stepScaler1.data8[3];

    Send_Data[cnt++] = stepScaler2.data8[0];
    Send_Data[cnt++] = stepScaler2.data8[1];
    Send_Data[cnt++] = stepScaler2.data8[2];
    Send_Data[cnt++] = stepScaler2.data8[3];

    Send_Data[cnt++] = 0x01;


    //Send_Data[cnt++] = sFrameTail;
    //uint8_t temp[3]={Send_Data[0],Send_Data[1],Send_Data[17]};
    //Stm32_ser.write(temp,sizeof(temp));
    Stm32_ser.write(Send_Data,25);
    ROS_INFO("Done %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %d",Send_Data[0],Send_Data[1],Send_Data[2],Send_Data[3],Send_Data[4],Send_Data[5],Send_Data[6],Send_Data[7],Send_Data[8],Send_Data[9],Send_Data[10],Send_Data[11],Send_Data[12],Send_Data[13],Send_Data[14],Send_Data[15],Send_Data[16],Send_Data[17],Send_Data[18],Send_Data[19],Send_Data[20],Send_Data[21],Send_Data[22],Send_Data[23],Send_Data[24],sizeof(Send_Data));
    //ROS_INFO_STREAM("Done Transmitting ... ");
    for (int i=0; i<25;i++){
            Send_Data[i]=0;
        }

}

void agribot_com::Cmd_stepPos_callback(const serial_node::ros_step_pav::ConstPtr& msg){
    uint8_t cnt = 0;
    
    //stepPos1.data_long = (uint32_t)(msg->step_pos1*(float)(STEP_RATIO)*(float)(STEP_RESLN)/360.0);
    //stepPos2.data_long = (uint32_t)(msg->step_pos2*(float)(STEP_RATIO)*(float)(STEP_RESLN)/360.0);
    stepPos1.data_long = (uint32_t)(msg->step_pos1*4800.0/360.0);
    stepPos2.data_long = (uint32_t)(msg->step_pos2*4800.0/360.0);
    stepScaler1.data_long = (uint32_t)(((float)(TIM_SCALER)/msg->step_vel1)-1);
    stepScaler2.data_long = (uint32_t)(((float)(TIM_SCALER)/msg->step_vel2)-1);

    Send_Data[cnt++] = hubVel1.data8[0];
    Send_Data[cnt++] = hubVel1.data8[1];
    Send_Data[cnt++] = hubVel1.data8[2];
    Send_Data[cnt++] = hubVel1.data8[3];

    Send_Data[cnt++] = hubVel2.data8[0];
    Send_Data[cnt++] = hubVel2.data8[1];
    Send_Data[cnt++] = hubVel2.data8[2];
    Send_Data[cnt++] = hubVel2.data8[3];

    Send_Data[cnt++] = stepPos1.data8[0];
    Send_Data[cnt++] = stepPos1.data8[1];
    Send_Data[cnt++] = stepPos1.data8[2];
    Send_Data[cnt++] = stepPos1.data8[3];

    Send_Data[cnt++] = stepPos2.data8[0];
    Send_Data[cnt++] = stepPos2.data8[1];
    Send_Data[cnt++] = stepPos2.data8[2];
    Send_Data[cnt++] = stepPos2.data8[3];

    Send_Data[cnt++] = stepScaler1.data8[0];
    Send_Data[cnt++] = stepScaler1.data8[1];
    Send_Data[cnt++] = stepScaler1.data8[2];
    Send_Data[cnt++] = stepScaler1.data8[3];

    Send_Data[cnt++] = stepScaler2.data8[0];
    Send_Data[cnt++] = stepScaler2.data8[1];
    Send_Data[cnt++] = stepScaler2.data8[2];
    Send_Data[cnt++] = stepScaler2.data8[3];

    Send_Data[cnt++] = 0x01;

    Stm32_ser.write(Send_Data,25);

    ROS_INFO("Done %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",Send_Data[0],Send_Data[1],Send_Data[2],Send_Data[3],Send_Data[4],Send_Data[5],Send_Data[6],Send_Data[7],Send_Data[8],Send_Data[9],Send_Data[10],Send_Data[11],Send_Data[12],Send_Data[13],Send_Data[14],Send_Data[15],Send_Data[16],Send_Data[17],Send_Data[18],Send_Data[19],Send_Data[20],Send_Data[21],Send_Data[22],Send_Data[23],Send_Data[24]);
    //ROS_INFO_STREAM("Done Transmitting ... ");
    step1_finish_flag = false;
    step2_finish_flag = false;
    for (int i=0; i<25;i++){
            Send_Data[i]=0;
        }
    
   // else ROS_INFO("steering not yet finished...");

}

void agribot_com::Cmd_turn_callback(const serial_node::ros_turn::ConstPtr& msg){
    uint8_t cnt = 0;
    //if(step1_finish_flag & step2_finish_flag){

    Steering_plane(msg->rotate_radius,msg->back_mid_vel);

    Send_Data[cnt++] = hubVel1.data8[0];
    Send_Data[cnt++] = hubVel1.data8[1];
    Send_Data[cnt++] = hubVel1.data8[2];
    Send_Data[cnt++] = hubVel1.data8[3];

    Send_Data[cnt++] = hubVel2.data8[0];
    Send_Data[cnt++] = hubVel2.data8[1];
    Send_Data[cnt++] = hubVel2.data8[2];
    Send_Data[cnt++] = hubVel2.data8[3];

    Send_Data[cnt++] = stepPos1.data8[0];
    Send_Data[cnt++] = stepPos1.data8[1];
    Send_Data[cnt++] = stepPos1.data8[2];
    Send_Data[cnt++] = stepPos1.data8[3];

    Send_Data[cnt++] = stepPos2.data8[0];
    Send_Data[cnt++] = stepPos2.data8[1];
    Send_Data[cnt++] = stepPos2.data8[2];
    Send_Data[cnt++] = stepPos2.data8[3];

    Send_Data[cnt++] = stepScaler1.data8[0];
    Send_Data[cnt++] = stepScaler1.data8[1];
    Send_Data[cnt++] = stepScaler1.data8[2];
    Send_Data[cnt++] = stepScaler1.data8[3];

    Send_Data[cnt++] = stepScaler2.data8[0];
    Send_Data[cnt++] = stepScaler2.data8[1];
    Send_Data[cnt++] = stepScaler2.data8[2];
    Send_Data[cnt++] = stepScaler2.data8[3];

    Send_Data[cnt++] = 0x01;

    Stm32_ser.write(Send_Data,25);

    ROS_INFO("Done %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",Send_Data[0],Send_Data[1],Send_Data[2],Send_Data[3],Send_Data[4],Send_Data[5],Send_Data[6],Send_Data[7],Send_Data[8],Send_Data[9],Send_Data[10],Send_Data[11],Send_Data[12],Send_Data[13],Send_Data[14],Send_Data[15],Send_Data[16],Send_Data[17],Send_Data[18],Send_Data[19],Send_Data[20],Send_Data[21],Send_Data[22],Send_Data[23],Send_Data[24]);
    //ROS_INFO_STREAM("Done Transmitting ... ");
    
    step1_finish_flag = false;
    step2_finish_flag = false;
    for (int i=0; i<25;i++){
            Send_Data[i]=0;
        }
    //}
    //else ROS_INFO("steering not yet finished...");

}


void agribot_com::Publish_ENCODER(){
    serial_node::stm2ros_tao encoder_msg;

    encoder_msg.header.seq = seq++;
    encoder_msg.header.stamp = ros::Time::now();

    
    encoder_msg.encoder3 = encoder_data.encoder3;
    encoder_msg.encoder4 = encoder_data.encoder4;

    encoder_msg.encoder_step1 = encoder_data.encoder_step1;
    encoder_msg.encoder_step2 = encoder_data.encoder_step2;

    encoder_msg.DAC_value1 = DAC_value1.data_ushort;
    encoder_msg.DAC_value2 = DAC_value2.data_ushort;
    encoder_msg.step1_finish_flag = step1_finish_flag;
    encoder_msg.step2_finish_flag = step2_finish_flag;
    encoder_Pub.publish(encoder_msg);
    //ROS_INFO("Pub Success...");
}


void agribot_com::Publish_Test(){
    serial_node::ros_to_32 msg;
    msg.hub_vel1=4.04;
    msg.hub_vel2=6.21;
    msg.step_pos1=20.44;
    msg.step_pos2=9.78;
    test_Pub.publish(msg);
}

void agribot_com::Steering_plane(float rotate_radius,float back_mid_vel){
	float dis_hori_front_left = (rotate_radius+agribot_com::dist_hori/2);
    float dis_hori_front_right = (rotate_radius-agribot_com::dist_hori/2);
    float r_front_left = sqrt(dis_hori_front_left*dis_hori_front_left+agribot_com::dist_vert*agribot_com::dist_vert);
    float r_front_right = sqrt(dis_hori_front_right*dis_hori_front_right+agribot_com::dist_vert*agribot_com::dist_vert);
    float theta_front_left = asin(agribot_com::dist_vert/r_front_left)/(2.0*pi);
    float theta_front_right = asin(agribot_com::dist_vert/r_front_right)/(2.0*pi);
    float v_front_left = r_front_left/rotate_radius*back_mid_vel;
    float v_front_right = r_front_right/rotate_radius*back_mid_vel;

    hubVel1.data = v_front_left;
    hubVel2.data = v_front_right;
    stepPos1.data_long = (uint32_t)(theta_front_left*2000.0f);
    stepPos2.data_long = (uint32_t)(theta_front_right*2000.0f);
    stepScaler1.data_long = (uint32_t)(((float)(TIM_SCALER)/10)-1);
    stepScaler2.data_long = (uint32_t)(((float)(TIM_SCALER)/10)-1);
    ROS_INFO("Rotate angle:%f %f",theta_front_left*360,theta_front_right*360);
}

