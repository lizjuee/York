#ifndef COMM_TRANSFORM_H
#define COMM_TRANSFORM_H

#include <string>
#include <vector>
using namespace std;
/*-----------------------------------------------------------------
 * If use ros_info,add define ROS
 * --------------------------------------------------------------*/
#define ROS
//////////////////////////////////////////////////////////////////
#define HEAD 0
#define LEN 1
#define IDH 2
#define IDL 3
#define DATA0 4
#define DATA1 5
#define DATA2 6
#define DATA3 7
#define DATA4 8
#define DATA5 9
#define DATA6 10
#define DATA7 11
#define ADD 12
#define CHECK 13

#pragma pack(1)
typedef struct
{
    unsigned char order_head;
    unsigned char order_len;
    unsigned char order_id_h;
    unsigned char order_id_l;
    unsigned char order_data[8] = {0};
    unsigned char order_num;
    unsigned char order_sum;

}t_Order;
#pragma pack(0)

////////////////////////////info of vehicle//////////////////////////////////////
#define MODE_STANDBY 0
#define MODE_REMOTE 1
#define MODE_CAN 2

#define BEEP_OFF 0
#define BEEP_ON 1

struct InfoVehicle
{
    //vehicle info
    unsigned char s_series = 0x00;
    unsigned char s_number = 0;
    //hardware and software version
    std::string s_ver_main_hard = "--";
    std::string s_ver_main_soft = "--";
    std::string s_ver_driver_hard = "--";
    std::string s_ver_driver_soft = "--";
    //system status
    bool s_device_state = false;
    bool s_sys_state = true;    //0x00 fine,0x01 error
    bool s_beep_state = false;
    unsigned char s_sys_mode = 0x0f;     //wokring mode:0x00 standby,0x01 remote,0x02 CAN
    float s_bat_vol = 0.0;     // *0.1V
    bool s_err_motor_current = true;
    bool s_err_motor_heat = true;
    bool s_err_driver_heat = true;
    bool s_err_bat_down = true; //<22V
    bool s_err_bat_low = true;  //<23V
    bool s_err_ctrl_lost = true;
    bool s_err_bump = true;
    //motion status
    short s_speed_x = 0x0000;   //mm/s
    float s_rotate = 0.0;   //0.001rad/s
    short s_speed_y = 0x0000;   //mm/s
    //contraller status
    unsigned char s_ctrl_swa = 0x00;     //0x02 up,0x03 down
    unsigned char s_ctrl_swb = 0x00;     //0x02 up;0x01 mid,0x03 down
    unsigned char s_ctrl_swc = 0x00;     //0x02 up;0x01 mid,0x03 down
    unsigned char s_ctrl_swd = 0x00;     //0x02 up;0x03 down
    unsigned char s_ctrl_left_h = 0x00;  //-100~100
    unsigned char s_ctrl_left_v = 0x00;  //-100~100
    unsigned char s_ctrl_right_h = 0x00; //-100~100
    unsigned char s_ctrl_right_v = 0x00; //-100~100
    unsigned char s_ctrl_left_vra = 0x00; //-100~100
    unsigned char s_ctrl_right_vra = 0x00; //-100~100
    //odometer
    int s_odo_left = 0x00000000;   //mm
    int s_odo_right = 0x00000000;  //mm
    int s_odo_left_s = 0x00000000;  //mm
    int s_odo_right_s = 0x00000000;  //mm
    //bumper
    bool s_bump[8] = {1,1,1,1,1,1,1,1};
    //driver status
    short s_driver_speed[4] = {0};    //PRM
    float s_driver_current[4] = {0};  //*0.1A
    int s_driver_pulse[4] = {0};
    float s_driver_volt[4] = {0};     //*0.1V
    short s_driver_heat[4] = {0};     //1C
    char s_motor_heat[4] = {0};     //1C
    bool s_driver_err_volt_low[4] = {1,1,1,1};
    bool s_motor_err_volt_low[4] =  {1,1,1,1};
    bool s_motor_err_current_over[4] =  {1,1,1,1};
    bool s_driver_err_heat_over[4] =  {1,1,1,1};
};

//////////////////////////////////////////////////////////////////


class CommTransform
{
public:
    explicit CommTransform(void);
    InfoVehicle GetVehicleInfo(void);
    bool IsOnline(void);
    void DataRecvHandle(vector<unsigned char> data);
    static unsigned char DataCheckSum(vector<unsigned char> data);
    vector<unsigned char> StdVehicleReset();
    vector<unsigned char> StdVehicleClear();
    vector<unsigned char> StdVehicleVersion();
    vector<unsigned char> StdVehicleMode(unsigned char mode,bool beep);
    vector<unsigned char> StdVehiclemove(short speed_x,short speed_y,short rotate);
    vector<unsigned char> StdVehicleSetState(bool state);
    vector<unsigned char> StdVehicleConciseMode(bool state);

public:
    bool f_comm_on;
    bool f_vehicle_on;
    long m_data_err_count;
    long m_data_count;

private:
    vector<unsigned char> m_data_recv;
    vector<unsigned char> m_data_send;
    InfoVehicle m_info_vehicle;

private:
    void DataRecvTransform();

};

#endif // COMM_TRANSFORM_H
