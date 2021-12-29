#include <comm_transform.h>
#include <string>
#include <vector>
#ifdef ROS
#include <ros/ros.h>
#endif

CommTransform::CommTransform()
{
    m_data_recv.clear();
    m_data_send.clear();
    m_data_count = 0;
    m_data_err_count = 0;
    f_comm_on = false;
    f_vehicle_on = false;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: GetVehicleInfo
 * detail: return vehicle info
 -----------------------------------------------------------------------------------------------------------------*/
InfoVehicle CommTransform::GetVehicleInfo()
{
    return m_info_vehicle;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: IsOnline
 * detail: return if vehicle is online,use before send cmd
 -----------------------------------------------------------------------------------------------------------------*/
bool CommTransform::IsOnline(void)
{
    if(!f_comm_on)return false;
    if(!f_vehicle_on)return false;
    return true;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DataRecvHandle
 * detail: Save data to buffer
 -----------------------------------------------------------------------------------------------------------------*/
void CommTransform::DataRecvHandle(vector<unsigned char> data)
{
    if(data.size() == 0) return;
    m_data_recv.insert(m_data_recv.end(),data.begin(),data.end());
    this->DataRecvTransform();
    if(m_data_recv.size() > 2000)m_data_recv.clear();
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DataCheckSum
 * detail: Sum all byte and check
 -----------------------------------------------------------------------------------------------------------------*/
 unsigned char CommTransform::DataCheckSum(vector<unsigned char> data)
{
    unsigned char checksum = 0x00;
    for(int i = 0 ; i < (data.size()-1); i++)
    {
        checksum += data[i];
    }
    return checksum;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DataRecvTransform
 * detail: Transform data from serial buffer to std status
 -----------------------------------------------------------------------------------------------------------------*/
void CommTransform::DataRecvTransform()
{
    vector<unsigned char> buffer;
    t_Order recv_buff;
    short temp;
    int i;
    for(i=0;i<m_data_recv.size()-1;i++)
    {       
        /////////////////////////////////////////////////data cheak////////////////////////////////////////////////////////
        if(m_data_recv[i] != 0x55)continue;  //head cheak
        m_data_count++;
        if(m_data_count>100000)
        {
            m_data_count = 0;
            m_data_err_count = 0;
        }
        buffer.clear();
        if(m_data_recv.size()-i < m_data_recv.at(i+1)) return;  //breaked data
        buffer.insert(buffer.begin(),m_data_recv.begin()+i,m_data_recv.begin()+i+m_data_recv.at(i+1));        
        i += m_data_recv.at(i+1)-1;
        if(buffer.size() <= 4) continue;    //too less data
        if(DataCheckSum(buffer) != (unsigned char)buffer.at(buffer.size()-1))    //sum cheak
        {
            m_data_err_count++;
            ROS_ERROR("data error");
            continue;
        }
        if(buffer.size() != 14) continue;
        memcpy(&recv_buff,buffer.data(),14);
        f_comm_on = true;
        ///////////////////////////////////////general callback/////////////////////////////////////////////////                
        if(recv_buff.order_id_h == 0x00)    //0x00xx----------sys order
        {
            if(recv_buff.order_id_l == 0x22) //0x0022----------sys version
            {
                m_info_vehicle.s_ver_main_hard = std::to_string(buffer.at(4)) + "." + std::to_string(buffer.at(5));
                m_info_vehicle.s_ver_driver_hard = std::to_string(buffer.at(6)) + "." + std::to_string(buffer.at(7));
                m_info_vehicle.s_ver_main_soft = std::to_string(buffer.at(8)) + "." + std::to_string(buffer.at(9));
                m_info_vehicle.s_ver_driver_soft = std::to_string(buffer.at(10)) + "." + std::to_string(buffer.at(11));
                //ROS_INFO("%s",m_vehicle_info.s_ver_driver_soft.data());
            }
        }
        ///////////////////////////////////////vehicle callback/////////////////////////////////////////////////
        if(recv_buff.order_id_h == 0x01)    //0x01xx----------vehicle order
        {
            //ROS_INFO("%X--%X",recv_buff.order_id_h,recv_buff.order_id_l);
            /************************0x01X1 heart beat***************************************/
            if((recv_buff.order_id_l & 0x0f) == 0x01)
            {
                m_info_vehicle.s_number = (recv_buff.order_id_l>>4)&0x0f;
                m_info_vehicle.s_series = recv_buff.order_data[0];
                m_info_vehicle.s_device_state = recv_buff.order_data[1];
                f_vehicle_on = true;
                ROS_INFO("beat:%d",(int)(m_info_vehicle.s_device_state));
            }
            /************************0x01X2 sys status***************************************/
            if((recv_buff.order_id_l & 0x0f) == 0x02)
            {
                m_info_vehicle.s_sys_state = recv_buff.order_data[0];
                m_info_vehicle.s_sys_mode = recv_buff.order_data[1];
                temp = (recv_buff.order_data[2]<<8 & 0xff00) | (recv_buff.order_data[3] & 0x00ff);
                m_info_vehicle.s_bat_vol = ((float)temp)/10;
                m_info_vehicle.s_beep_state = recv_buff.order_data[4];
                m_info_vehicle.s_err_motor_current = recv_buff.order_data[5] & 0x01;
                m_info_vehicle.s_err_motor_heat = (recv_buff.order_data[5]>>1) & 0x01;
                m_info_vehicle.s_err_driver_heat = (recv_buff.order_data[5]>>2) & 0x01;
                m_info_vehicle.s_err_bat_down = (recv_buff.order_data[5]>>3) & 0x01;
                m_info_vehicle.s_err_bat_low = (recv_buff.order_data[5]>>4) & 0x01;
                m_info_vehicle.s_err_ctrl_lost = (recv_buff.order_data[5]>>5) & 0x01;
                m_info_vehicle.s_err_bump = (recv_buff.order_data[5]>>6) & 0x01;
                //ROS_INFO("%d",(m_vehicle_info.s_sys_mode));
            }
            /************************0x01X3 motion status***************************************/
            if((recv_buff.order_id_l & 0x0f) == 0x03)
            {
                m_info_vehicle.s_speed_x =(recv_buff.order_data[0]<<8 & 0xff00) | (recv_buff.order_data[1] & 0x00ff);
                m_info_vehicle.s_speed_y =(recv_buff.order_data[2]<<8 & 0xff00) | (recv_buff.order_data[3] & 0x00ff);
                temp = (recv_buff.order_data[4]<<8 & 0xff00) | (recv_buff.order_data[5] & 0x00ff);
                m_info_vehicle.s_rotate = ((float)temp)/1000;
                //ROS_INFO("%f",(m_vehicle_info.s_rotate));
            }

            /************************0x01X4 main odom***************************************/
            if((recv_buff.order_id_l & 0x0f) == 0x04)
            {
                m_info_vehicle.s_odo_left = (recv_buff.order_data[0]<<24 & 0xff000000)
                        | (recv_buff.order_data[1]<<16 & 0x00ff0000)
                        | (recv_buff.order_data[2]<<8 & 0x0000ff00)
                        | (recv_buff.order_data[3] & 0x000000ff);
                m_info_vehicle.s_odo_right = (recv_buff.order_data[4]<<24 & 0xff000000)
                        | (recv_buff.order_data[5]<<16 & 0x00ff0000)
                        | (recv_buff.order_data[6]<<8 & 0x0000ff00)
                        | (recv_buff.order_data[7] & 0x000000ff);
                 //ROS_INFO("%d",(m_vehicle_info.s_odo_left));

            }
            /************************0x01X5 secondary odom***************************************/
            if((recv_buff.order_id_l & 0x0f) == 0x05)
            {
                m_info_vehicle.s_odo_left_s = (recv_buff.order_data[0]<<24 & 0xff000000)
                        | (recv_buff.order_data[1]<<16 & 0x00ff0000)
                        | (recv_buff.order_data[2]<<8 & 0x0000ff00)
                        | (recv_buff.order_data[3] & 0x000000ff);
                m_info_vehicle.s_odo_right_s = (recv_buff.order_data[4]<<24 & 0xff000000)
                        | (recv_buff.order_data[5]<<16 & 0x00ff0000)
                        | (recv_buff.order_data[6]<<8 & 0x0000ff00)
                        | (recv_buff.order_data[7] & 0x000000ff);
                 //ROS_INFO("%d",(m_vehicle_info.s_odo_left_s));

            }
            /************************0x01X6 ctrl status***************************************/
            if((recv_buff.order_id_l & 0x0f) == 0x06)
            {
                m_info_vehicle.s_ctrl_swa =recv_buff.order_data[0] & 0x03;
                m_info_vehicle.s_ctrl_swb =(recv_buff.order_data[0]>>2) & 0x03;
                m_info_vehicle.s_ctrl_swc =(recv_buff.order_data[0]>>4) & 0x03;
                m_info_vehicle.s_ctrl_swd =(recv_buff.order_data[0]>>6) & 0x03;
                m_info_vehicle.s_ctrl_left_h = recv_buff.order_data[1];
                m_info_vehicle.s_ctrl_left_v = recv_buff.order_data[2];
                m_info_vehicle.s_ctrl_right_h = recv_buff.order_data[3];
                m_info_vehicle.s_ctrl_right_v = recv_buff.order_data[4];
                m_info_vehicle.s_ctrl_left_vra = recv_buff.order_data[5];
                m_info_vehicle.s_ctrl_right_vra = recv_buff.order_data[6];
                //ROS_INFO("%d",(m_vehicle_info.s_ctrl_left_h));
            }
            /************************0x01X7 safety sensor***************************************/
            if((recv_buff.order_id_l & 0x0f) == 0x07)
            {
                m_info_vehicle.s_bump[0] = recv_buff.order_data[0] & 0x01;
                m_info_vehicle.s_bump[1] = (recv_buff.order_data[0]>>1) & 0x01;
                m_info_vehicle.s_bump[2] = (recv_buff.order_data[0]>>2) & 0x01;
                m_info_vehicle.s_bump[3] = (recv_buff.order_data[0]>>3) & 0x01;
                m_info_vehicle.s_bump[4] = (recv_buff.order_data[0]>>4) & 0x01;
                m_info_vehicle.s_bump[5] = (recv_buff.order_data[0]>>5) & 0x01;
                m_info_vehicle.s_bump[6] = (recv_buff.order_data[0]>>6) & 0x01;
                m_info_vehicle.s_bump[7] = (recv_buff.order_data[0]>>7) & 0x01;
                //ROS_INFO("%d",(m_info_vehicle.s_bump[0]));
            }
            /************************0x01X8 driver motion info***************************************/
            if((recv_buff.order_id_l & 0x0f) == 0x08)
            {
                m_info_vehicle.s_driver_speed[recv_buff.order_data[0]] = (recv_buff.order_data[1]<<8 & 0xff00) | (recv_buff.order_data[2] & 0x00ff);
                m_info_vehicle.s_driver_pulse[recv_buff.order_data[0]] = (recv_buff.order_data[3]<<24 & 0xff000000)
                        | (recv_buff.order_data[4]<<16 & 0x00ff0000)
                        | (recv_buff.order_data[5]<<8 & 0x0000ff00)
                        | (recv_buff.order_data[6] & 0x000000ff);
                //ROS_INFO("%d",(m_info_vehicle.s_driver_speed[0]));
            }
            /************************0x01X9 driver status info***************************************/
            if((recv_buff.order_id_l & 0x0f) == 0x09)
            {
                temp = (recv_buff.order_data[1]<<8 & 0xff00) | (recv_buff.order_data[2] & 0x00ff);
                m_info_vehicle.s_driver_volt[recv_buff.order_data[0]] = ((float)temp)/10;
                temp = (recv_buff.order_data[3]<<8 & 0xff00) | (recv_buff.order_data[4] & 0x00ff);
                m_info_vehicle.s_driver_current[recv_buff.order_data[0]] = ((float)temp)/10;
                m_info_vehicle.s_driver_heat[recv_buff.order_data[0]] = (char)(recv_buff.order_data[5]);
                m_info_vehicle.s_motor_heat[recv_buff.order_data[0]] = (char)(recv_buff.order_data[6]);
                m_info_vehicle.s_driver_err_volt_low[recv_buff.order_data[0]] = recv_buff.order_data[7] & 0x01;
                m_info_vehicle.s_motor_err_volt_low[recv_buff.order_data[0]] = (recv_buff.order_data[7]>>1) & 0x01;
                m_info_vehicle.s_motor_err_current_over[recv_buff.order_data[0]] = (recv_buff.order_data[7]>>2) & 0x01;
                m_info_vehicle.s_driver_err_heat_over[recv_buff.order_data[0]] = (recv_buff.order_data[7]>>3) & 0x01;
                //ROS_INFO("%f",(m_info_vehicle.s_driver_current[0]));
            }
        }
    }
    if(m_data_recv.size()>0)m_data_recv.erase(m_data_recv.begin(),m_data_recv.begin()+i);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: StdVehicleReset
 * detail: Vehicle reset CMD
 -----------------------------------------------------------------------------------------------------------------*/
vector<unsigned char> CommTransform::StdVehicleReset()
{
    vector<unsigned char> buffer(14,0X00);
    buffer[HEAD] = 0X55;
    buffer[LEN] = 0X0E;
    buffer[IDH] = 0x00;
    buffer[IDL] = 0x01;
    buffer[DATA0] = 0x01;
    buffer[DATA1] = m_info_vehicle.s_number;
    buffer[DATA2] = m_info_vehicle.s_series;
    buffer[ADD] = 0x01;
    buffer[CHECK] = DataCheckSum(buffer);
    return buffer;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: StdVehicleClear
 * detail: vehicle clear all error status
 -----------------------------------------------------------------------------------------------------------------*/
vector<unsigned char> CommTransform::StdVehicleClear()
{
    vector<unsigned char> buffer(14,0X00);
    buffer[HEAD] = 0X55;
    buffer[LEN] = 0X0E;
    buffer[IDH] = 0x00;
    buffer[IDL] = 0x41;
    buffer[DATA0] = 0x01;
    buffer[DATA1] = m_info_vehicle.s_number;
    buffer[DATA2] = m_info_vehicle.s_series;
    buffer[ADD] = 0x01;
    buffer[CHECK] = DataCheckSum(buffer);

    return buffer;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: StdVehicleVersion
 * detail:Get vehicle version
 -----------------------------------------------------------------------------------------------------------------*/
vector<unsigned char> CommTransform::StdVehicleVersion()
{
    vector<unsigned char> buffer(14,0X00);
    buffer[HEAD] = 0X55;
    buffer[LEN] = 0X0E;
    buffer[IDH] = 0x00;
    buffer[IDL] = 0x21;
    buffer[DATA0] = 0x01;
    buffer[DATA1] = m_info_vehicle.s_number;
    buffer[DATA2] = m_info_vehicle.s_series;
    buffer[ADD] = 0x01;
    buffer[CHECK] = DataCheckSum(buffer);
    return buffer;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: VehicleSetState
 * detail:
 -----------------------------------------------------------------------------------------------------------------*/
vector<unsigned char> CommTransform::StdVehicleSetState(bool state)
{
    vector<unsigned char> buffer(14,0X00);
    buffer[HEAD] = 0X55;
    buffer[LEN] = 0X0E;
    buffer[IDH] = 0x00;
    buffer[IDL] = 0x31;
    buffer[DATA0] = 0x01;
    buffer[DATA1] = m_info_vehicle.s_number;
    buffer[DATA2] = m_info_vehicle.s_series;
    buffer[DATA3] = state;
    buffer[DATA4] = m_info_vehicle.s_number;
    buffer[ADD] = 0x01;
    buffer[CHECK] = DataCheckSum(buffer);
    return buffer;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: StdVehicleMode
 * detail: Set vehicle mode
 -----------------------------------------------------------------------------------------------------------------*/
vector<unsigned char> CommTransform::StdVehicleMode(unsigned char mode,bool beep)
{
    if(mode > 2)mode = 0;
    vector<unsigned char> buffer(14,0X00);
    buffer[HEAD] = 0X55;
    buffer[LEN] = 0X0E;
    buffer[IDH] = 0x01;
    buffer[IDL] = (m_info_vehicle.s_number<<4 & 0xf0) + 0x02;
    buffer[DATA0] = mode;
    buffer[DATA1] = beep;
    buffer[ADD] = 0x01;
    buffer[CHECK] = DataCheckSum(buffer);

    return buffer;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: StdVehiclemove
 * detail: send move CMD
 -----------------------------------------------------------------------------------------------------------------*/
vector<unsigned char> CommTransform::StdVehiclemove(short speed_x, short speed_y, short rotate)
{
    if(speed_x>1000)speed_x = 1000;
    if(speed_y<-1000)speed_y = -1000;
    if(rotate>1000)rotate = 1000;
    if(rotate<-1000)rotate = -1000;

    vector<unsigned char> buffer(14,0X00);
    buffer[HEAD] = 0X55;
    buffer[LEN] = 0X0E;
    buffer[IDH] = 0x01;
    buffer[IDL] = (m_info_vehicle.s_number<<4 & 0xf0) + 0x03;
    buffer[DATA0] = static_cast <unsigned char>(speed_x>>8);
    buffer[DATA1] = static_cast <unsigned char>(speed_x);
    buffer[DATA2] = static_cast <unsigned char>(speed_y>>8);
    buffer[DATA3] = static_cast <unsigned char>(speed_y);
    buffer[DATA4] = static_cast <unsigned char>(rotate>>8);
    buffer[DATA5] = static_cast <unsigned char>(rotate);
    buffer[ADD] = 0x01;
    buffer[CHECK] = DataCheckSum(buffer);
    return buffer;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: VehicleConciseMode
 * detail: Open or close secondary callback
 -----------------------------------------------------------------------------------------------------------------*/
vector<unsigned char> CommTransform::StdVehicleConciseMode(bool state)
{
    vector<unsigned char> buffer(14,0X00);
    vector<unsigned char> send_buf;
    buffer[HEAD] = 0X55;
    buffer[LEN] = 0X0E;
    buffer[IDH] = 0x01;
    buffer[DATA0] = state;
    buffer[ADD] = 0x01;

    buffer[IDL] = (m_info_vehicle.s_number<<4 & 0xf0) + 0x06;
    buffer[CHECK] = DataCheckSum(buffer);
    send_buf.insert(send_buf.end(), buffer.begin(), buffer.end());

    buffer[IDL] = (m_info_vehicle.s_number<<4 & 0xf0) + 0x07;
    buffer[CHECK] = DataCheckSum(buffer);
    send_buf.insert(send_buf.end(), buffer.begin(), buffer.end());

    buffer[IDL] = (m_info_vehicle.s_number<<4 & 0xf0) + 0x08;
    buffer[CHECK] = DataCheckSum(buffer);
    send_buf.insert(send_buf.end(), buffer.begin(), buffer.end());

    buffer[IDL] = (m_info_vehicle.s_number<<4 & 0xf0) + 0x09;
    buffer[CHECK] = DataCheckSum(buffer);
    send_buf.insert(send_buf.end(), buffer.begin(), buffer.end());
    return buffer;
}
