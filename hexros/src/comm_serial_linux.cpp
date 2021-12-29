#include <comm_serial_linux.h>
#ifdef ROS
#include <ros/ros.h>
#endif
using namespace std;


CommSerialLinux::CommSerialLinux(void)
{
    m_serial_fd = -1;
    f_serial_disconnect = false;
}
CommSerialLinux::~CommSerialLinux()
{

}
/*------------------------------------------------------------------------------------------------------------------
 * name: SerialOpen
 * detail: Open serial with the sequence
 -----------------------------------------------------------------------------------------------------------------*/
bool CommSerialLinux::SerialOpen()
{		
    int count = sizeof (arr_device)/sizeof (arr_device[0]);
    if (SerialIsOpen()==true)return false;
    for(int i=0;i<count;i++)
    {
        m_serial_fd = open(arr_device[i].data(),O_RDWR | O_NOCTTY | O_NONBLOCK);
        if(m_serial_fd!=-1)
        {
            #ifdef ROS
            ROS_INFO("Serial device : %s",arr_device[i].data());
            #else
            printf("Serial device : %s",arr_device[i].data());
            #endif
            return true;
        }
    }
    #ifdef ROS
    ROS_ERROR("Open serial failed");
    #else
    printf("Open serial failed");
    #endif
    return false;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: SerialIsOpen
 * detail: Return serial state
 -----------------------------------------------------------------------------------------------------------------*/
bool CommSerialLinux::SerialIsOpen()
{
    return m_serial_fd < 0 ? false : true;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: SerialClose
 * detail: Close serial device
 -----------------------------------------------------------------------------------------------------------------*/
void CommSerialLinux::SerialClose()
{
    if(m_serial_fd < 0)return ;
    close(m_serial_fd);
    m_serial_fd = -1;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: SerialSet
 * detail: Set serial parameter
 -----------------------------------------------------------------------------------------------------------------*/
bool CommSerialLinux::SerialSet(int baudrate, int flow_ctrl, int databits, int stopbits, int parity)
{
    if (SerialIsOpen() == false)
        return false;
    struct termios options;
    if (tcgetattr(m_serial_fd, &options) != 0)
    {
        #ifdef ROS
        ROS_INFO("Serial options failed");
        #else
        printf("Serial options failed");
        #endif
        return (false);
    }
    ///////////////baudrate set//////////////////////////////////////
    for (int i = 0; i < sizeof(arr_baud) / sizeof(int); i++)
    {
        if (baudrate == arr_baud[i])
        {
            cfsetispeed(&options, arr_baud_set[i]);
            cfsetospeed(&options, arr_baud_set[i]);
            break;
        }
    }
    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;
    // 解决二进制传输时，数据遇到0x0d , 0x11,0x13 会被丢掉的问题
    options.c_iflag &= ~(BRKINT | ICRNL | ISTRIP | IXON);
    ///////////////flow ctrl set//////////////////////////////////////
    switch (flow_ctrl)
    {
    case 0: //no ctrl
        options.c_cflag &= ~CRTSCTS;
        break;
    case 1: //hardware ctrl
        options.c_cflag |= CRTSCTS;
        break;
    case 2: //software ctrl
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //////////databits set////////////////////////////////////////////
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        #ifdef ROS
        ROS_INFO("Wrong databits");
        #else
        printf("Wrong databits");
        #endif
        return (false);
    }
    ////////////////parity set//////////////////////////////////////
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O': //设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E': //设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        #ifdef ROS
        ROS_INFO("Wrong parity");
        #else
        printf("Wrong parity");
        #endif
        return (false);
    }
    ////////////////stopbits set//////////////////////////////////////
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        #ifdef ROS
        ROS_INFO("Wrong stopbits");
        #else
        printf("Wrong stopbits");
        #endif
        return (false);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1;  /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(m_serial_fd, TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(m_serial_fd, TCSANOW, &options) != 0)
    {
        #ifdef ROS
        ROS_INFO("options set failed");
        #else
        printf("options set failed");
        #endif
        return (false);
    }
    return (true);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: SerialStdInit
 * detail: Open and set serial default parameter
 -----------------------------------------------------------------------------------------------------------------*/
bool CommSerialLinux::StdSerialInit()
{
    if(SerialOpen() == false)return false;
    if(SerialSet(460800,0,8,1,'n') == false)return false;
    return true;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: SerialDataSend
 * detail: As name.
 -----------------------------------------------------------------------------------------------------------------*/
unsigned long CommSerialLinux::SerialDataSend(vector<unsigned char> data)
{
    static int disconnect_count = 0;
    if (SerialIsOpen()==false)return false;
    tcflush(m_serial_fd, TCIFLUSH);
    unsigned long data_len = data.size();
    unsigned long len = 0;
    while(1)
    {
        len = write(m_serial_fd, &data[0], data_len);
        if (len == data_len)
        {
            disconnect_count = 0;
            return len;
        }
        //serial send error
        if(len == -1)
        {
            disconnect_count++;
            //disconnect handle
            if(disconnect_count == 200)
            {
                disconnect_count = 0;
                f_serial_disconnect = true;
                #ifdef ROS
                ROS_INFO("serial diconnect");
                #else
                printf("serial diconnect");
                #endif
                SerialClose();
                return false;
            }
            continue;
        }
        //not send all data
        #ifdef ROS
        ROS_INFO("serial lost send data");
        #else
        printf("serial lost send data");
        #endif
        return len;
    }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DataRecv
 * detail: As name.
 -----------------------------------------------------------------------------------------------------------------*/
vector<unsigned char> CommSerialLinux::SerialDataRecv(void)
{
    static int disconnect_count = 0;
    fd_set fs_read;
    vector<unsigned char> recv_buff;
    unsigned char buff[100] = {0};
    struct timeval time;
    time.tv_sec = 0;
    time.tv_usec = 1000;
    int len, fs_sel;
    unsigned long pos =0;

    recv_buff.clear();
    if (SerialIsOpen()==false)return recv_buff;

    while(1)
    {
        FD_ZERO(&fs_read);
        FD_SET(m_serial_fd, &fs_read);
        fs_sel = select(m_serial_fd + 1, &fs_read, NULL, NULL, &time);//使用select实现串口的多路通信

        if (fs_sel > 0)
        {
            len = read(m_serial_fd, buff , 100);
            //available data
            if(len > 0)
            {
                disconnect_count = 0;
                for(int i=0;i<len;i++)
                {
                  recv_buff.push_back(buff[i]);
                }
                //too long data
                if(recv_buff.size()>2048)return recv_buff;
                continue;
            }
            //serial error
            if(len == -1)
            {
                #ifdef ROS
                ROS_INFO("serial read error");
                #else
                printf("serial read error");
                #endif
                disconnect_count++;
                //lost connect handle
                if(disconnect_count == 200)
                {
                    disconnect_count = 0;
                    f_serial_disconnect = true;
                    #ifdef ROS
                    ROS_INFO("serial diconnect");
                    #else
                    printf("serial diconnect");
                    #endif
                    SerialClose();
                    return recv_buff;
                }
                continue;
            }
            //no more data
            disconnect_count = 0;
            return recv_buff;
        }
        else
            return recv_buff;
    }
}
/*------------------------------------------------------------------------------------------------------------------
 * name: DataClear
 * detail: As name.
 -----------------------------------------------------------------------------------------------------------------*/
void CommSerialLinux::DataClear()
{
    if (SerialIsOpen()==false)return;
    tcflush(m_serial_fd, TCIOFLUSH);
}

