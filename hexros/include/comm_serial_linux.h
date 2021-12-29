
#ifndef LINUX_COMM_SERIAL_H
#define LINUX_COMM_SERIAL_H

#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      //file ctrl define
#include <termios.h>    //PPSIX terminal define
#include <errno.h>      //error define
using namespace std;

/*------------------------------------------------------------------------------------------------------------------
*PROJECT OF HEXMAN
*SETPS TO USE
 * 1.SetSerialScanState(true) to open auto scan
 * 2.Use signal NewSerialList(QStringList) or function GetSerialList() to get com list
 * 3.In serial list, odd rows is com names,even rows is description which can confirm device
 * 4.Init serial
 * 5.Open serial,return the open state and emit signal of open state
 * 6.Use function DataSend(QByteArray data) to send
 * 7.Connect NewDataRecv(QByteArray) to get new data at everytime it received
 * 8.Use DataRecv(int len) to get data in buffer with needed lenght at any time
 * 9.Close serial
 -----------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------
 * If use ros_info,add define ROS
 * --------------------------------------------------------------*/
#define ROS
/*-----------------------------------------------------------------
 * Serial device sequence.Add your needed device here
 * --------------------------------------------------------------*/
static std::string arr_device[] = {"/dev/CAN2COM_HUB","/dev/ttyUSB0","/dev/ttyS0"};


static int arr_baud[] = {460800, 115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300};
static unsigned int arr_baud_set[] = {B460800, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};


class CommSerialLinux
{
public:
    explicit CommSerialLinux(void);
    ~CommSerialLinux();

    bool SerialOpen(void);
    bool SerialIsOpen(void);
    void SerialClose(void);
    bool SerialSet(int baudrate, int flow_ctrl, int databits, int stopbits, int parity);
    bool StdSerialInit(void);
    vector<unsigned char> SerialDataRecv(void);
    unsigned long SerialDataSend(vector<unsigned char> data);
    void DataClear(void);

private:
    //void SerialScanHandle();

private:
    int m_serial_fd;
    bool f_serial_open_state;
    bool f_serial_disconnect;
};

#endif // LINUX_COMM_SERIAL_H
