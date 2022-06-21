#ifndef SERIALPORT_H
#define SERIALPORT_H
#include <stdio.h>  /*Standard Input&Output defintion*/
#include <iostream>
#include <time.h>
#include <cmath>
#include<stdlib.h>   /*Standard function libray defintion*/
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<termios.h>
#include<errno.h>
#include<string.h>

#define FALSE  -1
#define TRUE   0

using namespace std;

//SerialPort Class for DJI Maniflod
class SerialPort{
private:
    int fd;//串口号
    char *PortName;//串口名
    int speed, flow_ctrl, databits, stopbits, parity;
    int data_len;
    int UART0_Init();
    int UART0_Set();
public:
    SerialPort();
    SerialPort(char *pn);
    ~SerialPort();
    void Open();
    int Send(char *send_buf);
    int Recv(char *rcv_buf);
    void Close();
};




#endif // SERIALPORT_H
