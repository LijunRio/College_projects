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
#include"serialport.h"
#define FALSE  -1
#define TRUE   0

using namespace std;

SerialPort::SerialPort(){
    PortName = "/dev/ttyUSB0";
    speed = 115200;
    flow_ctrl = 0;
    databits = 8;
    stopbits = 1;
    parity = 'N';
    data_len = 9;
}
SerialPort::SerialPort(char *pn){
    PortName = pn;
    speed = 115200;
    flow_ctrl = 0;
    databits = 8;
    stopbits = 1;
    parity = 'N';
    data_len = 9;
}

SerialPort::~SerialPort(){
}

int SerialPort::UART0_Set(){
    int   i;
    int   status;
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300 };
    int   name_arr[] = { 115200, 19200, 9600, 4800, 2400, 1200, 300 };

    struct termios options;

    /*tcgetattr(fd,&options)µÃµœÓëfdÖžÏò¶ÔÏóµÄÏà¹Ø²ÎÊý£¬²¢œ«ËüÃÇ±£ŽæÓÚoptions,žÃº¯Êý»¹¿ÉÒÔ²âÊÔÅäÖÃÊÇ·ñÕýÈ·£¬žÃŽ®¿ÚÊÇ·ñ¿ÉÓÃµÈ¡£Èôµ÷ÓÃ³É¹Š£¬º¯Êý·µ»ØÖµÎª0£¬Èôµ÷ÓÃÊ§°Ü£¬º¯Êý·µ»ØÖµÎª1.
    */
    if (tcgetattr(fd, &options) != 0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }

    //ÉèÖÃŽ®¿ÚÊäÈë²šÌØÂÊºÍÊä³ö²šÌØÂÊ
    for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //ÐÞžÄ¿ØÖÆÄ£Êœ£¬±£Ö€³ÌÐò²»»áÕŒÓÃŽ®¿Ú
    options.c_cflag |= CLOCAL;
    //ÐÞžÄ¿ØÖÆÄ£Êœ£¬Ê¹µÃÄÜ¹»ŽÓŽ®¿ÚÖÐ¶ÁÈ¡ÊäÈëÊýŸÝ
    options.c_cflag |= CREAD;

    //ÉèÖÃÊýŸÝÁ÷¿ØÖÆ
    switch (flow_ctrl)
    {

    case 0://²»Ê¹ÓÃÁ÷¿ØÖÆ
        options.c_cflag &= ~CRTSCTS;
        break;

    case 1://Ê¹ÓÃÓ²ŒþÁ÷¿ØÖÆ
        options.c_cflag |= CRTSCTS;
        break;
    case 2://Ê¹ÓÃÈíŒþÁ÷¿ØÖÆ
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //ÉèÖÃÊýŸÝÎ»
    //ÆÁ±ÎÆäËû±êÖŸÎ»
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
        fprintf(stderr, "Unsupported data size\n");
        return (FALSE);
    }
    //ÉèÖÃÐ£ÑéÎ»
    switch (parity)
    {
    case 'n':
    case 'N': //ÎÞÆæÅŒÐ£ÑéÎ»¡£
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O'://ÉèÖÃÎªÆæÐ£Ñé
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E'://ÉèÖÃÎªÅŒÐ£Ñé
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //ÉèÖÃÎª¿Õžñ
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported parity\n");
        return (FALSE);
    }
    // ÉèÖÃÍ£Ö¹Î»
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB; break;
    case 2:
        options.c_cflag |= CSTOPB; break;
    default:
        fprintf(stderr, "Unsupported stop bits\n");
        return (FALSE);
    }

    //ÐÞžÄÊä³öÄ£Êœ£¬Ô­ÊŒÊýŸÝÊä³ö
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//ÎÒŒÓµÄ
    //options.c_lflag &= ~(ISIG | ICANON);

    //ÉèÖÃµÈŽýÊ±ŒäºÍ×îÐ¡œÓÊÕ×Ö·û
    options.c_cc[VTIME] = 1; /* ¶ÁÈ¡Ò»žö×Ö·ûµÈŽý1*(1/10)s */
    options.c_cc[VMIN] = 1; /* ¶ÁÈ¡×Ö·ûµÄ×îÉÙžöÊýÎª1 */

    //Èç¹û·¢ÉúÊýŸÝÒç³ö£¬œÓÊÕÊýŸÝ£¬µ«ÊÇ²»ÔÙ¶ÁÈ¡ Ë¢ÐÂÊÕµœµÄÊýŸÝµ«ÊÇ²»¶Á
    tcflush(fd, TCIFLUSH);

    //Œ€»îÅäÖÃ (œ«ÐÞžÄºóµÄtermiosÊýŸÝÉèÖÃµœŽ®¿ÚÖÐ£©
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("com set error!\n");
        return (FALSE);
    }
    return (TRUE);
}

int SerialPort::UART0_Init(){
    int err;
    //ÉèÖÃŽ®¿ÚÊýŸÝÖ¡žñÊœ
    if (UART0_Set() == FALSE)
    {
        return FALSE;
    }
    else
    {
        return  TRUE;
    }
}

void SerialPort::Open(){
    int open_code;
    fd = open(PortName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (FALSE == fd || -1 == fd)
    {
        perror("Can't Open Serial Port");
        open_code = FALSE;
    }
    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
        open_code = FALSE;
    }

    int err;
//    cout<<"fd :"<<open_code<<endl;
    do
    {	err = UART0_Init();
//        printf("Set Port Exactly!\n");
    } while (FALSE == err || FALSE == open_code);
}

int SerialPort::Recv(char *rcv_buf){
    int len, fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

    time.tv_sec = 1000;
    time.tv_usec = 0;

    fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
    // fs_sel = 1;

    if (fs_sel)
    {
        len = read(fd, rcv_buf, data_len);
//        printf("I am right!(version1.2) len = %d fs_sel = %d\n", len, fs_sel);
        return len;
    }
    else
    {
        printf("Sorry,I am wrong!");
        return FALSE;
    }
}

int SerialPort::Send(char *send_buf)
{
    int len = 0;

    len = write(fd, send_buf, data_len);
    if (len == data_len)
    {
        return len;
    }
    else
    {
        tcflush(fd, TCOFLUSH);
        return FALSE;
    }
}

void SerialPort::Close(){
    close(fd);
}
