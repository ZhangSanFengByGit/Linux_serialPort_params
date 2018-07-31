#ifndef SERIALPORT_H_  
#define SERIALPORT_H_  
#include <iostream>
#include <string>
/*设置串口配置参数*/
typedef struct
{
    char prompt;  //prompt after reciving data
    int  baudrate;  //baudrate
    char databit;  //data bits, 5, 6, 7, 8
    char  debug;  //debug mode, 0: none, 1: debug
    char  echo;   //echo mode, 0: none, 1: echo
    char fctl;   //flow control, 0: none, 1: hardware, 2: software
    char  tty;   //tty: 0, 1, 2, 3, 4, 5, 6, 7
    char parity;  //parity 0: none, 1: odd, 2: even
    char stopbit;  //stop bits, 1, 2
    const int reserved; //reserved, must be zero
}portinfo_t;

typedef portinfo_t *pportinfo_t;


/******************************************************************* 
* 名称：                  PortOpen 
* 功能：                打开串口及其初始化
* 入口参数：        fd    :文件描述符     port :串口号(ttymxc1) 
* 出口参数：        正确返回为1，错误返回为0 
*******************************************************************/  
int PortOpen();

/******************************************************************* 
* 名称：                  ReadCom 
* 功能：               
* 入口参数：        fd    :文件描述符     port :串口号(ttymxc1) 
* 出口参数：        正确返回为1，错误返回为0 
*******************************************************************/ 
int ReadCom(char *pcom_data);

/******************************************************************* 
* 名称：                  PortClose 
* 功能：               
* 入口参数：        fd    :文件描述符     port :串口号(ttymxc1) 
* 出口参数：        正确返回为1，错误返回为0 
*******************************************************************/ 
void PortClose();

int set_speed;

#endif //SERIALPORT_H_


