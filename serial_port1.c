//´®¿ÚÏà¹ØµÄÍ·ÎÄ¼þ  
//author:lzwang6 iflytek.com

#include<stdio.h>      /*±ê×¼ÊäÈëÊä³ö¶¨Òå*/  
#include<stdlib.h>     /*±ê×¼º¯Êý¿â¶¨Òå*/  
#include<unistd.h>     /*Unix ±ê×¼º¯Êý¶¨Òå*/  
#include<sys/types.h>   
#include<sys/stat.h>     
#include<fcntl.h>      /*ÎÄ¼þ¿ØÖÆ¶¨Òå*/  
#include<termios.h>    /*PPSIX ÖÕ¶Ë¿ØÖÆ¶¨Òå*/  
#include<errno.h>      /*´íÎóºÅ¶¨Òå*/  
#include<string.h> 
#include"serial_port1.h"
#include<iostream>
#include<pthread.h>


using namespace std;

//ºê¶¨Òå  
#define FALSE  -1  
#define TRUE   0  
#define IFLYTEK_COM_PATH    "/dev/ttymxc1"
#define msleep(n) usleep(n*1000)
typedef int BOOL;
BOOL listen_thr_stop_ = TRUE;
pthread_mutex_t mut;
int i4_uart_fd;
fd_set rd;
struct timeval timeout;
int nread,retval;
char read_tmp[32] = {0};
string save_path;



/******************************************************************* 
* Ãû³Æ£º                  UART0_Open 
* ¹¦ÄÜ£º                ´ò¿ª´®¿Ú²¢·µ»Ø´®¿ÚÉè±¸ÎÄ¼þÃèÊö 
* Èë¿Ú²ÎÊý£º        fd    :ÎÄ¼þÃèÊö·û     port :´®¿ÚºÅ(ttyS0,ttyS1,ttyS2) 
* ³ö¿Ú²ÎÊý£º        ÕýÈ··µ»ØÎª1£¬´íÎó·µ»ØÎª0 
*******************************************************************/  
int UART0_Open(int fd,char* port)  
{  
     
	fd = open(port, O_RDWR|O_NOCTTY|O_NDELAY);  
	if (FALSE == fd)  
	{  
		perror("Can't Open Serial Port");  
		return(FALSE);  
	}  
	//»Ö¸´´®¿ÚÎª×èÈû×´Ì¬                                 
	if(fcntl(fd, F_SETFL, 0) < 0)  
	{  
		printf("fcntl failed!\n");  
		return(FALSE);  
	}       
	else  
	{  
		printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));  
	}  
	//²âÊÔÊÇ·ñÎªÖÕ¶ËÉè±¸      
	if(0 == isatty(STDIN_FILENO))  
	{  
		printf("standard input is not a terminal device\n");  
		return(FALSE);  
	}  
	else  
	{  
		printf("isatty success!\n");  
	}                
	printf("fd->open=%d\n",fd);  
	return fd;  
}  
/******************************************************************* 
* Ãû³Æ£º                UART0_Close 
* ¹¦ÄÜ£º                ¹Ø±Õ´®¿Ú²¢·µ»Ø´®¿ÚÉè±¸ÎÄ¼þÃèÊö 
* Èë¿Ú²ÎÊý£º        fd    :ÎÄ¼þÃèÊö·û     port :´®¿ÚºÅ(ttyS0,ttyS1,ttyS2) 
* ³ö¿Ú²ÎÊý£º        void 
*******************************************************************/  
   
void UART0_Close(int fd)  
{  
    listen_thr_stop_ = TRUE;
	close(fd);  
}  
   
/******************************************************************* 
* Ãû³Æ£º                UART0_Set 
* ¹¦ÄÜ£º                ÉèÖÃ´®¿ÚÊý¾ÝÎ»£¬Í£Ö¹Î»ºÍÐ§ÑéÎ» 
* Èë¿Ú²ÎÊý£º        fd        ´®¿ÚÎÄ¼þÃèÊö·û 
*                              speed     ´®¿ÚËÙ¶È 
*                              flow_ctrl   Êý¾ÝÁ÷¿ØÖÆ 
*                           databits   Êý¾ÝÎ»   È¡ÖµÎª 7 »òÕß8 
*                           stopbits   Í£Ö¹Î»   È¡ÖµÎª 1 »òÕß2 
*                           parity     Ð§ÑéÀàÐÍ È¡ÖµÎªN,E,O,,S 
*³ö¿Ú²ÎÊý£º          ÕýÈ··µ»ØÎª1£¬´íÎó·µ»ØÎª0 
*******************************************************************/  
int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)  
{  
     
	int   i;  
	int   status;  
	int   speed_arr[] = { B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300};  
	int   name_arr[] = {115200, 38400, 19200,  9600,  4800,  2400,  1200,  300};  
           
	struct termios options;  
     
	/*tcgetattr(fd,&options)µÃµ½ÓëfdÖ¸Ïò¶ÔÏóµÄÏà¹Ø²ÎÊý£¬²¢½«ËüÃÇ±£´æÓÚoptions,¸Ãº¯Êý»¹¿ÉÒÔ²âÊÔÅäÖÃÊÇ·ñÕýÈ·£¬¸Ã´®¿ÚÊÇ·ñ¿ÉÓÃµÈ¡£Èôµ÷ÓÃ³É¹¦£¬º¯Êý·µ»ØÖµÎª0£¬Èôµ÷ÓÃÊ§°Ü£¬º¯Êý·µ»ØÖµÎª1. 
    */  
	if( tcgetattr( fd,&options)  !=  0)  
	{  
		perror("SetupSerial 1");      
		return(FALSE);   
	}  
    
    //ÉèÖÃ´®¿ÚÊäÈë²¨ÌØÂÊºÍÊä³ö²¨ÌØÂÊ  
	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)  
	{  
		if  (speed == name_arr[i])  
		{               
			cfsetispeed(&options, speed_arr[i]);   
			cfsetospeed(&options, speed_arr[i]);    
		}  
	}       
     
    //ÐÞ¸Ä¿ØÖÆÄ£Ê½£¬±£Ö¤³ÌÐò²»»áÕ¼ÓÃ´®¿Ú  
    options.c_cflag |= CLOCAL;  
    //ÐÞ¸Ä¿ØÖÆÄ£Ê½£¬Ê¹µÃÄÜ¹»´Ó´®¿ÚÖÐ¶ÁÈ¡ÊäÈëÊý¾Ý  
    options.c_cflag |= CREAD;  
    
    //ÉèÖÃÊý¾ÝÁ÷¿ØÖÆ  
    switch(flow_ctrl)  
    {  
        
		case 0 ://²»Ê¹ÓÃÁ÷¿ØÖÆ  
              options.c_cflag &= ~CRTSCTS; 
              options.c_cflag |= CLOCAL | CREAD;
              options.c_cflag &= ~(ICANON | ECHO | ECHOE | ISIG);
              options.c_cflag &= ~OPOST;
              options.c_cflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
              break;     
        
		case 1 ://Ê¹ÓÃÓ²¼þÁ÷¿ØÖÆ  
              options.c_cflag |= CRTSCTS;  
              break;  
		case 2 ://Ê¹ÓÃÈí¼þÁ÷¿ØÖÆ  
              options.c_cflag |= IXON | IXOFF | IXANY;  
              break;  
		case 3 ://ÆÁ±ÎÌØÊâ¿ØÖÆµÄÊôÐÔ  
				options.c_cflag |= CLOCAL | CREAD;
				options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
				options.c_oflag &= ~OPOST;
				options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON); 
              break;   
    }  
    //ÉèÖÃÊý¾ÝÎ»  
    //ÆÁ±ÎÆäËû±êÖ¾Î»  
    options.c_cflag &= ~CSIZE;  
    switch (databits)  
    {    
		case 5    :  
                     options.c_cflag |= CS5;  
                     break;  
		case 6    :  
                     options.c_cflag |= CS6;  
                     break;  
		case 7    :      
                 options.c_cflag |= CS7;  
                 break;  
		case 8:      
                 options.c_cflag |= CS8;  
                 break;    
		default:     
                 fprintf(stderr,"Unsupported data size\n");  
                 return (FALSE);   
    }  
    //ÉèÖÃÐ£ÑéÎ»  
    switch (parity)  
    {    
		case 'n':  
		case 'N': //ÎÞÆæÅ¼Ð£ÑéÎ»¡£  
                 options.c_cflag &= ~PARENB;   
                 options.c_iflag &= ~INPCK;      
                 break;   
		case 'o':    
		case 'O'://ÉèÖÃÎªÆæÐ£Ñé      
                 options.c_cflag |= (PARODD | PARENB);   
                 options.c_iflag |= INPCK;               
                 break;   
		case 'e':   
		case 'E'://ÉèÖÃÎªÅ¼Ð£Ñé    
                 options.c_cflag |= PARENB;         
                 options.c_cflag &= ~PARODD;         
                 options.c_iflag |= INPCK;        
                 break;  
		case 's':  
		case 'S': //ÉèÖÃÎª¿Õ¸ñ   
                 options.c_cflag &= ~PARENB;  
                 options.c_cflag &= ~CSTOPB;  
                 break;   
        default:    
                 fprintf(stderr,"Unsupported parity\n");      
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
                       fprintf(stderr,"Unsupported stop bits\n");   
                       return (FALSE);  
    }  
     
	//ÐÞ¸ÄÊä³öÄ£Ê½£¬Ô­Ê¼Êý¾ÝÊä³ö  
	options.c_oflag &= ~OPOST;  
    
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  
	//options.c_lflag &= ~(ISIG | ICANON);  
     
    //ÉèÖÃµÈ´ýÊ±¼äºÍ×îÐ¡½ÓÊÕ×Ö·û  
    options.c_cc[VTIME] = 1; /* ¶ÁÈ¡Ò»¸ö×Ö·ûµÈ´ý1*(1/10)s */    
    options.c_cc[VMIN] = 1; /* ¶ÁÈ¡×Ö·ûµÄ×îÉÙ¸öÊýÎª1 */  
     
    //Èç¹û·¢ÉúÊý¾ÝÒç³ö£¬½ÓÊÕÊý¾Ý£¬µ«ÊÇ²»ÔÙ¶ÁÈ¡ Ë¢ÐÂÊÕµ½µÄÊý¾Ýµ«ÊÇ²»¶Á  
    tcflush(fd,TCIFLUSH);  
     
    //¼¤»îÅäÖÃ (½«ÐÞ¸ÄºóµÄtermiosÊý¾ÝÉèÖÃµ½´®¿ÚÖÐ£©  
    if (tcsetattr(fd,TCSANOW,&options) != 0)    
	{  
		perror("com set error!\n");    
		return (FALSE);   
	}  
    return (TRUE);   
}  
/******************************************************************* 
* Ãû³Æ£º                UART0_Init() 
* ¹¦ÄÜ£º                ´®¿Ú³õÊ¼»¯ 
* Èë¿Ú²ÎÊý£º        fd       :  ÎÄ¼þÃèÊö·û    
*               speed  :  ´®¿ÚËÙ¶È 
*                              flow_ctrl  Êý¾ÝÁ÷¿ØÖÆ 
*               databits   Êý¾ÝÎ»   È¡ÖµÎª 7 »òÕß8 
*                           stopbits   Í£Ö¹Î»   È¡ÖµÎª 1 »òÕß2 
*                           parity     Ð§ÑéÀàÐÍ È¡ÖµÎªN,E,O,,S 
*                       
* ³ö¿Ú²ÎÊý£º        ÕýÈ··µ»ØÎª1£¬´íÎó·µ»ØÎª0 
*******************************************************************/  
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)  
{  
    int err;  
    //ÉèÖÃ´®¿ÚÊý¾ÝÖ¡¸ñÊ½  
    if (UART0_Set(fd,speed,3,8,1,'N') == FALSE)  
	{                                                           
		return FALSE;  
	}  
    else  
	{  
		return  TRUE;  
	}  
}  
   
/******************************************************************* 
* Ãû³Æ£º                  UART0_Recv 
* ¹¦ÄÜ£º                ½ÓÊÕ´®¿ÚÊý¾Ý 
* Èë¿Ú²ÎÊý£º        fd                  :ÎÄ¼þÃèÊö·û     
*                              rcv_buf     :½ÓÊÕ´®¿ÚÖÐÊý¾Ý´æÈërcv_buf»º³åÇøÖÐ 
*                              data_len    :Ò»Ö¡Êý¾ÝµÄ³¤¶È 
* ³ö¿Ú²ÎÊý£º        ÕýÈ··µ»ØÎª1£¬´íÎó·µ»ØÎª0 
*******************************************************************/  
int UART0_Recv(int fd, char *rcv_buf,int data_len)  
{  
	int len;  
    len = read(fd,rcv_buf,data_len);  
    printf("the recv is =====%s\n", rcv_buf);
	return len;       
}  
/******************************************************************** 
* Ãû³Æ£º                  UART0_Send 
* ¹¦ÄÜ£º                ·¢ËÍÊý¾Ý 
* Èë¿Ú²ÎÊý£º        fd                  :ÎÄ¼þÃèÊö·û     
*                              send_buf    :´æ·Å´®¿Ú·¢ËÍÊý¾Ý 
*                              data_len    :Ò»Ö¡Êý¾ÝµÄ¸öÊý 
* ³ö¿Ú²ÎÊý£º        ÕýÈ··µ»ØÎª1£¬´íÎó·µ»ØÎª0 
*******************************************************************/  
int UART0_Send(int fd, char *send_buf,int data_len)  
{  
    int len = 0;  
     
    len = write(fd,send_buf,data_len);  
    if (len == data_len )  
	{  
		printf("send data is %s\n",send_buf);
		return len;  
	}       
    else     
	{  
                 
		tcflush(fd,TCOFLUSH);  
		return FALSE;  
	}  
     
}  

int read_data(int fd, void *buf, int len)
{
    int count;
    int ret;
    ret = 0;
    count = 0;
    ret = read(fd, (char*)buf + count, len);
    if (ret < 1) 
    {
        fprintf(stderr, "Read error %s\n", strerror(errno));
    }
    count += ret;
    len = len - ret;
    *((char*)buf + count) = '\0';
    return count;
}

/** ¶Á´®¿ÚÊý¾Ý
*  @param:  void * pParam Ïß³Ì²ÎÊý
*  @note:
*  @see:
*/
char tmp_str[32]={0};
void read_port(void)
{   
    FD_ZERO(&rd);
    FD_SET(i4_uart_fd,&rd);
    timeout.tv_sec = 1;
    timeout.tv_usec = 500;
    int count = 0;
    int len = 32;
    static int pos;
	printf("1111111111111111pos==%d\n", pos);
    retval = select(i4_uart_fd+1, &rd, NULL, NULL,&timeout);
	
    switch (retval)
    {
        case 0:
            printf("no data input within  1s.\n");
            pos=0;
            break;
        case -1:
            perror("select");
            break;       
        default:
	    if((nread = read_data(i4_uart_fd, read_tmp, len)) > 0)
        {
            if(0 == pos)
            {
                memset(tmp_str, 0, 32);
            }
            //strcat(tmp_str, read_tmp);
            printf("=============>>[lizhu]==========>>nread=%d\n", nread);
 	        memcpy(tmp_str+pos, read_tmp, nread);
	        pos+=nread;
            //printf("%02x\n", read_tmp[0]);
            //printf("%02x, %02x, %02x\n", tmp_str[0], tmp_str[1], tmp_str[2]);
        }
        else
            printf("reci is 0\n");
        break;
    }
}


/** ´®¿Ú¼àÌýÏß³Ì
*
*  ¼àÌýÀ´×Ô´®¿ÚµÄÊý¾ÝºÍÐÅÏ¢
*  @param:  void * pParam Ïß³Ì²ÎÊý
*  @note:
*  @see:
*/
void *ListenThread(void* pParam)
{
    int i4_rcvlen;
     //printf("log mesg3333333333333331\n");
	// Ïß³ÌÑ­»·,ÂÖÑ¯·½Ê½¶ÁÈ¡´®¿ÚÊý¾Ý
	while(FALSE == listen_thr_stop_)
	{
        pthread_mutex_lock(&mut);
        read_port();
        pthread_mutex_unlock(&mut);
    }
    pthread_exit(NULL);
}

/** ¿ªÆô¼àÌýÏß³Ì
*
*  ±¾¼àÌýÏß³ÌÍê³É¶Ô´®¿ÚÊý¾ÝµÄ¼àÌý,²¢½«½ÓÊÕµ½µÄÊý¾Ý´òÓ¡µ½ÆÁÄ»Êä³ö
*  @return: bool  ²Ù×÷ÊÇ·ñ³É¹¦
*  @note:   µ±Ïß³ÌÒÑ¾­´¦ÓÚ¿ªÆô×´Ì¬Ê±,·µ»Øflase
*  @see:
*/
BOOL OpenListenThread()
{
    int i4_err;
    listen_thr_stop_ = FALSE;
    pthread_t   OpenListenThreadHandle_;
    //printf("log mesg111111111111111111\n");
    i4_err = pthread_create(&OpenListenThreadHandle_, NULL, ListenThread, NULL);
    if (0 != i4_err)
	{
        printf("create thread ListenThread failed: %d\n", strerror(i4_err));
        listen_thr_stop_ = TRUE;
        return 0;
	}
    //printf("log mesg222222222222222\n");
    return 1;
    
}
/******************************************************************** 
* Ãû³Æ:readCOMThread 
* ¹¦ÄÜ:´Ó´®¿Ú¶ÁÈ¡Êý¾ÝµÄÏß³Ì 
* Èë¿Ú²ÎÊý£ºPVOID param
* ³ö¿Ú²ÎÊý£º     
*******************************************************************/  
void *readCOMThread(void *arg) 
{
    int i4_err;
    i4_uart_fd = UART0_Open(i4_uart_fd, IFLYTEK_COM_PATH); //´ò¿ª´®¿Ú£¬·µ»ØÎÄ¼þÃèÊö·û
    do
	{  
		i4_err = UART0_Init(i4_uart_fd, set_speed, 0, 8, 1, 'N'); 
        printf("readCOMThread | Init serial Port succes");
        
	}while(FALSE == i4_err || FALSE == i4_uart_fd);
    
    //¿ªÊ¼¼àÌý´®¿Ú
	if (!OpenListenThread())
	{
		printf("readCOMThread | OpenListenThread failed\n");
		return NULL;
	}
    
}
#if 0
int main(int argc, char **argv)
{
    int i4_err;
    //pthread_t   readComThreadHandle_;
    //i4_err = pthread_create(&readComThreadHandle_, NULL, readCOMThread, NULL);
    //if (0 != i4_err)
	//{
		//scylla_pluslog_error("scylla_plus_inst::start | create thread readCOMThread failed");
    //    printf("create thread readCOMThread failed: %d\n", strerror(i4_err));
	//}
	
    i4_uart_fd = UART0_Open(i4_uart_fd, IFLYTEK_COM_PATH); //´ò¿ª´®¿Ú£¬·µ»ØÎÄ¼þÃèÊö·û
    
    do
	{  
		i4_err = UART0_Init(i4_uart_fd, 115200, 0, 8, 1, 'N'); 
        printf("readCOMThread | Init serial Port succes\n");
        
	}while(FALSE == i4_err || FALSE == i4_uart_fd);
    //printf("log mesg444444444444\n");
    //¿ªÊ¼¼àÌý´®¿Ú
	if (!OpenListenThread())
	{
		printf("readCOMThread | OpenListenThread failed\n");
		return 0;
	}
    //printf("log mesg666666666666\n");
    //²»¶Ï¼ì²âÏß³ÌÊÇ·ñ½áÊø
	while(!listen_thr_stop_)
	{
            //printf("log mesg77777777777\n");
		msleep(500);  //Î´½áÊøÔòÐÝÃß500ms£¬È»ºóÔÙ¼ì²â
	}
    
    //Ïß³Ì½áÊø£¬Ôò¹Ø±Õ¼àÌý´®¿ÚµÄ
    UART0_Close(i4_uart_fd);
    
    printf("readCOMThread | usrt prot close success.");
    return i4_err;
    
}
#endif
/******************************************************************* 
* Ãû³Æ£º                  PortOpen 
* ¹¦ÄÜ£º                ´ò¿ª´®¿Ú¼°Æä³õÊ¼»¯
* Èë¿Ú²ÎÊý£º        fd    :ÎÄ¼þÃèÊö·û     port :´®¿ÚºÅ(ttymxc1) 
* ³ö¿Ú²ÎÊý£º        ÕýÈ··µ»ØÎª1£¬´íÎó·µ»ØÎª0 
*******************************************************************/  
int PortOpen()
{
    int i4_err;
   // i4_uart_fd = UART0_Open(i4_uart_fd, IFLYTEK_COM_PATH); //´ò¿ª´®¿Ú£¬·µ»ØÎÄ¼þÃèÊö·û
	int i4_flag = 0;
	for(i4_flag; i4_flag < 50; i4_flag++)
	{
	   	i4_uart_fd = UART0_Open(i4_uart_fd, IFLYTEK_COM_PATH); //............
	   	if(FALSE != i4_uart_fd)
	   	{
	   		printf("PortOpen | Open Port succes\n");
	  		break;
	   	}
		sleep (5);
	}
    do
	{  
		i4_err = UART0_Init(i4_uart_fd, set_speed, 0, 8, 1, 'N'); 
        printf("readCOMThread | Init serial Port succes\n");
        
	}while(FALSE == i4_err || FALSE == i4_uart_fd);

    //¿ªÊ¼¼àÌý´®¿Ú
	if (!OpenListenThread())
	{
		printf("readCOMThread | OpenListenThread failed\n");
		return FALSE;
	}

    return TRUE;
}

/******************************************************************* 
* Ãû³Æ£º                  ReadCom 
* ¹¦ÄÜ£º               
* Èë¿Ú²ÎÊý£º        fd    :ÎÄ¼þÃèÊö·û     port :´®¿ÚºÅ(ttymxc1) 
* ³ö¿Ú²ÎÊý£º        ÕýÈ··µ»ØÎª1£¬´íÎó·µ»ØÎª0 
*******************************************************************/ 
int ReadCom(char *pcom_data)
{
    memcpy(pcom_data, tmp_str, 32);
    //printf("%02x, %02x, %02x, =========len=%d, strlen=%d\n", tmp_str[0], tmp_str[1], tmp_str[2], sizeof(tmp_str), strlen(tmp_str));
    
     
    //printf("tmp_str ===%s, len=%d\n", tmp_str, strlen(tmp_str));
    //printf("%02x, %02x, %02x\n", tmp_str[0], tmp_str[1], tmp_str[2]);
    return 0;
}

/******************************************************************* 
* Ãû³Æ£º                  PortClose 
* ¹¦ÄÜ£º               
* Èë¿Ú²ÎÊý£º        fd    :ÎÄ¼þÃèÊö·û     port :´®¿ÚºÅ(ttymxc1) 
* ³ö¿Ú²ÎÊý£º        ÕýÈ··µ»ØÎª1£¬´íÎó·µ»ØÎª0 
*******************************************************************/ 
void PortClose()
{
    listen_thr_stop_ = TRUE;
    //Ïß³Ì½áÊø£¬Ôò¹Ø±Õ¼àÌý´®¿ÚµÄ
    UART0_Close(i4_uart_fd);
    printf("readCOMThread | usrt prot close success.");
}



void ShowUsage()
{
    cout << "Usage   : serial <--name=your name> [Option]" << endl;
    cout << "Options :" << endl;
    cout << " --speed=set_speed                 enter the speed to set the speed for port." << endl;
    cout << " --path=com_parh                   enter the path where you save the data." << endl;
    cout << " --help                            Print this help." << endl;
 
    return;
}


int char2int(char *p){
	int sum = 0;
	while(*p != '\0'){
		sum = sum*10 + (*p - '0');
		p = p+1;
	}
	return sum;
}


int main(int argc, char **argv)
{
    if(argc == 3){
      for(int i=0; i<argc; i++){
        if(strncmp(argv[i] , "--speed=" , 8) == 0){
            set_speed = char2int(&argv[i][8]);
        }
        else if(strncmp(argv[i] , "--path=" , 7) == 0){
           save_path = &argv[i][7];
        }
        else if(strncmp(argv[i], "--help", 6) == 0){
          ShowUsage();
          return 0;
        }
        else if(strncmp(argv[i] , "./serial_port" , 7) == 0){
           continue;
        }
        else{
           cout << "Options '" << argv[i] << "' not valid. Run --help for details." << endl;
            return -1;
        }
      }
      
    }


    else if(argc == 2){
    	for(int i=0 ;i<2;i++){
    	  if(strncmp(argv[i] , "--speed=" , 8) == 0){
    	  	set_speed = char2int(&argv[i][8]);
       	 }
       	 else if(strncmp(argv[i] , "./serial_port" , 7) == 0){
       	    continue;
       	 }
       	 else if(strncmp(argv[i] , "--path=" , 7) == 0){
       	    save_path = &argv[i][7];
       	 }
       	 else if(strncmp(argv[i], "--help", 6) == 0){
       	   ShowUsage();
       	   return 0;
       	 }
       	 else{
       	    cout << "Options '" << argv[i] << "' not valid. Run --help for details." << endl;
       	     return -1;
       	 }
        }   
    }


    else{
      set_speed = 9600;
    }

    cout<<"set speed is "<<set_speed<<endl;

    int ret = 0;
    char * ptmp_str = NULL;
    ret = PortOpen();



    if(ret != 0)
    {
        printf("port open failed!\n");
        return -1;
    }




    while(1)
    {
        ptmp_str = (char *)malloc(64); 
        if(ptmp_str == NULL)
            return -1;
        ret = ReadCom(ptmp_str);


//       printf("ptmp_str ===%02x, %02x, %02x, %02x, %02x, %02x\n", ptmp_str[0], ptmp_str[1], ptmp_str[2], ptmp_str[3], ptmp_str[4], ptmp_str[5]);
	   printf("ptmp_str ===%02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\n", ptmp_str[0], ptmp_str[1], ptmp_str[2], ptmp_str[3], ptmp_str[4], ptmp_str[5], ptmp_str[6], ptmp_str[7], ptmp_str[8], ptmp_str[9], ptmp_str[10], ptmp_str[11], ptmp_str[12], ptmp_str[13], ptmp_str[14], ptmp_str[15], ptmp_str[16], ptmp_str[17], ptmp_str[18], ptmp_str[19], ptmp_str[20], ptmp_str[21], ptmp_str[22], ptmp_str[23]);
        

        free(ptmp_str);
        msleep(50);
    }




    return 0;
}

