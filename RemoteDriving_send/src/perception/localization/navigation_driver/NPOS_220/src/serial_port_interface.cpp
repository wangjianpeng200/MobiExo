#include "serial_port_interface.h"


/**
 * @brief uart_open
 * @param ttysn
 * @return
 */
int uart_open(const char *ttysn){
    int fd = open(ttysn,O_RDWR|O_NOCTTY); //| O_NDELAY
    if(fd == -1){
        perror("Open UART failed!");
        return -1;
    }
    return fd;
}


/**
 * @brief uart_conf_set
 * @param fd
 * @param nBaud  B115200
 * @param nBits  8
 * @param nStop  1
 * @param nEvent 0
 * @param c_flow 0
 * @return
 */

int uart_conf_set(int fd,int nBaud,int nBits,int nStop,char nEvent){
    struct termios options_new,options_old;
    bzero( &options_new, sizeof(options_new));

    //-->>获取终端属性
    if(tcgetattr(fd,&options_old) < 0){
        perror("tcgetattr error");
        return -1;
    }
    //options_new =  options_old;  //???
    options_new.c_cflag |= CLOCAL | CREAD;


    //-->>设置波特率
    switch(nBaud){
        case 9600:
            cfsetispeed(&options_new,B9600);   cfsetospeed(&options_new,B9600);   break;
        case 19200:
            cfsetispeed(&options_new,B19200);  cfsetospeed(&options_new,B19200);  break;
        case 115200:
            cfsetispeed(&options_new,B115200); cfsetospeed(&options_new,B115200); break;
        case 230400:
            cfsetispeed(&options_new,B230400); cfsetospeed(&options_new,B230400); break;
        default:
            fprintf(stderr,"Unkown baude!\n");
            return -1;
    }

   //-->>设置数据位
    switch(nBits){
        case 5:
            options_new.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options_new.c_cflag |= CS5;
            break;
        case 6:
            options_new.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options_new.c_cflag |= CS6;
            break;
        case 7:
            options_new.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options_new.c_cflag |= CS7;
            break;
        case 8:
            options_new.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options_new.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unkown bits!\n");
            return -1;
    }

    //-->>设置停止位
    switch(nStop){
        case 1:
            options_new.c_cflag &= ~CSTOPB;//CSTOPB：使用1位停止位
            break;
        case 2:
            options_new.c_cflag |= CSTOPB;//CSTOPB：使用两位停止位
            break;
        default:
            fprintf(stderr,"Unkown stop!\n");
            return -1;
    }

    //-->>设置校验位
    switch(nEvent){
        /*无奇偶校验位*/
        case 'n':
        case 'N':
            options_new.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            //options_new.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
            break;
        /*设为空格,即停止位为2位*/
        case 's':
        case 'S':
            options_new.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options_new.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            break;
        /*设置奇校验*/
        case 'o':
        case 'O':
            options_new.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options_new.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options_new.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options_new.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        /*设置偶校验*/
        case 'e':
        case 'E':
            options_new.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options_new.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options_new.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options_new.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        default:
            fprintf(stderr,"Unkown parity!\n");
            return -1;
    }

//    /*设置数据流控制*/
//    switch(０){
//        case 0://不进行流控制
//            options_new.c_cflag &= ~CRTSCTS;
//            break;
//        case 1://进行硬件流控制
//            options_new.c_cflag |= CRTSCTS;
//            break;
//        case 2://进行软件流控制
//            options_new.c_cflag |= IXON|IXOFF|IXANY;
//            break;
//        default:
//            fprintf(stderr,"Unkown c_flow!\n");
//            return -1;
//    }


//   /*设置控制模式*/
//    options_new.c_cflag |= CLOCAL;            //保证程序不占用串口
//    options_new.c_cflag |= CREAD;             //保证程序可以从串口中读取数据
//    options_new.c_cflag &= ~CSIZE;

    /*设置等待时间和最小接受字符*/
    options_new.c_cc[VTIME] = 0;//可以在select中设置
    options_new.c_cc[VMIN] = 0;//最少读取一个字符

//    /*设置输出模式为原始输出*/
    options_new.c_oflag &= ~OPOST;//OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

//    /*设置本地模式为原始模式
//     *ICANON：允许规范模式进行输入处理
//     *ECHO：允许输入字符的本地回显
//     *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
//     *ISIG：允许信号
//     */
//    options_new.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /*如果发生数据溢出，只接受数据，但是不进行读操作*/
    tcflush(fd,TCIFLUSH);

    /*激活配置*/
    if(tcsetattr(fd,TCSANOW,&options_new) < 0){
        perror("tcsetattr failed");
        return -1;
    }

    return 0;
}
