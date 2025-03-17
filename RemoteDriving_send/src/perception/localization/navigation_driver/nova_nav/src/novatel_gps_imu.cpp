#include "novatel_gps_imu.h"

/**
 * @brief udp_client_init  
 * @param server_port
 * @return
 */
int udp_client_init(const int server_port)
{
    /*set server IP and PORT*/
    struct sockaddr_in serveraddr;
    bzero(&serveraddr,sizeof(serveraddr));  //将字符串serveraddr的所有字节清为0

    serveraddr.sin_family = AF_INET;     //sin_family网络协议
    serveraddr.sin_port = htons(server_port);
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);

    //创建套接字
    int client_sock_fd = socket(AF_INET,SOCK_DGRAM,0);   //成功返回0，失败返回-1
    if(client_sock_fd == -1)
    {
        perror("create client_sock_fd failed!");
        exit(-1);
    }

    //如果绑定错误
    if(bind(client_sock_fd,(struct sockaddr *)&serveraddr,sizeof(serveraddr)) == -1){
        perror("udp client socket)fd bind failed1");
        exit(-1);
      }

    return client_sock_fd;
}

//-------------------------CRC校验--------------------------------------------------
#define CRC32_POLYNOMIAL 0xEDB88320L

/**
 * @brief CRC32Value Calculate a CRC value to be used by CRC calculation functions.
 * @param i
 * @return
 */
static unsigned long CRC32Value(int i)
{
    int j;
    unsigned long ulCRC;
       ulCRC = i;
    for ( j = 8 ; j > 0; j-- )
    {
    if ( ulCRC & 1 )
    ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
    else
    ulCRC >>= 1;
    }
    return ulCRC;
}

/**
 * @brief CalculateBlockCRC32  Calculates the CRC-32 of a block of data all at once
 * @param ulCount   Number of bytes in the data block
 * @param ucBuffer Data block
 * @return
 */
unsigned long CalculateBlockCRC32( unsigned long ulCount, unsigned char *ucBuffer)
{
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    while ( ulCount-- != 0 )
    {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return( ulCRC );
}


double rad(double x) { return x * M_PI / 180.0; }  //将度转为弧度
double deg(double x) { return x * 180 / M_PI; }    //将弧度转为度



/**
 * @brief move_gps_point  Moving GPS points horizontally
 * @param lon1、lat1、dist、heading are input parameter
 * @param lon2、lat2 are output parameter
 * @return
 */

void move_gps_point( double lon1,double lat1,double dist,double heading,double &lon2,double &lat2 )
{

    double a = 6378137;
    double b = 6356752.3142;
    double f = 1/298.257223563;
    double alpha1 = rad(heading) ;    //+ atan(0.1/4.35)
    double s = dist;   ///cos( atan(0.1/4.35) )
    double sinAlpha1 = sin(alpha1);
    double cosAlpha1 = cos(alpha1);
    
    double tanU1 = (1-f) * tan( rad(lat1) );
    double cosU1 = 1 / sqrt((1 + tanU1*tanU1));
	double sinU1 = tanU1*cosU1;
    double sigma1 = atan2(tanU1, cosAlpha1);
    double sinAlpha = cosU1 * sinAlpha1;
    double cosSqAlpha = 1 - sinAlpha*sinAlpha;
    double uSq = cosSqAlpha * (a*a - b*b) / (b*b);
    double A = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
    double B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));
    
    double sigma = s / (b*A);
	double sigmaP = 2*M_PI;
	
	double cos2SigmaM ;
    double sinSigma;
    double cosSigma ;
    double deltaSigma;
    while (fabs(sigma-sigmaP) > 1e-12) {
        cos2SigmaM = cos(2*sigma1 + sigma);
        sinSigma = sin(sigma);
        cosSigma = cos(sigma);
        deltaSigma = B*sinSigma*(cos2SigmaM+B/4*(cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)-
            B/6*cos2SigmaM*(-3+4*sinSigma*sinSigma)*(-3+4*cos2SigmaM*cos2SigmaM)));
            sigmaP = sigma;
            sigma = s / (b*A) + deltaSigma;
        }


    double tmp = sinU1*sinSigma - cosU1*cosSigma*cosAlpha1;
           lat2 = atan2(sinU1*cosSigma + cosU1*sinSigma*cosAlpha1,
            (1-f)*sqrt(sinAlpha*sinAlpha + tmp*tmp));
            double lambda = atan2(sinSigma*sinAlpha1, cosU1*cosSigma - sinU1*sinSigma*cosAlpha1);
            double C = f/16*cosSqAlpha*(4+f*(4-3*cosSqAlpha));
            double L = lambda - (1-C) * f * sinAlpha *
                (sigma + C*sinSigma*(cos2SigmaM+C*cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)));

            double revAz = atan2(sinAlpha, -tmp);  // final bearing

            lon2 = lon1+deg(L);
   		    lat2 = deg(lat2)	;

    // int a = 6378137;
    // double b = 6356752.3142;
    // double f = 1/298.257223563;
    // double alpha1 = rad(heading) + atan(0.2/4.35);;//+ atan(0.1/4.35);
    // double s = dist + 0;///cos( atan(0.2/4.35) );
    // double sinAlpha1 = sin(alpha1);
    // double cosAlpha1 = cos(alpha1);
    
    // double tanU1 = (1-f) * tan( rad(lat1) );
    // double cosU1 = 1 / sqrt((1 + tanU1*tanU1)), sinU1 = tanU1*cosU1;
    // double sigma1 = atan2(tanU1, cosAlpha1);
    // double sinAlpha = cosU1 * sinAlpha1;
    // double cosSqAlpha = 1 - sinAlpha*sinAlpha;
    // double uSq = cosSqAlpha * (a*a - b*b) / (b*b);
    // double A = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
    // double B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));
    
    // double sigma = s / (b*A);
    // double sigmaP = 2*M_PI;
	
    // double cos2SigmaM ;
    // double sinSigma;
    // double cosSigma ;
    // double deltaSigma;
    // while (fabs(sigma-sigmaP) > 1e-12) {
    //     cos2SigmaM = cos(2*sigma1 + sigma);
    //     sinSigma = sin(sigma);
    //     cosSigma = cos(sigma);
    //     deltaSigma = B*sinSigma*(cos2SigmaM+B/4*(cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)-
    //         B/6*cos2SigmaM*(-3+4*sinSigma*sinSigma)*(-3+4*cos2SigmaM*cos2SigmaM)));
    //         sigmaP = sigma;
    //         sigma = s / (b*A) + deltaSigma;
    //     }


    // double tmp = sinU1*sinSigma - cosU1*cosSigma*cosAlpha1;
	// 	   lat2 = atan2(sinU1*cosSigma + cosU1*sinSigma*cosAlpha1,
    //         (1-f)*sqrt(sinAlpha*sinAlpha + tmp*tmp));
    //         double lambda = atan2(sinSigma*sinAlpha1, cosU1*cosSigma - sinU1*sinSigma*cosAlpha1);
    //         double C = f/16*cosSqAlpha*(4+f*(4-3*cosSqAlpha));
    //         double L = lambda - (1-C) * f * sinAlpha *
    //             (sigma + C*sinSigma*(cos2SigmaM+C*cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)));

    //         double revAz = atan2(sinAlpha, -tmp);  // final bearing

	// 	   lon2 = lon1+deg(L);
   	// 	   lat2 = deg(lat2)	;
	
}
