/*
This program reads the the raw gyroscope, accelerometer and magnetometer data
on a BerryIMU connected to a Raspberry Pi.
*/

#include <sys/time.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include "IMU.c"

#define DT 0.02         // [s/loop] loop period. 20ms
#define AA 0.97         // complementary filter constant

#define A_GAIN 0.0573    // [deg/LSB]
#define G_GAIN 0.070     // [deg/s/LSB]

void  INThandler(int sig)// Used to do a nice clean exit when Ctrl-C is pressed
{
  signal(sig, SIG_IGN);
  exit(0);
}

int mymillis()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
  long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
  result->tv_sec = diff / 1000000;
  result->tv_usec = diff % 1000000;
  return (diff<0);
}

int main(int argc, char *argv[])
{
  float rate_gyr_y = 0.0;   // [deg/s]
  float rate_gyr_x = 0.0;   // [deg/s]
  float rate_gyr_z = 0.0;   // [deg/s]

  int  accRaw[3];
  int  magRaw[3];
  int  gyrRaw[3];

  int startInt  = mymillis();
  struct  timeval tvBegin, tvEnd,tvDiff;

  signal(SIGINT, INThandler);

  detectIMU();
  enableIMU();

  gettimeofday(&tvBegin, NULL);

  //printf("Loop Time 0\t");
  printf("LoopTime,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,MagX,MagY,MagZ\n");
  printf("0,");
  while(1)
  {
    startInt = mymillis();

    //read ACC and GYR data
    readACC(accRaw);
    readGYR(gyrRaw);

    //Convert Gyro raw to degrees per second
    rate_gyr_x = (float) gyrRaw[0]  * G_GAIN;
    rate_gyr_y = (float) gyrRaw[1]  * G_GAIN;
    rate_gyr_z = (float) gyrRaw[2]  * G_GAIN;

    //read magnetometer data
    readMAG(magRaw);

    //printf(" GyroX %i \t GyroY %i \t GyroZ %i \t",rate_gyr_x,rate_gyr_y,rate_gyr_z);
    //printf(" AccX %i \t AccY %i \t AccZ %i \t",accRaw[0],accRaw[1],accRaw[2]);
    //printf(" MagX %i \t MagY %i \t MagZ %i \n",magRaw[0],magRaw[1],magRaw[2]);
    printf("%.3f,%.3f,%.3f,",rate_gyr_x,rate_gyr_y,rate_gyr_z);
    printf("%.3f,%.3f,%.3f,",accRaw[0],accRaw[1],accRaw[2]);
    printf("%i,%i,%i\n",magRaw[0],magRaw[1],magRaw[2]);

    //Each loop should be at least 20ms.
    while(mymillis() - startInt < (DT*1000)){
      usleep(100);
    }

    //printf("Loop Time %d\t", mymillis()- startInt);
    printf("%d,", mymillis()- startInt);
  }
}
