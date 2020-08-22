#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "MPU6050_I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
static MPU6050 mpu;

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
static bool dmpReady = false;  // set true if DMP init was successful
static uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
static uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
static uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
static uint16_t fifoCount;     // count of all bytes currently in FIFO
static uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
static Quaternion q;           // [w, x, y, z]         quaternion container
static VectorInt16 aa;         // [x, y, z]            accel sensor measurements
static VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
static VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
static VectorFloat gravity;    // [x, y, z]            gravity vector
static float euler[3];         // [psi, theta, phi]    Euler angle container
static float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
static uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


uint8_t MPU6050_init();
void MPU6050_GetMeasurements(float *yaw,float *pitch,float *roll, int *aax, int *aay, int *aaz, FILE *fplog);
/******************************************************************
*	初期化
******************************************************************/
uint8_t MPU6050_init()
{
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();
    // verify connection
//    printf("Testing device connections...\n");
//    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
	devStatus = mpu.testConnection();

printf("+++ MPU6050 connection [%d]\n",devStatus);

	if(devStatus == 0){
		printf("*** MPU6050 connection failed\n");
		return -1;
	}

    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }

	return devStatus;
}


/******************************************************************
*	測定値獲得
******************************************************************/
void MPU6050_GetMeasurements(float *yaw,float *pitch,float *roll,int *aax, int *aay, int *aaz,FILE *fplog)
{
	char tmp[256];
//	float LR,UD;

    // if programming failed, don't try to do anything
    if (!dmpReady){
        printf("\n");
		return;
	}
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        printf("+++ FIFO overflow reset!\n");
        usleep(100000);  //100ms
//		sleep(1);

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            sprintf(tmp,"quat %7.2f %7.2f %7.2f %7.2f       ", q.w,q.x,q.y,q.z);
//			fputs(tmp,fplog);

        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            printf("euler %7.2f %7.2f %7.2f    ", euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);
        #endif


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);

            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			*yaw 	= ypr[0] * 180/M_PI;
			*pitch	= ypr[1] * 180/M_PI;
			*roll	= ypr[2] * 180/M_PI;
//            printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);



            // display real acceleration, adjusted to remove gravity 追加(重力を除外した加速度)
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//          mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//          printf("areal %6d %6d %6d    \n", aaReal.x, aaReal.y, aaReal.z);





        #endif

//        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            printf("areal %6d %6d %6d    ", aaReal.x, aaReal.y, aaReal.z);
//        #endif

//        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
			*aax = aaWorld.x;
			*aay = aaWorld.y;
			*aaz = aaWorld.z;

//            printf("aworld %6d %6d %6d    \n", aaWorld.x, aaWorld.y, aaWorld.z);
//        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif
//	    printf("\n");
    }
//    printf("\n");

}

//int main() {
//    setup();
//    usleep(100000);
//    for (;;)
//        loop();
//    return 0;
//}

