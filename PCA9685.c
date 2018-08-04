//gcc -Wall -O2 -lm PCA9685.c -o PCA9685
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD


int PCA9685_init(void);
void PCA9685_pwmWrite(uint8_t ch, double pulseWidth_usec);

static char *m_i2cFileName = "/dev/i2c-1";
//static int m_driverAddress = 0x40;
static int m_nI2c;
static uint8_t PCA9685_read(uint8_t adr);
static void PCA9685_write(uint8_t adr, uint8_t dat);
static void PCA9685_setPWM(uint8_t ch, uint16_t onTime, uint16_t offTime);

#define I2C_ADR	0x40
#define PWM_FREQUENCY 60					//60Hz 16.7ms
#define PWM_PULSE_WIDTH_MAX	12000			//12ms

/**********************************************************************
* test
**********************************************************************/
/*
int main(int argc, char *argv[])
{
	char *i2cFileName = "/dev/i2c-1";
	int driverAddress = 0x40;
	int i;
	double dfVal;

	if((m_nI2c = open(i2cFileName, O_RDWR)) < 0){
		printf("m_nI2c open err\n");
		return -1;
	}
 
	if(ioctl(m_nI2c, I2C_SLAVE, driverAddress) < 0){
		printf("ioctl err\n");
		return -1;
	}

	PCA9685_init(PWM_FREQUENCY);

	for(i=0;i<100;i++){
        dfVal = i*200+1000;
		if(PWM_PULSE_WIDTH_MAX < dfVal)break;
		printf("%0.1lfms\n",dfVal/1000);
		PCA9685_pwmWrite(0, dfVal);
		PCA9685_pwmWrite(1, PWM_PULSE_WIDTH_MAX - dfVal);
		sleep(1);
	}

	printf("stop\n");
	sleep(5);
	PCA9685_pwmWrite(0, 0);


	return 0;
}
*/

/**********************************************************************
*	PCA9685_init
**********************************************************************/
int PCA9685_init(void)
{
	float prescaleval = 25000000;

	if((m_nI2c = open(m_i2cFileName, O_RDWR)) < 0){
		printf("m_nI2c open err\n");
		return -1;
	}
 
	if(ioctl(m_nI2c, I2C_SLAVE, I2C_ADR) < 0){
		printf("ioctl err\n");
		return -1;
	}

	PCA9685_write(PCA9685_MODE1, 0x0);
	usleep(100000);//100ms

	prescaleval /= 4096;
	prescaleval /= PWM_FREQUENCY;
	prescaleval -= 1;
	printf("Estimated pre-scale: %f\n", prescaleval);

	uint8_t prescale = floor(prescaleval + 0.5);
	printf("Final pre-scale: %d\n", prescale); 

	uint8_t oldmode = PCA9685_read(PCA9685_MODE1);
	uint8_t newmode = (oldmode&0x7F) | 0x10; 
	PCA9685_write(PCA9685_MODE1, newmode); 
	PCA9685_write(PCA9685_PRESCALE, prescale);
	PCA9685_write(PCA9685_MODE1, oldmode);
//	sleep(5);
	sleep(1);
	PCA9685_write(PCA9685_MODE1, oldmode | 0xa1);  

	return 0;
}


/**********************************************************************
*	PCA9685_pwmWrite
**********************************************************************/
void PCA9685_pwmWrite(uint8_t ch, double pulseWidth_usec)
{
	double pulselength;
	double pulseWidth;
	// 1秒=1000000usを60Hzで割ってパルス長を算出。
	pulselength = 1000000 / PWM_FREQUENCY;
	// 12bit(2^12=4096)分解能相当へ。1分解能当たりの時間算出。
	pulselength /= 4096;
	// PWMのパルス設定値を算出。
	pulseWidth = pulseWidth_usec / pulselength;

	// PWM値設定。
	//  setPWM(channel, on_timing, off_timing)
	//  channelで指定したチャネルのPWM出力のon(0→1）になるタイミングと
	//  off(1→0)になるタイミングを0～4095で設定する。
	PCA9685_setPWM(ch, 0, pulseWidth);
}
/**********************************************************************
*	PCA9685_setPWM
**********************************************************************/
static void PCA9685_setPWM(uint8_t ch, uint16_t onTime, uint16_t offTime)
{
	uint8_t sendData[5];

	sendData[0] = LED0_ON_L + 4 * ch;
	sendData[1] = (uint8_t)(0x00ff & onTime);
	sendData[2] = (uint8_t)((0xff00 & onTime) >> 8);
	sendData[3] = (uint8_t)(0x00ff & offTime);
	sendData[4] = (uint8_t)((0xff00 & offTime) >> 8);
 
	if(write(m_nI2c, sendData, 5) != 5){
		printf("PCA9685_setPWM() err\n");
	}
}


/**********************************************************************
*	PCA9685_read
**********************************************************************/
static uint8_t PCA9685_read(uint8_t adr)
{
	uint8_t sendData;
	uint8_t readData;

	sendData = adr;
	if(write(m_nI2c, &sendData, 1) != 1){
		printf("PCA9685_read() err1\n");
	}
	else{
		if(read(m_nI2c, &readData, 1) != 1){
			printf("PCA9685_read() err2\n");
		}
	 }

	return readData;
}
/**********************************************************************
*	PCA9685_write
**********************************************************************/
static void PCA9685_write(uint8_t adr, uint8_t dat)
{
	uint8_t buf[2];

	buf[0] = adr;
	buf[1] = dat;
	if(write(m_nI2c, buf, 2) != 2){
		printf("PCA9685_write() err\n");
	}
}


