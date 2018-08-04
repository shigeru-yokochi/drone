#include <stdio.h>
#include <linux/i2c-dev.h> 
#include <fcntl.h>
#include <math.h>	//-lm
#include <sys/ioctl.h>
#include <unistd.h>

int HMC5883L_init(void);
int HMC5883L_GetDirection(void);

static int m_fd;
static int m_nDirection;

static int HMC5883L_write(unsigned char address, unsigned char data);
static unsigned char HMC5883L_read(unsigned char address);
static void HMC5883L_readData(int *idata);
/*****************************************************************
*	初期化
*****************************************************************/
int HMC5883L_init(void)
{
	char *i2cfile = "/dev/i2c-1";
	unsigned char i2caddr = 0x1e;
	unsigned char data;

	if ((m_fd = open(i2cfile, O_RDWR)) < 0) {
		printf("Faild to open i2c port\n");
		return -1;
	}
	if (ioctl(m_fd, I2C_SLAVE, i2caddr) < 0) {
		printf("Unable to get bus access to talk to slave\n");
		return -1;
	}
	data = HMC5883L_read(0x0a);
	if (data != 0x48) {
		printf("Identification Register 0x0a check failure [%02x]", data);
		return -1;
	}
	data = HMC5883L_read(0x0b);
	if (data != 0x34) {
		printf("Identification Register 0x0b check failure [%02x]", data);
		return -1;
	}
	data = HMC5883L_read(0x0c);
	if (data != 0x33) {
		printf("Identification Register 0x0c check failure [%02x]", data);
		return -1;
	}
	HMC5883L_write(0x00, 0xe0); /* set default val to Config Reg A */
	HMC5883L_write(0x02, 0x00); /* set continuous mode to Mode Reg */
	
	m_nDirection = 0;

	return 0;
}
/*****************************************************************
*	方角獲得 0..359 0:北
*****************************************************************/
int HMC5883L_GetDirection(void)
{
	int nCorrectionValue = 20;
	unsigned char status;
	int data[3];

	status = HMC5883L_read(0x09);
	if (status | 0x1) {
		HMC5883L_readData(data);
		m_nDirection = (int)(atan2((double)data[0] + nCorrectionValue, ((double)data[2] + nCorrectionValue)*(-1)) * 180 / 3.14159265358979 + 180);
	}

	return m_nDirection;
}
/*****************************************************************
*	write
*****************************************************************/
static int HMC5883L_write(unsigned char address, unsigned char data)
{
	unsigned char buf[2];
	buf[0] = address;
	buf[1] = data;
	if ((write(m_fd, buf, 2)) != 2) {
		printf("Error writing to i2c slave\n");
		return -1;
	}

	return 0;
}
/*****************************************************************
*	read
*****************************************************************/
static unsigned char HMC5883L_read(unsigned char address)
{
	unsigned char buf[1];
	buf[0] = address;
	if ((write(m_fd, buf, 1)) != 1) {
		printf("Error writing to i2c slave\n");
	}
	if (read(m_fd, buf, 1) != 1) {
		printf("Error reading from i2c slave\n");
	}

	return buf[0];
}
/*****************************************************************
*	data read
*****************************************************************/
static void HMC5883L_readData(int *idata)
{
	unsigned char i, addr, data[6];

	for (i = 0; i<6; i++) {
		addr = 0x03 + i;
		data[i] = HMC5883L_read(addr);
	}
	idata[0] = ((int)data[0] << 24 | (int)data[1] << 16) >> 16;
	idata[1] = ((int)data[2] << 24 | (int)data[3] << 16) >> 16;
	idata[2] = ((int)data[4] << 24 | (int)data[5] << 16) >> 16;
}
