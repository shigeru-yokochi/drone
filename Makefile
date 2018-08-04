CC = g++

CFLAGS = -O0 -g -Wall -c -I./ -I./VL53L0X_1.0.2_API/inc/platform -I./VL53L0X_1.0.2_API/inc/core
LDFLAGS       = -L/usr/local/lib 
LIBS          = -lm -lbluetooth
OBJS          = PCA9685.o MPU6050_I2Cdev.o MPU6050_core.o mpu6050.o vl53l0x.o main.o ble-scan-rssi.o hmc5883l.o
PROGRAM	= main

all:		$(PROGRAM)

$(PROGRAM):		$(OBJS)
				$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $(PROGRAM) ./VL53L0X_1.0.2_API/lib/libVL53L0X_Rasp.a

clean:;		rm -f *.o $(PROGRAM)

