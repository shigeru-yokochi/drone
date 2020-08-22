﻿#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <wiringPi.h>

//VL53L0X
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"


//BLE
extern int Ble_init(int nBeaconMax, char *cpAdr1, char *cpAdr2, char *cpAdr3, char *cpAdr4);
extern int BleRSSI();
extern int Ble_close();

//VL53L0X
#define VL53L0X_MAX 5
#define VL53L0X_XSHUT_1_GPIO 26
#define VL53L0X_XSHUT_2_GPIO 19
#define VL53L0X_XSHUT_3_GPIO 20
#define VL53L0X_XSHUT_4_GPIO 16
#define VL53L0X_XSHUT_5_GPIO 21	//Altitude
extern VL53L0X_Error VL53L0X_init(uint16_t xshut_gpio,uint16_t i2c_address,uint16_t device_id);
extern void VL53L0X_close(uint16_t device_id);
extern VL53L0X_Error VL53L0X_GetMeasurements(uint16_t *pVL53L0X_Measurement,uint16_t device_id);
//MPU6050
extern uint8_t MPU6050_init();
extern void MPU6050_GetMeasurements(float *yaw,float *pitch,float *roll, int *aax, int *aay, int *aaz, FILE *fplog);
//PCA9685
extern int PCA9685_init(void);
extern void PCA9685_pwmWrite(uint8_t ch, double pulseWidth_usec);
//#define PWM_PULSE_WIDTH_MAX	12000			//12ms

//HMC5883L
extern int HMC5883L_init(void);
extern int HMC5883L_GetDirection(void);


//main.c
static int I2c_device_init(void);
static int Debug_Print_init(void);
static void DebugPrint(char *buf,FILE *fp);
static void BLHeli_init(void);
static int SetPwmBalance(int nPwm1,int nPwm2);
static int PwmBarance(void);
static void HeadNorth(int nDirection, int *npPower1, int *npPower2, int *npPower3, int *npPower4);
void GetAttitudeControl(double *dfpPower);
static void BETAFPV_F4_2S_AIO_Main_Loop(void);
static bool Get_Correction_Power(uint16_t d1,uint16_t d2,int *correction_power);
static int Get_Altitude_Ctrl_Event(uint16_t alt,uint16_t alt_save);
static void Get_Horizontal_Level_Power(float val,int *correction_power);
static bool Get_Altitude_Ctrl_Power(int event,int *status,int *correction_power);

//BLE BEACON
#define BLE_BEACON_MAX	2	//1個使用 最大4個
#define BLE_BEACON_ADR1	"B8:27:EB:55:B7:DC"
#define BLE_BEACON_ADR2 "B8:27:EB:DE:1F:0C"
#define BLE_BEACON_ADR3 ""
#define BLE_BEACON_ADR4 ""


//PID制御
//#define DELTA_T 0.0625		//1秒間に16回
#define DELTA_T 0.0333		//1秒間に30回
//PID制御(pitch)
#define P_KP		-0.6		//比例
#define P_KI		0.03		//積分
#define P_KD		0.0001		//微分
float GetPIDPitch(float fSensorVal,float fTragetVal);
static float m_fDiffP[2];
static float m_fIntegralP;
//PID制御(roll)
#define R_KP		-0.6		//比例
#define R_KI		0.03		//積分
#define R_KD		0.0001		//微分
float GetPIDRoll(float fSensorVal,float fTragetVal);
static float m_fDiffR[2];
static float m_fIntegralR;

//debug用
static FILE *m_fp,*m_fpVL53L0X;	

//BLHeli PWM値
//#define PWM_MIN		1180
#define PWM_MIN		1190
#define PWM_MAX		1600
//#define PWM_BASE_POWER	370 
//#define PWM_BASE_POWER	290	//test
//#define PWM_BASE_POWER	240	//test
#define PWM_BASE_POWER	240	//test
//#define PWM_POWER	0.2



//BETAFPV F4 2S AIO Brushless No Rx
#define BETAFPV_F4_2S_AIO_ROLL		0
#define BETAFPV_F4_2S_AIO_PITCH		1
#define BETAFPV_F4_2S_AIO_THROTTLE	2
#define BETAFPV_F4_2S_AIO_YAW		3
#define BETAFPV_F4_2S_AIO_AUX1		4
#define BETAFPV_F4_2S_AIO_ARM		4	//aux1
#define BETAFPV_F4_2S_AIO_AUX2		5
#define BETAFPV_F4_2S_AIO_AUX3		6
#define BETAFPV_F4_2S_AIO_AUX4		7
#define BETAFPV_F4_2S_AIO_ARM_OFF	1000
#define BETAFPV_F4_2S_AIO_ARM_ON	1500
#define	BETAFPV_F4_2S_AIO_NEUTRAL			1500
#define	BETAFPV_F4_2S_AIO_NEUTRAL_THROTTLE	950
#define THRESHOLD_DISTANCE 250	//障害物回避距離(mm)
#define CORRECTION_POWER_P 50	//障害物回避用の出力補正値
#define CORRECTION_POWER_N -50	//障害物回避用の出力補正値


#define MINIMUM_GROUND_CLEARANCE	40	//最小地上高(mm)
#define MAXIMUM_GROUND_CLEARANCE	500	//最大地上高(mm)

#define DEBUG_MAINLOOP_TO			5	//デバッグ用メインループタイムアウト指定(sec)
#define FLIGHT_TIME					4	//DEBUG_MAINLOOP_TO - FLIGHT_TIME = landing time
#define CLIMB_POWER 				300
#define LANDING_POWER 				200
//#define CLIMB_POWER 				50	//飛ばないテスト用
//#define LANDING_POWER 			30


//姿勢制御用データ格納用
typedef struct {
	int init;
	float yaw;
	float pitch;
	float roll;
	float pre_yaw;
	float pre_pitch;
	float pre_roll;
	int aax;
	int aay;
	int aaz;
	int pre_aax;
	int pre_aay;
	int pre_aaz;

}struct_AttitudeData;
static struct_AttitudeData m_AttitudeData;



/********************************************************************************
*	メイン
********************************************************************************/
int main(int argc, char **argv)
{
//debug-------------
//	int i, nHeadPower[4];
//	for (i = 0; i < 360; i++) {
//		memset(nHeadPower, 0, sizeof(nHeadPower));
//		HeadNorth(i, &nHeadPower[0], &nHeadPower[1], &nHeadPower[2], &nHeadPower[3]);//北に向かう
//		printf("%03d:%03d %03d %03d %03d (%d)(%d)(%d)(%d)\n", i, nHeadPower[0], nHeadPower[1], nHeadPower[2], nHeadPower[3],
//			nHeadPower[0] + nHeadPower[2],
//			nHeadPower[2] + nHeadPower[1],
//			nHeadPower[1] + nHeadPower[3],
//			nHeadPower[3] + nHeadPower[0]);
//	}
//	return 0;
//---------debug

	if(wiringPiSetupGpio() == -1){
        printf("wiringPiSetupGpio() error\n");
        return -1;
    }

	if (Ble_init(BLE_BEACON_MAX, BLE_BEACON_ADR1, BLE_BEACON_ADR2, BLE_BEACON_ADR3, BLE_BEACON_ADR4) == -1)return -1;	//BLE初期化
	if(I2c_device_init() == -1)return -1;	//i2cデバイス初期化
	if(Debug_Print_init() == -1)return -1;	//デバッグ出力用ファイル初期化
//	BLHeli_init();							//BLHeli(ESC)初期化
//	if(PwmBarance() == -1)goto TAG_EXIT;	//各pwmの出力調整


//goto TAG_EXIT;

	//PID制御値初期化
	m_fDiffP[0] = 0.;
	m_fDiffP[1] = 0.;
	m_fIntegralP = 0.;
	m_fDiffR[0] = 0.;
	m_fDiffR[1] = 0.;
	m_fIntegralR = 0.;


	printf("DELTA_T=%0.4lf\nPKp=%0.4lf\nPKi=%0.4lf\nPKd=%0.4lf\n\PWM_BASE_POWER=%d\n",DELTA_T,P_KP,P_KI,P_KD, PWM_BASE_POWER);

	BETAFPV_F4_2S_AIO_Main_Loop();
TAG_EXIT:
	//終了処理
	PCA9685_pwmWrite(0, 0);
	PCA9685_pwmWrite(1, 0);
	PCA9685_pwmWrite(2, 0);
	PCA9685_pwmWrite(3, 0);
	PCA9685_pwmWrite(4, 0);
	PCA9685_pwmWrite(5, 0);
	PCA9685_pwmWrite(6, 0);
	

	VL53L0X_close(4);

	Ble_close();

	fclose(m_fp);
	fclose(m_fpVL53L0X);

    return 0;
}

/********************************************************************************
*	BETAFPV F4 2S AIO Brushless No Rx用　メインループ
********************************************************************************/
static void BETAFPV_F4_2S_AIO_Main_Loop(void)
{
	uint16_t VL53L0X_Measurement[VL53L0X_MAX];		//VL53L0X_MAX台分の測定値(mm)
	uint16_t save_altitude = 0;						//前回の高度を保存
//	char tmp[256];
//	double dfPower[4];		//pwm1..4の個別用出力調整値
//	int nHeadPower[4];		//指定方向へ移動するための出力値
//	int nDirection;
//	int nOffsetPower = OFFSET_POWER;
//	int nMode= 0;		//最大地上高検知:1
//	int nSaveHeight = 0;
	int roll_power = 0; 		//roll補正値
	int pitch_power = 0; 		//pitch補正値
	int altitude_event  = 1;  	//高度制御イベント値
	int altitude_status = 1;  	//高度制御状態値
	int altitude_power = 0;   	//高度制御用出力値
	struct timeval tv_start;
	struct timeval tv_now;
	double dfFlightTimeStart;
	double dfFlightTime;
//	double dfFlightTimeSave=0.;


	//BETAFPV_F4_2S_AIO init
	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_ROLL		, BETAFPV_F4_2S_AIO_NEUTRAL);
	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_PITCH	, BETAFPV_F4_2S_AIO_NEUTRAL);
	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_YAW		, BETAFPV_F4_2S_AIO_NEUTRAL);
	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_THROTTLE	, BETAFPV_F4_2S_AIO_NEUTRAL_THROTTLE);
	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_ARM		, BETAFPV_F4_2S_AIO_ARM_OFF);
	sleep(1);

	printf("--- arming start\n");
	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_ARM, BETAFPV_F4_2S_AIO_ARM_ON);
	sleep(1);
	memset(&m_AttitudeData, 0, sizeof(m_AttitudeData));//初期化


	gettimeofday(&tv_start, NULL);		//start time
	dfFlightTimeStart = ((double)(tv_start.tv_usec) / 1000000) + tv_start.tv_sec;
	//	printf("%ld %06lu\n", tv_start.tv_sec, tv_start.tv_usec);

//	start_time = time(NULL);

	//上昇開始
	altitude_event = 1;
	Get_Altitude_Ctrl_Power(altitude_event,&altitude_status,&altitude_power);
	//throttle
	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_THROTTLE, (double)(BETAFPV_F4_2S_AIO_NEUTRAL_THROTTLE + altitude_power));



	printf("--- started main loop.\n");
	for(;;){
		gettimeofday(&tv_now, NULL);		//now time
		dfFlightTime = ((double)(tv_now.tv_usec) / 1000000) + tv_now.tv_sec - dfFlightTimeStart;
//		printf("Flight time %0.6lf\n", dfFlightTime);
		if (dfFlightTime > DEBUG_MAINLOOP_TO) {						//debug 指定秒で終了する
			printf("--- stop. Debug Time out. [%dsec]\n", DEBUG_MAINLOOP_TO);
			break;
		}



		if (dfFlightTime > FLIGHT_TIME) {
			//ランディング開始
			altitude_event = 5;
			Get_Altitude_Ctrl_Power(altitude_event,&altitude_status,&altitude_power);
			//throttle
			PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_THROTTLE, (double)(BETAFPV_F4_2S_AIO_NEUTRAL_THROTTLE + altitude_power));
		}





//		if(c_GetMeasurements(&VL53L0X_Measurement[0],0) != VL53L0X_ERROR_NONE)break;	//VL53L0X測定値獲得 1
//		if(VL53L0X_GetMeasurements(&VL53L0X_Measurement[1],1) != VL53L0X_ERROR_NONE)break;	//VL53L0X測定値獲得 2
//		if(VL53L0X_GetMeasurements(&VL53L0X_Measurement[2],2) != VL53L0X_ERROR_NONE)break;	//VL53L0X測定値獲得 3
//		if(VL53L0X_GetMeasurements(&VL53L0X_Measurement[3],3) != VL53L0X_ERROR_NONE)break;	//VL53L0X測定値獲得 4
		save_altitude = VL53L0X_Measurement[4];												//前回の高度を保存
		if(VL53L0X_GetMeasurements(&VL53L0X_Measurement[4],4) != VL53L0X_ERROR_NONE)break;	//VL53L0X測定値獲得 5 高度

		//最大地上高で静止させる
//		if (dfFlightTime <= FLIGHT_TIME) {
//			if (VL53L0X_Measurement[4] > MAXIMUM_GROUND_CLEARANCE) {
//				if(nMode == 1 && nSaveHeight > VL53L0X_Measurement[4]){//降下を検知したら再上昇させる
//					nOffsetPower = OFFSET_POWER+20;
//					nMode= 0;	
//				}
//				else{
//					nMode= 1;	//最大地上高検知
//					nOffsetPower = LANDING_POWER;//降下開始
//					nSaveHeight = VL53L0X_Measurement[4];
//				}
//			}
//			else {
//				nOffsetPower = OFFSET_POWER;
//			}
//		}


//	    sprintf(tmp,"VL53L0X:%dmm\n", VL53L0X_Measurement[4]);								//degbug地上高(mm)
//		DebugPrint(tmp,m_fpVL53L0X);													//debug用

		if (dfFlightTime >= 2.0) {
			if(VL53L0X_Measurement[4] < MINIMUM_GROUND_CLEARANCE){								//最小地上高未満だったら終了する
				printf("--- stop. Minimum ground clearance. [%dmm][%dmm]\n",VL53L0X_Measurement[4],MINIMUM_GROUND_CLEARANCE);
				break;
			}
		}


		//ジャイロ/加速度(+90度で設置しているのでrollとpitchを逆にしている)
		MPU6050_GetMeasurements(&m_AttitudeData.yaw,
								&m_AttitudeData.roll,
								&m_AttitudeData.pitch,
								&m_AttitudeData.aax,
								&m_AttitudeData.aay,
								&m_AttitudeData.aaz,
								m_fp);//MPU6050測定値獲得

//		GetAttitudeControl(dfPower);//姿勢制御値獲得





		//高度制御イベント値
		altitude_event = Get_Altitude_Ctrl_Event(VL53L0X_Measurement[4],save_altitude);	
		Get_Altitude_Ctrl_Power(altitude_event,&altitude_status,&altitude_power);
		//モータ出力(throttle)
		PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_THROTTLE, (double)(BETAFPV_F4_2S_AIO_NEUTRAL_THROTTLE + altitude_power));


		printf("Power %d time %0.2lf VL53L0X: %4d roll_power %4d pitch_power %4d MPU6050(yaw,roll,pitch) %4.2lf %4.2lf %4.2lf\n", 
				altitude_power, 
				dfFlightTime, 
				VL53L0X_Measurement[4],
				roll_power,
				pitch_power,
				m_AttitudeData.yaw,m_AttitudeData.roll,m_AttitudeData.pitch);

	}	//for()


	gettimeofday(&tv_now, NULL);		//now time
	dfFlightTime = ((double)(tv_now.tv_usec) / 1000000) + tv_now.tv_sec - dfFlightTimeStart;
	printf("Flight time %0.6lf\n", dfFlightTime);



	printf("--- throttle neutral\n");
	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_THROTTLE, BETAFPV_F4_2S_AIO_NEUTRAL_THROTTLE);

	printf("--- arming stop!\n");
	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_ARM, BETAFPV_F4_2S_AIO_ARM_OFF);
	sleep(1);	


//test	
//	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_ARM, BETAFPV_F4_2S_AIO_ARM_ON);
//	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_ROLL		, 1600);
//	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_PITCH	, 1700);
//	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_YAW		, 1800);
//	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_THROTTLE	, 1900);
//	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_AUX2		, 1800);
//	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_AUX3		, 1700);
//	sleep(5);	
//	PCA9685_pwmWrite(BETAFPV_F4_2S_AIO_ARM, BETAFPV_F4_2S_AIO_ARM_OFF);
//	sleep(1);	

}
/********************************************************************************
*	障害物回避用の出力補正値獲得（ROLL補正とPITCH補正に使用する）
* 進行方向の障害物との距離がTHRESHOLD_DISTANCE未満になった場合、進行方向を逆にする
* 進行方向と進行逆方向とも障害物との距離がTHRESHOLD_DISTANCE未満になった場合、failになる
********************************************************************************/
static bool Get_Correction_Power(uint16_t d1,uint16_t d2,int *correction_power)
{
	if(d1 < THRESHOLD_DISTANCE){
		if(d2 < THRESHOLD_DISTANCE){
			//補正不能
			*correction_power = 0;
			//printf("+++ THRESHOLD_DISTANCE MINIMUM\n");
			return false;
		}
		//マイナス補正
		*correction_power = CORRECTION_POWER_N;
		return true;
	}
	//プラス補正
	if(d2 < THRESHOLD_DISTANCE){
		*correction_power = CORRECTION_POWER_P;
		return true;
	}
	//補正なし
	*correction_power = 0;
	return true;
}
/********************************************************************************
*	姿勢を水平に近づける出力補正値獲得（ROLL補正とPITCH補正に使用する）
********************************************************************************/
static void Get_Horizontal_Level_Power(float val,int *correction_power)
{
	if(val < -2){
		*correction_power= -20;
		return ;
	}
	if(val > 2){
		*correction_power= 20;
		return ;
	}
	*correction_power = 0;
}
/********************************************************************************
*	高度制御イベント取得
*  取得イベントは2,3,4
********************************************************************************/
static int Get_Altitude_Ctrl_Event(uint16_t alt,uint16_t alt_save)
{
	//50mmを超えた下降検知
	if(alt+50 < alt_save){
		return 4;
	}
	if(alt > MAXIMUM_GROUND_CLEARANCE){
		return 3;
	}
	return 2;
}
/********************************************************************************
*	高度制御出力取得
********************************************************************************/
static bool Get_Altitude_Ctrl_Power(int event,int *status,int *correction_power)
{
	switch(*status){
		case 1:
			if(event == 1){
				*correction_power = CLIMB_POWER;
				*status = 2;
			}
			return true;
		case 2:
			if(event == 3){
				*correction_power = LANDING_POWER;
				*status = 3;
			}
			if(event == 4){
				(*correction_power)+=20;
			}
			if(event == 5){
				*correction_power = LANDING_POWER;
				*status = 5;
			}
			return true;
		case 3:
			if(event == 2){
				*correction_power = CLIMB_POWER;
				*status = 2;
			}
			if(event == 4){
				*correction_power = CLIMB_POWER;
				*status = 4;
			}
			if(event == 5){
				*correction_power = LANDING_POWER;
				*status = 5;
			}
			return true;
		case 4:
			if(event == 2){
				*status = 2;
			}
			if(event == 5){
				*correction_power = LANDING_POWER;
				*status = 5;
			}
			return true;
		case 5:
			return true;
	}
	printf("*** error Get_Altitude_Power() not status.[%d]\n",*status);
	return false;
}

/*****************************************************************
*	姿勢制御値獲得
******************************************************************/
void GetAttitudeControl(double *dfpPower)
{
	float aa=0;	//加速量　0..10000を想定
	float pow;

	if (m_AttitudeData.init == 0) {//初回
		m_AttitudeData.pre_pitch = m_AttitudeData.pitch;
		m_AttitudeData.pre_roll = m_AttitudeData.roll;
		m_AttitudeData.pre_aax = m_AttitudeData.aax;
		m_AttitudeData.pre_aay = m_AttitudeData.aay;
		m_AttitudeData.pre_aaz = m_AttitudeData.aaz;
		m_AttitudeData.init = 1;
		printf("--- m_AttitudeData init OK\n");
		return;
	}
	else {
		//値save
		m_AttitudeData.pre_yaw = m_AttitudeData.yaw;
		m_AttitudeData.pre_pitch = m_AttitudeData.pitch;
		m_AttitudeData.pre_roll = m_AttitudeData.roll;
		m_AttitudeData.pre_aax = m_AttitudeData.aax;
		m_AttitudeData.pre_aay = m_AttitudeData.aay;
		if (m_AttitudeData.init == 1) {//2回目
			m_AttitudeData.init = 2;
		}
		else {
			aa = (float)abs(m_AttitudeData.aaz - m_AttitudeData.pre_aaz);
		}
		m_AttitudeData.pre_aaz = m_AttitudeData.aaz;
		if (m_AttitudeData.aaz == 0) {
			return;
		}
	}

/*
	printf("yaw:%7.2f pitch:%7.2f roll:%7.2f aax:%5d aay:%5d aaz:%5d aaz-abs:%5.0f ", 
		m_AttitudeData.yaw,
		m_AttitudeData.pitch,
		m_AttitudeData.roll,
		m_AttitudeData.aax,
		m_AttitudeData.aay,
		m_AttitudeData.aaz,
		aa);
*/

	aa = aa / 200;
//	aa = aa / 3000;		//加速度をほぼ無視する3000
//	ここから　やりかけ↓
	if (m_AttitudeData.roll < 0) {
		pow = 0 - m_AttitudeData.roll / 1.5;
		dfpPower[0] = 0 - aa - pow;
		dfpPower[3] = 0 - aa - pow;
		dfpPower[1] = aa + pow + 2;
		dfpPower[2] = aa + pow + 2;
	}
	if (m_AttitudeData.roll > 0) {
		pow = m_AttitudeData.roll / 1.5;
		dfpPower[0] = aa + pow + 2;
		dfpPower[3] = aa + pow + 2;
		dfpPower[1] = 0 - aa - pow;
		dfpPower[2] = 0 - aa - pow;
	}

	if (m_AttitudeData.pitch < 0) {
		pow = 0 - m_AttitudeData.pitch / 1.5;
		dfpPower[0] = aa + pow + 2;
		dfpPower[2] = aa + pow + 2;
		dfpPower[1] = 0 - aa - pow;
		dfpPower[3] = 0 - aa - pow;
	}
	if (m_AttitudeData.pitch > 0) {
		pow = m_AttitudeData.pitch / 1.5;
		dfpPower[0] = 0 - aa - pow;
		dfpPower[2] = 0 - aa - pow;
		dfpPower[1] = aa + pow + 2;
		dfpPower[3] = aa + pow + 2;
	}

//	printf("pow[0]:%4.0lf,pow[1]:%4.0lf,pow[2]:%4.0lf,pow[3]:%4.0lf\n", dfpPower[0], dfpPower[1], dfpPower[2], dfpPower[3]);

}






/*****************************************************************
*	PID制御(pitch)
******************************************************************/
float GetPIDPitch(float fSensorVal,float fTragetVal)
{
	float p,i,d;

	m_fDiffP[0] = m_fDiffP[1];
//	m_fDiffP[1] = fSensorVal - fTragetVal;
	m_fDiffP[1] = fTragetVal - fSensorVal;
	m_fIntegralP += (m_fDiffP[0] + m_fDiffP[1]) / 2.0 * DELTA_T;

	p = P_KP * m_fDiffP[1];							//比例
	i = P_KI * m_fIntegralP;							//積分
	d = P_KD * (m_fDiffP[1] - m_fDiffP[0]) / DELTA_T;	//微分

	return p + i + d;
}
/*****************************************************************
*	PID制御(roll)
******************************************************************/
float GetPIDRoll(float fSensorVal,float fTragetVal)
{
	float p,i,d;

	m_fDiffR[0] = m_fDiffR[1];
//	m_fDiffR[1] = fSensorVal - fTragetVal;
	m_fDiffR[1] = fTragetVal - fSensorVal;
	m_fIntegralR += (m_fDiffR[0] + m_fDiffR[1]) / 2.0 * DELTA_T;

	p = R_KP * m_fDiffR[1];							//比例
	i = R_KI * m_fIntegralR;							//積分
	d = R_KD * (m_fDiffR[1] - m_fDiffR[0]) / DELTA_T;	//微分

	return p + i + d;
}


/********************************************************************************
*	各pwmの出力調整
********************************************************************************/
static int PwmBarance(void)
{
	int n13,n24,n14,n23;

	n13 = SetPwmBalance(1,3);
	if(n13 == -1)return -1;
	n24 = SetPwmBalance(2,4);
	if(n24 == -1)return -1;
	n14 = SetPwmBalance(1,4);
	if(n14 == -1)return -1;
	n23 = SetPwmBalance(2,3);
	if(n23 == -1)return -1;


printf("+++ n02=%d n13=%d n03=%d n12=%d\n",n13,n24,n14,n23);
	return 0;
}
/********************************************************************************
*	pwmバランス調整
********************************************************************************/
static int SetPwmBalance(int nPwm1,int nPwm2)
{
	uint16_t VL53L0X_Measurement;		//測定値(mm)
	time_t	start_time;
	int i,save,ret=-1;

	start_time = time(NULL);
	printf("--- SetPwmBalance\n");
	for(i=0;i<200;i++){
		if(difftime(time(NULL), start_time) > DEBUG_MAINLOOP_TO){						//debug 指定秒で終了する
			printf("--- stop. Debug Time out. [%0.0lfs]\n",DEBUG_MAINLOOP_TO);
			ret=-1;
			break;
		}

		if(VL53L0X_GetMeasurements(&VL53L0X_Measurement,0) != VL53L0X_ERROR_NONE)break;	//VL53L0X測定値獲得
//	    printf("VL53L0X:%dmm  pwm=%d\n", VL53L0X_Measurement,i);								//degbug地上高(mm)
		if(i == 0)save=VL53L0X_Measurement;
		if(save+7 < VL53L0X_Measurement){//7mm上昇したら
			ret = i;
			break;
		}

		PCA9685_pwmWrite(nPwm1, (double)(PWM_MIN + PWM_BASE_POWER + i)); 
		PCA9685_pwmWrite(nPwm2, (double)(PWM_MIN + PWM_BASE_POWER + i)); 

	}
	PCA9685_pwmWrite(nPwm1, PWM_MIN);
	PCA9685_pwmWrite(nPwm2, PWM_MIN);

	usleep(2000000);
	return ret;
}

/********************************************************************************
*	北(2-4側)に向かうための追加するモータ出力値を求める
*	nDirection : 0..359
*	機体は、2-4側:0(北) 4-1側:90(東) 1-3側:180(南) 3-2側:270(西)
********************************************************************************/
static void HeadNorth(int nDirection,int *npPower1, int *npPower2, int *npPower3, int *npPower4)
{
//	角度	モータ3	モータ1	モータ4	モータ2
//------------------------------------------------
//	0		50%		50
//------------------------------------------------
//	45		0		100		0
//------------------------------------------------
//	90				50		50
//------------------------------------------------
//	135				0		100		0
//------------------------------------------------
//	180						50		50
//------------------------------------------------
//	225		0				0		100
//------------------------------------------------
//	270		50						50
//------------------------------------------------
//	315		100		0				0
//------------------------------------------------
//	0(360)	50		50

	double dfPowerThrust = 50;			//推進力
	double dfOneDegreesPower;			//1度あたりの推進力

	dfOneDegreesPower = dfPowerThrust / 90;

	if (nDirection < 0 || nDirection > 359)return;

	if (nDirection < 45) {
		*npPower3 = (int)(dfPowerThrust / 2 - dfOneDegreesPower * nDirection);	//50..0
		*npPower1 = (int)(dfOneDegreesPower * nDirection + dfPowerThrust / 2);	//50..100
		return;
	}
	if (nDirection < 90) {
		nDirection -= 45;
		*npPower1 = (int)(dfPowerThrust - dfOneDegreesPower * nDirection);		//100..50
		*npPower4 = (int)(dfOneDegreesPower * nDirection);						//0..50
		return;
	}
	if (nDirection < 135) {
		nDirection -= 90;
		*npPower1 = (int)(dfPowerThrust / 2 - dfOneDegreesPower * nDirection);	//50..0
		*npPower4 = (int)(dfOneDegreesPower * nDirection + dfPowerThrust / 2);	//50..100
		return;
	}
	if (nDirection < 180) {
		nDirection -= 135;
		*npPower4 = (int)(dfPowerThrust - dfOneDegreesPower * nDirection);		//100..50
		*npPower2 = (int)(dfOneDegreesPower * nDirection);						//0..50
		return;
	}
	if (nDirection < 225) {
		nDirection -= 180;
		*npPower4 = (int)(dfPowerThrust / 2 - dfOneDegreesPower * nDirection);	//50..0
		*npPower2 = (int)(dfOneDegreesPower * nDirection + dfPowerThrust / 2);	//50..100
		return;
	}
	if (nDirection < 270) {
		nDirection -= 225;
		*npPower2 = (int)(dfPowerThrust - dfOneDegreesPower * nDirection);		//100..50
		*npPower3 = (int)(dfOneDegreesPower * nDirection);						//0..50
		return;
	}
	if (nDirection < 315) {
		nDirection -= 270;
		*npPower2 = (int)(dfPowerThrust / 2 - dfOneDegreesPower * nDirection);	//50..0
		*npPower3 = (int)(dfOneDegreesPower * nDirection + dfPowerThrust / 2);	//50..100
		return;
	}
	if (nDirection < 360) {
		nDirection -= 315;
		*npPower3 = (int)(dfPowerThrust - dfOneDegreesPower * nDirection);		//100..50
		*npPower1 = (int)(dfOneDegreesPower * nDirection);						//0..50
		return;
	}
}

/********************************************************************************
*	BLHeli(ESC)初期化
********************************************************************************/
static void BLHeli_init(void)
{
	int i;

//	for(i=0;i<4;i++){
//		printf("--- init pwm. [%d]\n",i+1);
//		PCA9685_pwmWrite(i+1, PWM_MIN -20); 
//		usleep(2000000);
//		PCA9685_pwmWrite(i+1, PWM_MIN );
//		usleep(1000000);
//	}


	PCA9685_pwmWrite(0, 1145);
	usleep(2000000);
	PCA9685_pwmWrite(1, 1145);
	usleep(2000000);
	PCA9685_pwmWrite(2, 1145);
	usleep(2000000);
	PCA9685_pwmWrite(3, 1145);
	usleep(2000000);
	PCA9685_pwmWrite(4, 1300);
	usleep(2000000);




}
/********************************************************************************
*	i2cデバイス初期化
********************************************************************************/
static int I2c_device_init(void)
{
	//L53L0Xを複数使用するためxshutをlowにする
  // 	pinMode(VL53L0X_XSHUT_1_GPIO,OUTPUT);
  // 	pinMode(VL53L0X_XSHUT_2_GPIO,OUTPUT);
  // 	pinMode(VL53L0X_XSHUT_3_GPIO,OUTPUT);
  // 	pinMode(VL53L0X_XSHUT_4_GPIO,OUTPUT);
   	pinMode(VL53L0X_XSHUT_5_GPIO,OUTPUT);
  //  digitalWrite(VL53L0X_XSHUT_1_GPIO,LOW);
  //  digitalWrite(VL53L0X_XSHUT_2_GPIO,LOW);
  //  digitalWrite(VL53L0X_XSHUT_3_GPIO,LOW);
  //  digitalWrite(VL53L0X_XSHUT_4_GPIO,LOW);
    digitalWrite(VL53L0X_XSHUT_5_GPIO,LOW);
//	if(VL53L0X_init(VL53L0X_XSHUT_1_GPIO,0x2a,0) != VL53L0X_ERROR_NONE){	//距離センサ 1
//		printf("*** VL53L0X_init()err\n");
//		return -1;
//	}
//	if(VL53L0X_init(VL53L0X_XSHUT_2_GPIO,0x2b,1) != VL53L0X_ERROR_NONE){	//距離センサ 2
//		printf("*** VL53L0X_init()err\n");
//		return -1;
//	}
//	if(VL53L0X_init(VL53L0X_XSHUT_3_GPIO,0x2c,2) != VL53L0X_ERROR_NONE){	//距離センサ 3
//		printf("*** VL53L0X_init()err\n");
//		return -1;
//	}
//	if(VL53L0X_init(VL53L0X_XSHUT_4_GPIO,0x2d,3) != VL53L0X_ERROR_NONE){	//距離センサ 4
//		printf("*** VL53L0X_init()err\n");
//		return -1;
//	}
	if(VL53L0X_init(VL53L0X_XSHUT_5_GPIO,0x2e,4) != VL53L0X_ERROR_NONE){	//距離センサ 5 Altitude
		printf("*** VL53L0X_init()err\n");
		return -1;
	}
	printf("--- VL53L0X_init() OK\n");

	if(MPU6050_init() != 0){						//ジャイロ加速度センサ
		printf("*** MPU6050_init()err\n");
		VL53L0X_close(0);
		return -1;
	}
	printf("--- MPU6050_init() OK\n");

	if(PCA9685_init() != 0){					//PWMドライバ
		printf("*** PCA9685_init()err\n");
		VL53L0X_close(0);
		return -1;
	}
	printf("--- PCA9685_init() OK\n");

	if (HMC5883L_init() == 1) {					//コンパス
		printf("*** HMC5883L_init() error.\n");
		VL53L0X_close(0);
		return -1;
	}
	printf("--- HMC5883L_init() OK\n");

	return 0;
}


/********************************************************************************
*	デバッグ出力用ファイル初期化
********************************************************************************/
static int Debug_Print_init(void)
{
	if((m_fp = fopen("/tmp/test.log", "w")) == NULL) {	//debug用 ジャイロ加速度センサ値表示
		printf("fopen err\n");
		VL53L0X_close(0);
		return -1;
	}

	if((m_fpVL53L0X = fopen("/tmp/VL53L0X.log", "w")) == NULL) {//debug用 距離センサ値表示
		printf("fopen err\n");
		fclose(m_fp);
		VL53L0X_close(0);
		return -1;
	}

	return 0;
}



/********************************************************************************
*	デバッグ用
********************************************************************************/
static void DebugPrint(char *buf,FILE *fp)
{
	fputs(buf,fp);
	fseek(fp,  0L, SEEK_SET);
}

