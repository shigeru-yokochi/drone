#include <stdlib.h>
#include <stdio.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include <wiringPi.h>
#include <unistd.h>

#define VERSION_REQUIRED_MAJOR 	1
#define VERSION_REQUIRED_MINOR 	0
#define VERSION_REQUIRED_BUILD 	1
#define MEASUREMENTS_MAX		256

#define DEVICE_MAX              5   //i2cアドレスを変更した場合の最大デバイス数

VL53L0X_Error VL53L0X_init(uint16_t xshut_gpio,uint16_t i2c_address,uint16_t device_id);
void VL53L0X_close(uint16_t device_id);
VL53L0X_Error VL53L0X_GetMeasurements(uint16_t *pVL53L0X_Measurement,uint16_t device_id);


static VL53L0X_Error VL53L0X_WaitMeasurementDataReady(VL53L0X_DEV Dev);
static VL53L0X_Dev_t m_MyDevice[DEVICE_MAX];
static VL53L0X_Dev_t *m_pMyDevice[DEVICE_MAX];
static VL53L0X_RangingMeasurementData_t m_RangingMeasurementData[DEVICE_MAX];
static VL53L0X_RangingMeasurementData_t *m_pRangingMeasurementData[DEVICE_MAX];
static uint16_t* m_pResults[DEVICE_MAX];
static uint32_t m_measurement[DEVICE_MAX];

static void VL53L0X_print_pal_error(VL53L0X_Error Status);
static VL53L0X_Error VL53L0X_WaitStopCompleted(VL53L0X_DEV Dev);
/************************************************************************
*	初期化
************************************************************************/
VL53L0X_Error VL53L0X_init(uint16_t xshut_gpio,uint16_t i2c_address,uint16_t device_id)
{
    VL53L0X_Error           Status = VL53L0X_ERROR_NONE;
    VL53L0X_Version_t		Version;
    VL53L0X_Version_t		*pVersion   = &Version;
    VL53L0X_DeviceInfo_t	DeviceInfo;
    int32_t status_int;


    printf ("VL53L0X PAL Continuous Ranging example\n\n");

	m_pMyDevice[device_id] = &m_MyDevice[device_id];
	m_pRangingMeasurementData[device_id]    = &m_RangingMeasurementData[device_id];
	m_pResults[device_id] = 0;
	m_measurement[device_id] = 0;

    // xshutをlow-highにしてi2cアドレスをdefaultの0x29で初期化する
    digitalWrite(xshut_gpio,LOW);
    digitalWrite(xshut_gpio,HIGH);
    usleep(100000); //100ms
    m_pMyDevice[device_id]->fd = VL53L0X_i2c_init("/dev/i2c-1", 0x29); //choose between i2c-0 and i2c-1; On the raspberry pi zero, i2c-1 are pins 2 and 3 
    m_pMyDevice[device_id]->I2cDevAddr      = i2c_address;
    VL53L0X_SetDeviceAddress(m_pMyDevice[device_id],	m_pMyDevice[device_id]->I2cDevAddr<<1);
    m_pMyDevice[device_id]->fd = VL53L0X_i2c_init("/dev/i2c-1", m_pMyDevice[device_id]->I2cDevAddr); //choose between i2c-0 and i2c-1; On the raspberry pi zero, i2c-1 are pins 2 and 3


//    if (MyDevice.fd<0) {
    if (m_pMyDevice[device_id]->fd<0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        printf ("Failed to init\n");
    }

    /*
     *  Get the version of the VL53L0X API running in the firmware
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        status_int = VL53L0X_GetVersion(pVersion);
        if (status_int != 0)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    /*
     *  Verify the version of the VL53L0X API running in the firmrware
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        if( pVersion->major != VERSION_REQUIRED_MAJOR ||
            pVersion->minor != VERSION_REQUIRED_MINOR ||
            pVersion->build != VERSION_REQUIRED_BUILD )
        {
            printf("VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %d). This example requires %d.%d.%d.\n",
                pVersion->major, pVersion->minor, pVersion->build, pVersion->revision,
                VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
        }
    }

    // End of implementation specific
    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_DataInit\n");
//        Status = VL53L0X_DataInit(&MyDevice); // Data initialization
        Status = VL53L0X_DataInit(m_pMyDevice[device_id]); // Data initialization
        VL53L0X_print_pal_error(Status);
    }
    
    if(Status == VL53L0X_ERROR_NONE)
    {
//        Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
        Status = VL53L0X_GetDeviceInfo(m_pMyDevice[device_id], &DeviceInfo);
    }
    if(Status == VL53L0X_ERROR_NONE)
    {
        printf("VL53L0X_GetDeviceInfo:\n");
        printf("Device Name : %s\n", DeviceInfo.Name);
        printf("Device Type : %s\n", DeviceInfo.Type);
        printf("Device ID : %s\n", DeviceInfo.ProductId);
        printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
        printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

        if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
        	printf("Error expected cut 1.1 but found cut %d.%d\n",
        			DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
        	Status = VL53L0X_ERROR_NOT_SUPPORTED;
        }
    }


    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;


    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_StaticInit\n");
        Status = VL53L0X_StaticInit(m_pMyDevice[device_id]); // Device Initialization
        // StaticInit will set interrupt by default
        VL53L0X_print_pal_error(Status);
    }
    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_PerformRefCalibration\n");
        Status = VL53L0X_PerformRefCalibration(m_pMyDevice[device_id],
        		&VhvSettings, &PhaseCal); // Device Initialization
        VL53L0X_print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(m_pMyDevice[device_id],
        		&refSpadCount, &isApertureSpads); // Device Initialization
        VL53L0X_print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {

        printf ("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(m_pMyDevice[device_id], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
        VL53L0X_print_pal_error(Status);
    }
    
    if(Status == VL53L0X_ERROR_NONE)
    {
		printf ("Call of VL53L0X_StartMeasurement\n");
		Status = VL53L0X_StartMeasurement(m_pMyDevice[device_id]);
		VL53L0X_print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
		m_pResults[device_id] = (uint16_t*)malloc(sizeof(uint16_t) * MEASUREMENTS_MAX);
	}


	return Status;
}
/************************************************************************
*	測定値獲得
************************************************************************/
VL53L0X_Error VL53L0X_GetMeasurements(uint16_t *pVL53L0X_Measurement,uint16_t device_id)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    Status = VL53L0X_WaitMeasurementDataReady(m_pMyDevice[device_id]);
    if(Status != VL53L0X_ERROR_NONE)return Status;;

    Status = VL53L0X_GetRangingMeasurementData(m_pMyDevice[device_id], m_pRangingMeasurementData[device_id]);
    *(m_pResults[device_id] + m_measurement[device_id]) = m_pRangingMeasurementData[device_id]->RangeMilliMeter;
//    printf("In loop m_measurement[device_id] %d: %d\n", m_measurement[device_id], m_pRangingMeasurementData[device_id]->RangeMilliMeter);

    VL53L0X_ClearInterruptMask(m_pMyDevice[device_id], VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

	m_measurement[device_id]++;
	if(m_measurement[device_id] == MEASUREMENTS_MAX){
		m_measurement[device_id] = 0;
	}

	*pVL53L0X_Measurement = m_pRangingMeasurementData[device_id]->RangeMilliMeter;
    if(*pVL53L0X_Measurement > 1000 || *pVL53L0X_Measurement == 0){
        *pVL53L0X_Measurement = 1000;   //計測不能値を変更する
    }

    return Status;
}
/************************************************************************
*	終了処理
************************************************************************/
void VL53L0X_close(uint16_t device_id)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	if(m_pResults[device_id] != 0){
	    free(m_pResults[device_id]);
	}

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_StopMeasurement\n");
        Status = VL53L0X_StopMeasurement(m_pMyDevice[device_id]);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("VL53L0X_Wait Stop to be competed\n");
        Status = VL53L0X_WaitStopCompleted(m_pMyDevice[device_id]);
    }

    if(Status == VL53L0X_ERROR_NONE){
		Status = VL53L0X_ClearInterruptMask(m_pMyDevice[device_id],
			VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	}

    VL53L0X_i2c_close();

}
/************************************************************************
*	
************************************************************************/
static void VL53L0X_print_pal_error(VL53L0X_Error Status)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n", Status, buf);
}
/************************************************************************
*	
************************************************************************/
static VL53L0X_Error VL53L0X_WaitMeasurementDataReady(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}
/************************************************************************
*	
************************************************************************/
static VL53L0X_Error VL53L0X_WaitStopCompleted(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
	
    }

    return Status;
}
