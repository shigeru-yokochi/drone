#include <stdlib.h>
#include <stdio.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include <wiringPi.h>

#define VERSION_REQUIRED_MAJOR 	1
#define VERSION_REQUIRED_MINOR 	0
#define VERSION_REQUIRED_BUILD 	1
#define MEASUREMENTS_MAX		256


VL53L0X_Error VL53L0X_init(void);
void VL53L0X_close(void);
VL53L0X_Error VL53L0X_GetMeasurements(uint16_t *pVL53L0X_Measurement);


static VL53L0X_Error VL53L0X_WaitMeasurementDataReady(VL53L0X_DEV Dev);
static VL53L0X_Dev_t m_MyDevice;
static VL53L0X_Dev_t *m_pMyDevice;
static VL53L0X_RangingMeasurementData_t m_RangingMeasurementData;
static VL53L0X_RangingMeasurementData_t *m_pRangingMeasurementData;
static uint16_t* m_pResults;
static uint32_t m_measurement;

static void VL53L0X_print_pal_error(VL53L0X_Error Status);
static VL53L0X_Error VL53L0X_WaitStopCompleted(VL53L0X_DEV Dev);
/************************************************************************
*	初期化
************************************************************************/
VL53L0X_Error VL53L0X_init(void)
{
    VL53L0X_Error 			Status = VL53L0X_ERROR_NONE;
    VL53L0X_Version_t		Version;
    VL53L0X_Version_t		*pVersion   = &Version;
    VL53L0X_DeviceInfo_t	DeviceInfo;
    int32_t status_int;



    printf ("VL53L0X PAL Continuous Ranging example\n\n");

	m_pMyDevice = &m_MyDevice;
	m_pRangingMeasurementData    = &m_RangingMeasurementData;
	m_pResults = 0;
	m_measurement = 0;

    // Initialize Comms
    pinMode(20,OUTPUT);  
    sleep(1)
 
    digitalWrite(20,LOW);
    sleep(1)
    m_pMyDevice->I2cDevAddr      = 0x3b;
printf("---------- code:%d\n",VL53L0X_SetDeviceAddress(m_pMyDevice,	m_pMyDevice->I2cDevAddr << 1));
 
   sleep(1)
 
    m_pMyDevice->fd = VL53L0X_i2c_init("/dev/i2c-1", m_pMyDevice->I2cDevAddr); //choose between i2c-0 and i2c-1; On the raspberry pi zero, i2c-1 are pins 2 and 3
  sleep(1)
  digitalWrite(20,HIGH);
   sleep(1)
 

//    if (MyDevice.fd<0) {
    if (m_pMyDevice->fd<0) {
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
        Status = VL53L0X_DataInit(m_pMyDevice); // Data initialization
        VL53L0X_print_pal_error(Status);
    }
    
    if(Status == VL53L0X_ERROR_NONE)
    {
//        Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
        Status = VL53L0X_GetDeviceInfo(m_pMyDevice, &DeviceInfo);
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
        Status = VL53L0X_StaticInit(m_pMyDevice); // Device Initialization
        // StaticInit will set interrupt by default
        VL53L0X_print_pal_error(Status);
    }
    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_PerformRefCalibration\n");
        Status = VL53L0X_PerformRefCalibration(m_pMyDevice,
        		&VhvSettings, &PhaseCal); // Device Initialization
        VL53L0X_print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(m_pMyDevice,
        		&refSpadCount, &isApertureSpads); // Device Initialization
        VL53L0X_print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {

        printf ("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(m_pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
        VL53L0X_print_pal_error(Status);
    }
    
    if(Status == VL53L0X_ERROR_NONE)
    {
		printf ("Call of VL53L0X_StartMeasurement\n");
		Status = VL53L0X_StartMeasurement(m_pMyDevice);
		VL53L0X_print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
		m_pResults = (uint16_t*)malloc(sizeof(uint16_t) * MEASUREMENTS_MAX);
	}


	return Status;
}
/************************************************************************
*	測定値獲得
************************************************************************/
VL53L0X_Error VL53L0X_GetMeasurements(uint16_t *pVL53L0X_Measurement)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    Status = VL53L0X_WaitMeasurementDataReady(m_pMyDevice);
    if(Status != VL53L0X_ERROR_NONE)return Status;;

    Status = VL53L0X_GetRangingMeasurementData(m_pMyDevice, m_pRangingMeasurementData);
    *(m_pResults + m_measurement) = m_pRangingMeasurementData->RangeMilliMeter;
//    printf("In loop m_measurement %d: %d\n", m_measurement, m_pRangingMeasurementData->RangeMilliMeter);

    VL53L0X_ClearInterruptMask(m_pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

	m_measurement++;
	if(m_measurement == MEASUREMENTS_MAX){
		m_measurement = 0;
	}

	*pVL53L0X_Measurement = m_pRangingMeasurementData->RangeMilliMeter;

    return Status;
}
/************************************************************************
*	終了処理
************************************************************************/
void VL53L0X_close(void)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	if(m_pResults != 0){
	    free(m_pResults);
	}

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_StopMeasurement\n");
        Status = VL53L0X_StopMeasurement(m_pMyDevice);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("VL53L0X_Wait Stop to be competed\n");
        Status = VL53L0X_WaitStopCompleted(m_pMyDevice);
    }

    if(Status == VL53L0X_ERROR_NONE){
		Status = VL53L0X_ClearInterruptMask(m_pMyDevice,
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

