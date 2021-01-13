#include <math.h>
#include <memory.h>

#include "main.h"
#include "fatfs.h"
#include <bsp_driver_sd.h>
#include <ff_gen_drv.h>
#include <sd_diskio.h>

#include "firmware.h"

#include "din.h"
#include "dout.h"
#include "adc.h"
#include "dac7760.h"
#include "dac7563.h"
#include "pwm.h"

#include "din_dlog.h"
#include "utils.h"

////////////////////////////////////////////////////////////////////////////////

extern "C" SPI_HandleTypeDef hspi4;
extern "C" void SPI4_Init(void);
SPI_HandleTypeDef *hspiMaster = &hspi4; // for MASTER-SLAVE communication

DWORD g_fatTime;

SetParams currentState;

////////////////////////////////////////////////////////////////////////////////
// MASTER - SLAVE Communication

uint32_t input[(sizeof(Request) + 3) / 4 + 1];
uint32_t output[(sizeof(Request) + 3) / 4];
volatile int transferCompleted;

void beginTransfer() {
    transferCompleted = 0;
    HAL_SPI_TransmitReceive_DMA(hspiMaster, (uint8_t *)output, (uint8_t *)input, sizeof(Request));
    RESET_PIN(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == hspiADC) {
		ADC_Measure_Finish(true);
		return;
	}

	SET_PIN(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin);
	transferCompleted = 1;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == hspiADC) {
		ADC_Measure_Finish(false);
		return;
	}

    SET_PIN(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin);
	transferCompleted = 2;
}

int waitTransferCompletion() {
	uint32_t startTick = HAL_GetTick();
	while (!transferCompleted) {
		if (HAL_GetTick() - startTick > 500) {
			HAL_SPI_Abort(hspiMaster);
			SET_PIN(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin);
			return 2;
		}

		DLOG_LoopWrite();
	}
	return transferCompleted;
}

////////////////////////////////////////////////////////////////////////////////

void Command_GetInfo(Request &request, Response &response) {
	response.getInfo.firmwareMajorVersion = FIRMWARE_VERSION_MAJOR;
	response.getInfo.firmwareMinorVersion = FIRMWARE_VERSION_MINOR;
	response.getInfo.idw0 = HAL_GetUIDw0();
	response.getInfo.idw1 = HAL_GetUIDw1();
	response.getInfo.idw2 = HAL_GetUIDw2();
	response.getInfo.afeVersion = ((READ_PIN(AFE_ID1_GPIO_Port, AFE_ID1_Pin) << 1) | READ_PIN(AFE_ID0_GPIO_Port, AFE_ID0_Pin)) + 1;
}

void Command_GetState(Request &request, Response &response) {
	g_fatTime = request.getState.fatTime;

	response.getState.flags = 0;
	if (BSP_PlatformIsDetected() == SD_PRESENT) {
		response.getState.flags |= GET_STATE_COMMAND_FLAG_SD_CARD_PRESENT;
	}

	response.getState.dinStates = Din_readDataInputs();

	response.getState.ainValues[0] = ADC_samples[0];
	response.getState.ainValues[1] = ADC_samples[1];
	response.getState.ainValues[2] = ADC_samples[2];
	response.getState.ainValues[3] = ADC_samples[3];

	response.getState.ainFaultStatus = ADC_faultStatus;

	memcpy(&response.getState.dlogState, &dlogState, sizeof(DlogState));
}

void Command_SetParams(Request &request, Response &response) {
	Din_SetParams(request.setParams);
	Dout_SetParams(request.setParams);
	ADC_SetParams(request.setParams);
	DAC_SetParams(0, request.setParams);
	DAC_SetParams(1, request.setParams);
	DACDual_SetParams(0, request.setParams);
	DACDual_SetParams(1, request.setParams);
	PWM_SetParams(0, request.setParams);
	PWM_SetParams(1, request.setParams);

	memcpy(&currentState, &request.setParams, sizeof(SetParams));

	response.setParams.result = 1; // success
}

void Command_DlogRecordingStart(Request &request, Response &response) {
	DLOG_Start(request.dlogRecordingStart);
}

void Command_DlogRecordingStop(Request &request, Response &response) {
	DLOG_Stop();
}

void Command_DiskDriveInitialize(Request &request, Response &response) {
	if (g_fileSystemIsMounted) {
		response.diskDriveInitialize.result = STA_NOINIT;
	} else {
		response.diskDriveInitialize.result = (uint32_t)SD_Driver.disk_initialize(0);
	}
}

void Command_DiskDriveStatus(Request &request, Response &response) {
	if (g_fileSystemIsMounted) {
		response.diskDriveStatus.result = STA_NOINIT;
	} else {
		response.diskDriveStatus.result = (uint32_t)SD_Driver.disk_status(0);
	}
}

void Command_DiskDriveRead(Request &request, Response &response) {
	if (g_fileSystemIsMounted) {
		response.diskDriveRead.result = RES_ERROR;
	} else {
		response.diskDriveRead.result = (uint32_t)SD_Driver.disk_read(0,
			response.diskDriveRead.buffer, request.diskDriveRead.sector, 1);
	}
}

void Command_DiskDriveWrite(Request &request, Response &response) {
	if (g_fileSystemIsMounted) {
		response.diskDriveWrite.result = RES_ERROR;
	} else {
		response.diskDriveWrite.result = (uint32_t)SD_Driver.disk_write(0,
			request.diskDriveWrite.buffer, request.diskDriveWrite.sector, 1);
	}
}

void Command_DiskDriveIoctl(Request &request, Response &response) {
	if (g_fileSystemIsMounted) {
		response.diskDriveInitialize.result = RES_ERROR;
	} else {
		response.diskDriveIoctl.result = (uint32_t)SD_Driver.disk_ioctl(0,
			request.diskDriveIoctl.cmd, request.diskDriveIoctl.buffer);

		memcpy(response.diskDriveRead.buffer, request.diskDriveIoctl.buffer,
			DISK_DRIVER_IOCTL_BUFFER_MAX_SIZE);
	}
}

////////////////////////////////////////////////////////////////////////////////
// Setup & Loop

extern "C" void setup() {
	Din_Setup();
	Dout_Setup();
	ADC_Setup();
	DAC_Setup(0);
    DAC_Setup(1);
    DACDual_Setup();
    PWM_Setup();
}

extern "C" void loop() {
	beginTransfer();

    auto transferResult = waitTransferCompletion();

    Request &request = *(Request *)input;
    Response &response = *(Response *)output;

    if (transferResult == 1) {
    	response.command = 0x80 | request.command;

    	if (request.command == COMMAND_GET_INFO) {
			Command_GetInfo(request, response);
		} else if (request.command == COMMAND_GET_STATE) {
			Command_GetState(request, response);
		} else if (request.command == COMMAND_SET_PARAMS) {
			Command_SetParams(request, response);
		} else if (request.command == COMMAND_DLOG_RECORDING_START) {
			Command_DlogRecordingStart(request, response);
		} else if (request.command == COMMAND_DLOG_RECORDING_STOP) {
			Command_DlogRecordingStop(request, response);
		} else if (request.command == COMMAND_DISK_DRIVE_INITIALIZE) {
			Command_DiskDriveInitialize(request, response);
		} else if (request.command == COMMAND_DISK_DRIVE_STATUS) {
			Command_DiskDriveStatus(request, response);
		} else if (request.command == COMMAND_DISK_DRIVE_READ) {
			Command_DiskDriveRead(request, response);
		} else if (request.command == COMMAND_DISK_DRIVE_WRITE) {
			Command_DiskDriveWrite(request, response);
		} else if (request.command == COMMAND_DISK_DRIVE_IOCTL) {
			Command_DiskDriveIoctl(request, response);
		} else {
	    	response.command = COMMAND_NONE;
		}
    } else {
    	response.command = COMMAND_NONE;

    	HAL_SPI_DeInit(hspiMaster);
		SPI4_Init();
    }
}
