#include <memory.h>

#include "main.h"
#include "fatfs.h"

#include "firmware.h"
#include "dlog_file.h"
#include "utils.h"

using namespace eez;

extern "C" TIM_HandleTypeDef htim6; // for DIN's data logging

DlogState dlogState;

uint8_t g_dinResources;
uint8_t g_doutResources;

uint8_t *g_writerBuffer = g_buffer;
static const uint32_t WRITER_BUFFER_SIZE = 48 * 1024;
dlog_file::Writer g_writer(g_writerBuffer, WRITER_BUFFER_SIZE);

uint8_t *g_fileWriteBuffer = g_buffer + WRITER_BUFFER_SIZE;
static const uint32_t FILE_WRITE_BUFFER_SIZE = 24 * 1024;
uint32_t g_fileWriteBufferIndex;

uint32_t g_numSamples;
uint32_t g_maxNumSamples;
uint32_t g_lastSavedBufferIndex;

bool g_fileSystemIsMounted = false;
FIL g_file;

uint8_t readDataInputs();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
		if (g_numSamples < g_maxNumSamples) {
			// this is valid sample
			g_writer.writeBit(1);

			if (g_dinResources & 0b00000001) g_writer.writeBit(READ_PIN(DIN0_GPIO_Port, DIN0_Pin) ? 1 : 0);
			if (g_dinResources & 0b00000010) g_writer.writeBit(READ_PIN(DIN1_GPIO_Port, DIN1_Pin) ? 1 : 0);
			if (g_dinResources & 0b00000100) g_writer.writeBit(READ_PIN(DIN2_GPIO_Port, DIN2_Pin) ? 1 : 0);
			if (g_dinResources & 0b00001000) g_writer.writeBit(READ_PIN(DIN3_GPIO_Port, DIN3_Pin) ? 1 : 0);
			if (g_dinResources & 0b00010000) g_writer.writeBit(READ_PIN(DIN4_GPIO_Port, DIN4_Pin) ? 1 : 0);
			if (g_dinResources & 0b00100000) g_writer.writeBit(READ_PIN(DIN5_GPIO_Port, DIN5_Pin) ? 1 : 0);
			if (g_dinResources & 0b01000000) g_writer.writeBit(READ_PIN(DIN6_GPIO_Port, DIN6_Pin) ? 1 : 0);
			if (g_dinResources & 0b10000000) g_writer.writeBit(READ_PIN(DIN7_GPIO_Port, DIN7_Pin) ? 1 : 0);

			if (g_doutResources & 0b00000001) g_writer.writeBit(currentState.doutStates & 0b00000001 ? 1 : 0);
			if (g_doutResources & 0b00000010) g_writer.writeBit(currentState.doutStates & 0b00000010 ? 1 : 0);
			if (g_doutResources & 0b00000100) g_writer.writeBit(currentState.doutStates & 0b00000100 ? 1 : 0);
			if (g_doutResources & 0b00001000) g_writer.writeBit(currentState.doutStates & 0b00001000 ? 1 : 0);
			if (g_doutResources & 0b00010000) g_writer.writeBit(currentState.doutStates & 0b00010000 ? 1 : 0);
			if (g_doutResources & 0b00100000) g_writer.writeBit(currentState.doutStates & 0b00100000 ? 1 : 0);
			if (g_doutResources & 0b01000000) g_writer.writeBit(currentState.doutStates & 0b01000000 ? 1 : 0);
			if (g_doutResources & 0b10000000) g_writer.writeBit(currentState.doutStates & 0b10000000 ? 1 : 0);

			g_writer.flushBits();

			g_numSamples++;
		}
	}
}

void DLOG_FillParameters(DlogRecordingStart &dlogRecordingStart, dlog_file::Parameters &dlogFileParameters) {
	memset(&dlogFileParameters, 0, sizeof(dlogFileParameters));

    dlogFileParameters.xAxis.unit = UNIT_SECOND;
    dlogFileParameters.xAxis.step = dlogRecordingStart.period;
    dlogFileParameters.xAxis.range.min = 0;
    dlogFileParameters.xAxis.range.max = dlogRecordingStart.duration;

    dlogFileParameters.yAxisScale = dlog_file::SCALE_LINEAR;

    g_dinResources = dlogRecordingStart.resources;
    for (int i = 0; i < 8; i++) {
    	if (g_dinResources & (1 << i)) {
    		auto &yAxis = dlogFileParameters.yAxes[dlogFileParameters.numYAxes++];

            yAxis.unit = UNIT_BIT;
            yAxis.range.min = 0;
            yAxis.range.max = 1;
            yAxis.channelIndex = i;

            strcpy(yAxis.label, dlogRecordingStart.dinLabels + i * (CHANNEL_LABEL_MAX_LENGTH + 1));
    	}
    }

    g_doutResources = dlogRecordingStart.resources >> 8;
    for (int i = 0; i < 8; i++) {
    	if (g_doutResources & (1 << i)) {
    		auto &yAxis = dlogFileParameters.yAxes[dlogFileParameters.numYAxes++];

            yAxis.unit = UNIT_BIT;
            yAxis.range.min = 0;
            yAxis.range.max = 1;
            yAxis.channelIndex = 8 + i;

            strcpy(yAxis.label, dlogRecordingStart.doutLabels + i * (CHANNEL_LABEL_MAX_LENGTH + 1));
    	}
    }

    dlogFileParameters.period = dlogRecordingStart.period;
	dlogFileParameters.duration = dlogRecordingStart.duration;
}

FRESULT DLOG_CreateRecordingsDir() {
	auto result = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
	if (result != FR_OK) {
		return result;
	}
	g_fileSystemIsMounted = true;

    FILINFO fno;
    result = f_stat("/Recordings", &fno);
	if (result != FR_OK) {
		result = f_mkdir("/Recordings");
	}
	return result;
}

FRESULT DLOG_WriteHeader(DlogRecordingStart &dlogRecordingStart) {
	auto result = DLOG_CreateRecordingsDir();
	if (result != FR_OK) {
		return result;
	}

	result = f_open(&g_file, dlogRecordingStart.filePath, FA_WRITE | FA_CREATE_ALWAYS);
	if (result != FR_OK) {
		return result;
	}

	UINT bw;
	result = f_write(&g_file, g_writer.getBuffer(), g_writer.getDataOffset(), &bw);
	if (result != FR_OK) {
		goto Exit;
	}

	if (bw < g_writer.getDataOffset()) {
		// disk is full
		result = FR_DENIED;
		goto Exit;
	}

Exit:
	return result;
}

FRESULT DLOG_StartFile(DlogRecordingStart &dlogRecordingStart) {
	dlog_file::Parameters dlogFileParameters;
	DLOG_FillParameters(dlogRecordingStart, dlogFileParameters);

	g_writer.reset();
	g_writer.writeFileHeaderAndMetaFields(dlogFileParameters);
	return DLOG_WriteHeader(dlogRecordingStart);
}

uint32_t DLOG_WriteFile(bool flush = false) {
	__disable_irq();
	{
		uint32_t n = FILE_WRITE_BUFFER_SIZE - g_fileWriteBufferIndex;

		uint32_t writerBufferIndex = g_writer.getBufferIndex();
		uint32_t diff = writerBufferIndex - g_lastSavedBufferIndex;

		if (diff < n) {
			n = diff;
		}

		if (diff > WRITER_BUFFER_SIZE) {
			// overflow detected

			uint32_t nInvalid = diff - WRITER_BUFFER_SIZE;
			if (nInvalid >= n) {
				nInvalid = n;
			}

			// invalid samples
			memset(g_fileWriteBuffer + g_fileWriteBufferIndex, 0, nInvalid);
			g_fileWriteBufferIndex += nInvalid;
			g_lastSavedBufferIndex += nInvalid;

			n -= nInvalid;
		}

		if (n > 0) {
			uint32_t from = g_lastSavedBufferIndex % WRITER_BUFFER_SIZE;
			memcpy(g_fileWriteBuffer + g_fileWriteBufferIndex, g_writerBuffer + from, n);
			g_fileWriteBufferIndex += n;
			g_lastSavedBufferIndex += n;
		}
	}
	__enable_irq();

	if (g_fileWriteBufferIndex == FILE_WRITE_BUFFER_SIZE || flush) {
		UINT btw = g_fileWriteBufferIndex;
		UINT bw;
		FRESULT result = f_write(&g_file, g_fileWriteBuffer, btw, &bw);

		if (result != FR_OK) {
			goto Exit;
		}
		if (bw < btw) {
			// disk is full
			result = FR_DENIED;
			goto Exit;
		}

		g_fileWriteBufferIndex = 0;

Exit:
		return result != FR_OK ? DLOG_STATE_FINISH_RESULT_MASS_STORAGE_ERROR : DLOG_STATE_FINISH_RESULT_OK;
	}

	return DLOG_STATE_FINISH_RESULT_OK;
}

void DLOG_CloseFile() {
	f_close(&g_file);
	// unmount
	f_mount(NULL, 0, 0);
	g_fileSystemIsMounted = false;
}

void DLOG_Start(DlogRecordingStart &dlogRecordingStart) {
	auto result = DLOG_StartFile(dlogRecordingStart);
	if (result != FR_OK) {
		DLOG_CloseFile();
		dlogState.state = DLOG_STATE_FINISH_RESULT_MASS_STORAGE_ERROR;
	} else {
		TIM6->ARR = (uint16_t)(dlogRecordingStart.period * 10000000) - 1; // convert to microseconds

		g_numSamples = 0;
		g_maxNumSamples = (uint32_t)(dlogRecordingStart.duration / dlogRecordingStart.period) + 1;

		g_lastSavedBufferIndex = g_writer.getDataOffset();
		g_fileWriteBufferIndex = 0;

		dlogState.fileLength = g_writer.getFileLength();
		dlogState.numSamples = g_numSamples;
		dlogState.state = DLOG_STATE_EXECUTING;

		HAL_TIM_Base_Start_IT(&htim6);
	}
}

void DLOG_LoopWrite() {
	if (g_fileSystemIsMounted) {
		auto result = DLOG_WriteFile(false);

		dlogState.fileLength = g_writer.getFileLength();
		dlogState.numSamples = g_numSamples;

		if (result != DLOG_STATE_FINISH_RESULT_OK) {
			dlogState.state = result;
		} else if (g_numSamples >= g_maxNumSamples) {
			dlogState.state = DLOG_STATE_FINISH_RESULT_OK;
		}
	}
}

void DLOG_Stop() {
	HAL_TIM_Base_Stop_IT(&htim6);

	// flush buffer
	uint32_t result = DLOG_STATE_FINISH_RESULT_OK;
	do {
		result = DLOG_WriteFile(true);
		if (result != DLOG_STATE_FINISH_RESULT_OK) {
			break;
		}
	} while (g_lastSavedBufferIndex < g_writer.getBufferIndex());

	dlogState.fileLength = g_writer.getFileLength();
	dlogState.numSamples = g_numSamples;
	dlogState.state = result;

	DLOG_CloseFile();
}
