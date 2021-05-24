#include <memory.h>

#include "main.h"
#include "firmware.h"

uint8_t *DLOG_buffer = g_buffer;
uint32_t DLOG_bufferIndex = 0;
uint32_t DLOG_bufferLastTransferredIndex = 0;
uint32_t DLOG_recordSize = 0;
uint32_t DLOG_recordIndex = 0;
uint32_t DLOG_bufferSize = 0;
bool ADC_DLOG_started = false;
bool DIN_DLOG_started = false;

void DLOG_Data(Response &response) {
	response.dlogRecordingData.recordIndex = DLOG_recordIndex;

	auto n = DLOG_bufferIndex - DLOG_bufferLastTransferredIndex;
	if (n > DLOG_bufferSize) {
		response.dlogRecordingData.numRecords = 0xFFFF; // buffer overflow
		return;
	}
	if (n > sizeof(response.dlogRecordingData.buffer)) {
		n = sizeof(response.dlogRecordingData.buffer);
	}

	if (n > DLOG_bufferSize) {
		response.dlogRecordingData.numRecords = 0xFFFF; // buffer overflow
		return;
	}

	n /= DLOG_recordSize;
	response.dlogRecordingData.numRecords = n;
	n *= DLOG_recordSize;

	if (n > 0) {
		auto i = DLOG_bufferLastTransferredIndex % DLOG_bufferSize;
		auto j = (DLOG_bufferLastTransferredIndex + n) % DLOG_bufferSize;
		if (i < j) {
			memcpy(response.dlogRecordingData.buffer, DLOG_buffer + i, n);
		} else {
			memcpy(response.dlogRecordingData.buffer, DLOG_buffer + i, DLOG_bufferSize - i);
			memcpy(response.dlogRecordingData.buffer + DLOG_bufferSize - i, DLOG_buffer, j);
		}
	}

	DLOG_bufferLastTransferredIndex += n;

	DLOG_recordIndex += response.dlogRecordingData.numRecords;
}
