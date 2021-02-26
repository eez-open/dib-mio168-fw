#pragma once

extern uint8_t *DLOG_buffer;
extern uint32_t DLOG_bufferIndex;
extern uint32_t DLOG_bufferLastTransferredIndex;
extern uint32_t DLOG_recordSize;
extern uint32_t DLOG_recordIndex;
extern uint32_t DLOG_bufferSize;
extern bool ADC_DLOG_started;
extern bool DIN_DLOG_started;

void DLOG_Data(Response &response);
