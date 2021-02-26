#pragma once

void DIN_DLOG_Start(DlogRecordingStart &dlogRecordingStart);
void DIN_DLOG_LoopWrite();
void DIN_DLOG_Stop();

extern bool g_fileSystemIsMounted;
extern DlogState dlogState;
