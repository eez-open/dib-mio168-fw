#pragma once

void DLOG_Start(DlogRecordingStart &dlogRecordingStart);
void DLOG_LoopWrite();
void DLOG_Stop();

extern bool g_fileSystemIsMounted;
extern DlogState dlogState;
