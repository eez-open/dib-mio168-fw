#pragma once

#include <stdlib.h>
#include <stdint.h>

static const size_t MAX_PATH_LENGTH = 255;
static const size_t CHANNEL_LABEL_MAX_LENGTH = 5;

extern uint8_t g_afeVersion;

enum MeasureMode {
	MEASURE_MODE_CURRENT,
	MEASURE_MODE_VOLTAGE
};

enum SourceMode {
	SOURCE_MODE_CURRENT,
	SOURCE_MODE_VOLTAGE
};

enum Waveform {
	WAVEFORM_NONE,
	WAVEFORM_DC,
	WAVEFORM_SINE,
	WAVEFORM_SINE_HALF,
	WAVEFORM_SINE_RECTIFIED,
	WAVEFORM_TRIANGLE,
	WAVEFORM_SQUARE,
	WAVEFORM_PULSE,
	WAVEFORM_SAWTOOTH,
	WAVEFORM_ARBITRARY
};
////////////////////////////////////////////////////////////////////////////////

enum Command {
	COMMAND_NONE = 0x69365f39,

    COMMAND_GET_INFO = 0x0200ccb7,
    COMMAND_GET_STATE = 0x0093f106,
    COMMAND_SET_PARAMS = 0x1ff1c78c,

    COMMAND_DLOG_RECORDING_START = 0x7d3f9f28,
    COMMAND_DLOG_RECORDING_STOP = 0x7de480a1,
    COMMAND_DLOG_RECORDING_DATA = 0x1ddc8028,

    COMMAND_DISK_DRIVE_INITIALIZE = 0x0f8066f8,
    COMMAND_DISK_DRIVE_STATUS = 0x457f0700,
    COMMAND_DISK_DRIVE_READ = 0x4478491d,
    COMMAND_DISK_DRIVE_WRITE = 0x7d652b00,
    COMMAND_DISK_DRIVE_IOCTL = 0x22cc23c8
};

#define GET_STATE_COMMAND_FLAG_SD_CARD_PRESENT (1 << 0)

#define DLOG_STATE_IDLE 0
#define DLOG_STATE_EXECUTING 1
#define DLOG_STATE_FINISH_RESULT_OK 2
#define DLOG_STATE_FINISH_RESULT_BUFFER_OVERFLOW 3
#define DLOG_STATE_FINISH_RESULT_MASS_STORAGE_ERROR 4

struct DlogState {
    uint8_t state; // DLOG_STATE_...
    uint32_t fileLength;
    uint32_t numSamples;
};

struct WaveformParameters {
	Waveform waveform;
	float frequency;
	float phaseShift;
	float amplitude;
	float offset;
	float dutyCycle;
};

struct SetParams {
	uint8_t dinRanges;
	uint8_t dinSpeeds;

	uint8_t doutStates;
	WaveformParameters doutWaveformParameters[8];

	struct {
		uint8_t mode; // enum SourceMode
		uint8_t range;
        float nplc; // from 0 to 25
        float p1CalX;
        float p1CalY;
        float p2CalX;
        float p2CalY;
    } ain[4];

    uint8_t powerLineFrequency; // 50 or 60

	struct {
		uint8_t outputEnabled;
		uint8_t outputRange;
		float outputValue;
	} aout_dac7760[2];

	struct {
		float voltage;
	} aout_dac7563[2];

	WaveformParameters aoutWaveformParameters[4];

	struct {
		float freq;
		float duty;
	} pwm[2];
};

struct DlogRecordingStart {
    float period;
    float duration;
    uint32_t resources;
    char filePath[MAX_PATH_LENGTH + 1];
    char dinLabels[8 * (CHANNEL_LABEL_MAX_LENGTH + 1)];
    char doutLabels[8 * (CHANNEL_LABEL_MAX_LENGTH + 1)];
    char ainLabels[4 * (CHANNEL_LABEL_MAX_LENGTH + 1)];
};

#define DISK_DRIVER_IOCTL_BUFFER_MAX_SIZE 4

struct Request {
    uint32_t command;

    union {
        struct {
            uint32_t fatTime;
        } getState;

		SetParams setParams;

        DlogRecordingStart dlogRecordingStart;

        struct {
            uint32_t sector;
        } diskDriveRead;

        struct {
            uint32_t sector;
            uint16_t reserved;
            uint8_t buffer[1024];
        } diskDriveWrite;

        struct {
            uint8_t cmd;
            uint8_t buffer[DISK_DRIVER_IOCTL_BUFFER_MAX_SIZE];
        } diskDriveIoctl;
    };
};

struct Response {
	uint32_t command;

    union {
        struct {
            uint8_t firmwareMajorVersion;
            uint8_t firmwareMinorVersion;
            uint32_t idw0;
            uint32_t idw1;
            uint32_t idw2;
            uint8_t afeVersion;
        } getInfo;

        struct {
            uint8_t flags; // GET_STATE_COMMAND_FLAG_...
            uint8_t dinStates;
            float ainValues[4];
            uint16_t ainFaultStatus;
            uint8_t ainDiagStatus;
            float activePower;
            float reactivePower;
            float voltRMS;
            float currRMS;
            DlogState dlogState;
        } getState;

        struct {
            uint8_t result; // 1 - success, 0 - failure
        } setParams;

        struct {
            uint32_t recordIndex;
            uint16_t numRecords;
            uint8_t buffer[1024];
        } dlogRecordingData;

        struct {
            uint8_t result;
        } diskDriveInitialize;

        struct {
            uint8_t result;
        } diskDriveStatus;

        struct {
            uint8_t result;
            uint8_t buffer[512];
        } diskDriveRead;

        struct {
            uint8_t result;
        } diskDriveWrite;

        struct {
            uint8_t result;
            uint8_t buffer[DISK_DRIVER_IOCTL_BUFFER_MAX_SIZE];
        } diskDriveIoctl;
	};
};

inline double getAinConversionFactor(uint8_t afeVersion, uint8_t channelIndex, uint8_t mode, uint8_t range) {
    if (afeVersion == 1) {
        if (channelIndex < 2) {
            if (mode == MEASURE_MODE_VOLTAGE) {
                if (range == 0) {
                    return 2.4; // +/- 2.4 V
                }
                if (range == 1) {
                    return 48.0; // +/- 48 V
                }
                return 240.0; // +/- 240 V
            }
            if (mode == MEASURE_MODE_CURRENT) {
                return 2.4 / 33 / 1; // +/- 48 mV (22 ohm, PGA is 1)
            }
        } else {
            if (mode == MEASURE_MODE_VOLTAGE) {
                if (range == 0) {
                    return 2.4; // +/- 2.4 V
                }
                return 12.0; // +/- 12 V
            }
            if (mode == MEASURE_MODE_CURRENT) {
                if (range == 0) {
                    return 2.4 / 22 / 1; // +/- 48 mA (22 ohm, PGA is 1)
                }
                if (range == 1) {
                    return 2.4 / 0.33 / 4; // +/- 1.2 A (0.33 ohm, PGA is 4)
                }
                return 2.4 / 0.01 / 12; // +/- 10 A (0.01 ohm, PGA is 12)
            }
        }
    } else if (afeVersion == 3) {
        if (channelIndex == 0) {
            if (range == 0) {
                return 50.0 * 2.4 / 1.8; // +/- 50 V (50 V is 1.8 V)
            }
            return 450.0 * 2.4 / 1.86; // +/- 450 V (450 V is 1.86 V)
        } else if (channelIndex == 1) {
            if (range == 0) {
                return 2.4 / 1.886; // +/- 1 A (1A is 1.886 V, PGA is 1)
            }
            return 10.0 * 2.4 / 0.82 / 2; // +/- 10 A (10 A is 0.82 V, PGA is 2)
        } else if (channelIndex == 2) {
            if (mode == MEASURE_MODE_VOLTAGE) {
                if (range == 0) {
                    return 2.4; // +/- 2.4 V
                }
                if (range == 1) {
                    return 48.0; // +/- 48.0 V
                }
                return 240.0; // +/- 240 V
            }

            return 2.4 / 33 / 1; // +/- 48 mA (33 ohm, PGA is 1)
        } else {
            if (mode == MEASURE_MODE_VOLTAGE) {
                if (range == 0) {
                    return 2.4; // +/- 2.4 V
                }
                return 12.0; // +/- 12 V
            }
            if (mode == MEASURE_MODE_CURRENT) {
                if (range == 0) {
                    return 2.4 / 22 / 1; // +/- 48 mA (22 ohm, PGA is 1)
                }
                if (range == 1) {
                    return 2.4 / 0.33 / 4; // +/- 1.2 A (0.33 ohm, PGA is 4)
                }
                return 2.4 / 0.01 / 12; // +/- 10 A (0.01 ohm, PGA is 12)
            }
        }
    }
    return 0;
}

////////////////////////////////////////////////////////////////////////////////

extern SetParams currentState;

////////////////////////////////////////////////////////////////////////////////

static const uint32_t BUFFER_SIZE = ((48 + 24) * 1024);
extern uint8_t g_buffer[];

