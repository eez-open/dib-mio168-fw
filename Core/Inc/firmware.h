#pragma once

static const size_t MAX_PATH_LENGTH = 255;
static const size_t CHANNEL_LABEL_MAX_LENGTH = 5;

enum MeasureMode {
	MEASURE_MODE_CURRENT,
	MEASURE_MODE_VOLTAGE
};

enum SourceMode {
	SOURCE_MODE_CURRENT,
	SOURCE_MODE_VOLTAGE
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

struct SetParams {
	uint8_t dinRanges;
	uint8_t dinSpeeds;

	uint8_t doutStates;

	struct {
		uint8_t mode; // enum SourceMode
		uint8_t range;
        float nplc; // from 0 to 25
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
            DlogState dlogState;
        } getState;

        struct {
            uint8_t result; // 1 - success, 0 - failure
        } setParams;

        struct {
            float conversionFactors[4];
        } dlogRecordingStart;

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

////////////////////////////////////////////////////////////////////////////////

extern SetParams currentState;

////////////////////////////////////////////////////////////////////////////////

static const uint32_t BUFFER_SIZE = ((48 + 24) * 1024);
extern uint8_t g_buffer[];

