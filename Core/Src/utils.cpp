#include "main.h"

#include "utils.h"

#define NOP_1() __ASM volatile ("NOP")
#define NOP_2() NOP_1(); NOP_1()
#define NOP_4() NOP_2(); NOP_2()
#define NOP_10() NOP_4(); NOP_4(); NOP_2()
#define NOP_20() NOP_10(); NOP_10()
#define NOP_40() NOP_20(); NOP_20()
#define NOP_80() NOP_40(); NOP_40()
#define NOP_160() NOP_80(); NOP_80()
#define NOP_174() NOP_160(); NOP_10(); NOP_4()

void delayMicroseconds(uint32_t microseconds) {
	while (microseconds--) {
		// 174 NOP's = 180 NOP's (180 MHz) - 6 NOP's to compensate looping costs
		NOP_174();
	}
}
