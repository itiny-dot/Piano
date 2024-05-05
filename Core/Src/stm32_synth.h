#include <stdint.h>

// 10 samples per period, B4 ~494hz is ~2ms, C4 ~261 is ~4ms
// poll every 8ms, max poll time via uart161.7ms
#define SIZE_OF_BUFFER 100
#define SIZE_OF_SAMPLE 10

uint16_t sine[] = {
		 122,  645, 1906, 3166, 3689,
		3689, 3166, 1906,  645,  122
};
uint16_t triangle[] = {
		 122, 1014, 1906, 2797, 3689,
		3689, 2797, 1906, 1014,  122
		};
uint16_t digital[] = {
		4095, 4095, 4095, 4095, 4095,
		 122,  122,  122,  122,  122
		 };
uint16_t sawL[] = {
		3689, 3292, 2896, 2500, 2104,
		1707, 1311,  915,  519,  122
		};
uint16_t sawT[] = {
		 122,  519,  915, 1311, 1707,
		2104, 2500, 2896, 3292, 3689
};
