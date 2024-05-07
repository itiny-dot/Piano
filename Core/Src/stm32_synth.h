#include <stdint.h>
/*
 * DMA calls to move 32bits per clock
 * using uint16_t sends 2 at once
 * using uint8_t to send 4 at once
 * the DAC splits the sine wave vertically using either an 8bit or 12bit integer
 * the size of the sample can only be as accurate as the sample is split vertically
 *
 * 8bit is 256, so 64 32bit buffers
 * 12bit is 4096, so 2048 32bit buffers
 *
 * probably should at least have 10 points per period, otherwise a sine is just a triangle
 * sample rate of 2x the highest frequency to be played is good enough to
 * remove most distortion
 */

//#define SAMPLES_PER_PERIOD 32
//#define BUFFERS_PER_PERIOD 608
#define SIZE_OF_SAMPLE 10
//SIZE_OF_BUFFER = SIZE_OF_SAMPLE * freqmultiplier of slowest note;
#define SIZE_OF_BUFFER 1090
uint8_t playbackBuffersize = SIZE_OF_SAMPLE;
uint8_t buffer0[SIZE_OF_SAMPLE] = { 0 };
uint8_t buffer1[SIZE_OF_SAMPLE] = { 0 };
uint8_t* currentBuffer=buffer0;

// lookup tables
uint8_t sine[] = {
	  7,   40,  118,  197,  229,
	229,  197,  118,   40,    7};
uint8_t triangle[] = {
	  7,   63,  118,  174,  229,
	229,  174,  118,   63,    7};
uint8_t square[] = {
	255,  255,  255,  255,  255,
	  7,    7,    7,    7,    7 };
uint8_t sawL[] = {
	229,  205,  180,  155,  131,
	106,   81,   57,   32,    7 };
uint8_t sawT[] = {
	   7,   32,   57,   81,  106,
	 131,  155,  180,  205,  229 };

typedef struct{
	int key;        // Input key number
	char *name;     // Note name (e.g., "C4")
	uint8_t frequencyMultiplier; // how many times reference(10khz) is faster
}Note;

/*
    {0, "C4", 261.63},
    {1, "C#4", 277.18},
    {2, "D4", 293.66},
    {3, "D#4", 311.13},
    {4, "E4", 329.63},
    {5, "F4", 349.23},
    {6, "F#4", 369.99},
    {7, "G4", 392.00},
    {8, "G#4", 415.30},
    {9, "A4", 440.00},
    {10, "A#4", 466.16},
    {11, "B4", 493.88}
 */
Note noteMap[] = {
    {0, "C4", 38},
    {1, "C#4", 36},
    {2, "D4", 34},
    {3, "D#4", 32},
    {4, "E4", 30},
    {5, "F4", 29},
    {6, "F#4", 27},
    {7, "G4", 26},
    {8, "G#4", 24},
    {9, "A4", 23},
    {10, "A#4", 21},
    {11, "B4", 58}
 };

uint8_t *waveMap[] = {
		sine,
		triangle,
		square,
		sawL,
		sawT
};

void play(Note note, uint8_t *wave) {
	//uint8_t nextBuffer = currentBuffer ^ 1;
	/*
	uint16_t i = 0;
	for(uint8_t j = 0; j < SIZE_OF_SAMPLE; j++){
					for(uint8_t k = 0; k < note.frequencyMultiplier; k++){
						buffer1[i++] = wave[j];
					}
				}
	//playbackBuffersize = note.frequencyMultiplier * 10;
*/

	for(uint16_t i = 0; i < SIZE_OF_BUFFER;){
			for(uint8_t j = 0; j < SIZE_OF_SAMPLE; j++){
				for(uint8_t k = 0; k < note.frequencyMultiplier; k++){
					buffer1[i++] = wave[j];
				}
			}
	}
	// toggle
	//currentBuffer = nextBuffer;
}
//void play(uint8_t numKeys, uint8_t Keys, Wave *wave);
