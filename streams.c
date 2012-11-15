#include <stdio.h>
#include <inttypes.h>

#define kHDSPe_MADI_FX	1
#define kMADIface_XT	2

#define NUM_OUTPUTS_S_HMFX	(64*3+4)
#define NUM_OUTPUTS_D_HMFX	(32*3+4)
#define NUM_OUTPUTS_Q_HMFX	(16*3+4)
#define NUM_INPUTS_S_HMFX	(64*3+2)
#define NUM_INPUTS_D_HMFX	(32*3+2)
#define NUM_INPUTS_Q_HMFX	(16*3+2)

#define NUM_OUTPUTS_S_MFXT	(64*3+6)
#define NUM_OUTPUTS_D_MFXT	(32*3+6)
#define NUM_OUTPUTS_Q_MFXT	(16*3+6)
#define NUM_INPUTS_S_MFXT	(64*3+4)
#define NUM_INPUTS_D_MFXT	(32*3+4)
#define NUM_INPUTS_Q_MFXT	(16*3+4)

#define NUM_INPUT_STREAMS_HMFX	25
#define NUM_OUTPUT_STREAMS_HMFX 26

#define NUM_INPUT_STREAMS_MFXT	26
#define NUM_OUTPUT_STREAMS_MFXT 27

#define BYTES_PER_SAMPLE			4
#define INITIAL_SAMPLE_RATE			44100
#define NUM_SAMPLE_FRAMES			8192
#define SAMPLE_FRAMES_PER_BUFFER	8192 //4096
#define BIT_DEPTH					32
#define MIN_RATE					30000
#define MAX_RATE					200000


int main(void) {

    int card_type = kHDSPe_MADI_FX;
    int i;

    int numInputStreams  = card_type == kHDSPe_MADI_FX ? NUM_INPUT_STREAMS_HMFX  : NUM_INPUT_STREAMS_MFXT;
	int numOutputStreams = card_type == kHDSPe_MADI_FX ? NUM_OUTPUT_STREAMS_HMFX : NUM_OUTPUT_STREAMS_MFXT;
    
    for (i = 0; i < numInputStreams; i++)
    {
		int n;
		int32_t *buf = 0;
		
		if (card_type == kHDSPe_MADI_FX) { 
			n = (i == 0) ? 2 : 8;
			if (i > 0) {
				buf += ((i-1)*8+2) * NUM_SAMPLE_FRAMES;
			}
		} else { // MADIface XT
			n = (i < 2) ? 2 : 8;
			if (i == 1) {
				buf += 2 * NUM_SAMPLE_FRAMES;
			} else if (i > 1) {
				buf += ((i-2)*8+4) * NUM_SAMPLE_FRAMES;
			}
		}
        printf ("Inputstream %i with %i channels at byte offset %i\n", i, n, buf);
    }

      for (i = 0; i < numOutputStreams; i++)
    {
		int n;
		int32_t *buf = 0;
		
		if (card_type == kHDSPe_MADI_FX) {
			n = (i == 0 || i == NUM_OUTPUT_STREAMS_HMFX-1) ? 2 : 8;
			if (i == NUM_OUTPUT_STREAMS_HMFX-1) {
				buf += 2 * NUM_SAMPLE_FRAMES; // HW channel 2..3
			} else if (i > 0) {
				buf += ((i-1)*8+4) * NUM_SAMPLE_FRAMES; // HW channel 4..195
			}
		} else {
			n = (i < 2 || i == NUM_OUTPUT_STREAMS_MFXT-1) ? 2 : 8;
			if (i == NUM_OUTPUT_STREAMS_MFXT-1) {
				buf += 4 * NUM_SAMPLE_FRAMES; // Phones, HW channel 4..5
			} else if (i == 1) {
				buf += 2 * NUM_SAMPLE_FRAMES; // AES, HW channel 2..3
			} else if (i > 1) {
				buf += ((i-2)*8+6) * NUM_SAMPLE_FRAMES; // HW channel 6..197
			} // else Analog, HW channel 0..1			
		}
        printf ("Outputstream %i with %i channels at byte offset %i\n", i, n, buf);
    }

      for (i = 0; i < 194; i++) {
          printf ("Linux input ch %i offset %i\n", i , (i < 2) ? 0 : 65536 + 8 * 4 * 8192 * (((i-2)/8)));
      }
 
      for (i = 0; i < 196; i++) {
          printf ("Linux output ch %i offset %i\n", i ,
                  (i < 2) ? 0 : ((i > 193) ? 65536 :
                    131072 + 8 * 4 * 8192 * ((i-2)/8)));
      }

    return 0;
}
