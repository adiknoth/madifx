/*
 * Copyright (c) 2013-2014 Adrian Knoth (Google Inc.)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#include <stdio.h>
#include <inttypes.h>

#define NUM_AES_PAGES 32768*2/4096
#define NUM_MADI_PAGES 32768*192/4096
#define NUM_DMA_CH_PAGES 32768*8/4096
#define MAX_PAGE_TABLE_SIZE 4096
#define HARDWARE_PAGE_SIZE  4096

uint64_t f(int offset, int foo) {
    return offset;
}

struct blabuffer {
    uint64_t (*getPhysicalSegment)(int offset, int foo);
} g;

void dbgout (int addr, uint64_t value, char *what) {
    printf ("Writing %s to PT %i segment %i\n", what, addr, value);
}

int main(void) {
    int i;
    uint64_t dmaPageTable[MAX_PAGE_TABLE_SIZE] = { 0 };
    struct blabuffer* outBufferDescriptor;
    struct blabuffer* inBufferDescriptor;

    g.getPhysicalSegment = f;

    outBufferDescriptor = &g;
    inBufferDescriptor = &g;

#if 0
    for (int i = 0; i < 25; i++) {
        printf ("offset %i ", (i < 2) ? 0 : 8192 * (((i-2)/8)+1));
        printf ("first %i step %i\n", (i < 2) ? 32 * i : 32 * ((i-2) % 8),
        (i < 2) ? 64 : 256);
    }
#endif
    
		// AES In, stereo
    for (i = 0; i < NUM_AES_PAGES; i++) {
        dmaPageTable[i+MAX_PAGE_TABLE_SIZE/2] = inBufferDescriptor->getPhysicalSegment(i*HARDWARE_PAGE_SIZE, 0);		
        dbgout(i+MAX_PAGE_TABLE_SIZE/2, inBufferDescriptor->getPhysicalSegment(i*HARDWARE_PAGE_SIZE, 0), "AES IN");		

    }
		// MADI In, 192 channels
		for (i = 0; i < NUM_MADI_PAGES; i++) {
			dmaPageTable[i+MAX_PAGE_TABLE_SIZE/2+NUM_DMA_CH_PAGES] = inBufferDescriptor->getPhysicalSegment((i+NUM_AES_PAGES)*HARDWARE_PAGE_SIZE, 0);				
			dbgout(i+MAX_PAGE_TABLE_SIZE/2+NUM_DMA_CH_PAGES, inBufferDescriptor->getPhysicalSegment((i+NUM_AES_PAGES)*HARDWARE_PAGE_SIZE, 0), "MADI IN");				
		}
	
		// AES Out, stereo
		for (i = 0; i < NUM_AES_PAGES; i++) {
			dmaPageTable[i] = outBufferDescriptor->getPhysicalSegment(i*HARDWARE_PAGE_SIZE, 0);		
			dbgout(i, outBufferDescriptor->getPhysicalSegment(i*HARDWARE_PAGE_SIZE, 0), "AES OUT");		
		}
		// Phones Out, stereo
		for (i = 0; i < NUM_AES_PAGES; i++) {
			dmaPageTable[i+1*NUM_DMA_CH_PAGES] = outBufferDescriptor->getPhysicalSegment((i+1*NUM_AES_PAGES)*HARDWARE_PAGE_SIZE, 0);		
			dbgout(i+1*NUM_DMA_CH_PAGES, outBufferDescriptor->getPhysicalSegment((i+1*NUM_AES_PAGES)*HARDWARE_PAGE_SIZE, 0), "Phones OUT");
		}
		// MADI Out, 192 channels
		for (i = 0; i < NUM_MADI_PAGES; i++) {
			dmaPageTable[i+2*NUM_DMA_CH_PAGES] = outBufferDescriptor->getPhysicalSegment((i+2*NUM_AES_PAGES)*HARDWARE_PAGE_SIZE, 0);				
			dbgout(i+2*NUM_DMA_CH_PAGES, outBufferDescriptor->getPhysicalSegment((i+2*NUM_AES_PAGES)*HARDWARE_PAGE_SIZE, 0), "MADI OUT");
		}

        printf ("Full pagetable:\n");
        for (i = 0 ; i < MAX_PAGE_TABLE_SIZE; i++) {
            printf ("PT %i %i\n", i, dmaPageTable[i]);
        }
		
    return 0;
}

