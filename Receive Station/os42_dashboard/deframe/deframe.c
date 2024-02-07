/*---------------------------------------------------------------------------*\

  FILE........: drs232_ldpc.c
  AUTHOR......: David Rowe
  DATE CREATED: Sep 2016

  Looks for a unique word in series of soft decision symbols.  When
  found, deframes a RS232 encoded frame of soft decision bit, LDPC
  decodes, and outputs a frame of packed bytes.  Used for high bit
  rate Horus SSTV reception.

  Frame format:

    16 bytes 0x55 - 0xabcdef01 UW - 256 bytes of payload - 2 bytes CRC - 65 bytes LPDC Parity

  Each byte is encoded as a 10 bit RS232 serial word: 
  
    0 LSB .... MSB 1

  Building:
   
    $ gcc drs232_ldpc.c mpdecode_core.c -o drs232_ldpc -Wall -lm

\*---------------------------------------------------------------------------*/


/*
  Copyright (C) 2016 David Rowe

  All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License version 2.1, as
  published by the Free Software Foundation.  This program is
  distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program; if not, see <http://www.gnu.org/licenses/>.
*/

/*
 * 
 * This file was modified by Sasha VE3SVF to support OS-42 imagery reception. This is a general packet deframer. 
 */

#include <assert.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* states -----------------------------------------------*/

#define LOOK_FOR_UW    0
#define COLLECT_PACKET 1

/* packet parameters */

#define UW_BYTES               4
#define UW_BITS                32
#define UW_ALLOWED_ERRORS      5
#define BYTES_PER_PACKET       62
#define BITS_PER_BYTE          8
#define SYMBOLS_PER_PACKET BYTES_PER_PACKET*BITS_PER_BYTE

/* UW pattern we look for, including start/stop bits */

uint8_t uw[] = {0,0,0,1,1,0,1,0,1,1,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,0,1};

int main(int argc, char *argv[]) {
    FILE       *fin, *fout;
    int         state, next_state, i, j, k, ind, score, verbose;
    float       symbol;
    uint8_t     bit, bit_buffer[UW_BITS];
    double      symbol_buf[SYMBOLS_PER_PACKET];
    uint8_t     packet[BYTES_PER_PACKET];
    uint8_t     abyte;
    uint16_t    packet_errors, packets;
    int         iter;

    assert(sizeof(uw) == UW_BITS);


    /* process command line ----------------------------------------------*/

    if (argc < 3) {
  fprintf(stderr, "Bad usage, less than two parameters. Bad boy.\n");
  exit(1);
    }

    if (strcmp(argv[1], "-")  == 0) fin = stdin;
    else if ( (fin = fopen(argv[1],"rb")) == NULL ) {
  fprintf(stderr, "Error opening input file: %s: %s.\n",
         argv[1], strerror(errno));
  exit(1);
    }

    if (strcmp(argv[2], "-") == 0) fout = stdout;
    else if ( (fout = fopen(argv[2],"wb")) == NULL ) {
  fprintf(stderr, "Error opening output file: %s: %s.\n",
         argv[2], strerror(errno));
  exit(1);
    }

    verbose = 0;
    if (argc > 3) {
        if (strcmp(argv[3], "-v") == 0) {
            verbose = 1;
        }
        if (strcmp(argv[3], "-vv") == 0) {
            verbose = 2;
        }
    }

    state = LOOK_FOR_UW;
    memset(bit_buffer,0,  sizeof(bit_buffer));

    packet_errors = packets = 0;

    while(fread(&symbol, BITS_PER_BYTE, 1, fin) == 1) {

        /* make hard decision for purpose of UW detection */

        bit = symbol;
        next_state = state;
        if (state == LOOK_FOR_UW) {

            /* put latest input bit into sliding buffer */
            
            for(i=0; i<UW_BITS-1; i++) {
                bit_buffer[i] = bit_buffer[i+1];
            }
            bit_buffer[i] = bit;

            /* lets see if it matches the UW */

            score = 0;
            for(i=0; i<UW_BITS; i++) {
                score += (bit_buffer[i] == uw[i]);
                /* if (i == BITS_PER_BYTE)
                    printf(" ");
                    printf("%1d", unpacked_packet[i]); */
            }
            //printf("\n");
            
            //fprintf(stderr,"UW score: %d\n", score);
            if (score >= (UW_BITS-UW_ALLOWED_ERRORS)) {
                fprintf(stderr,"UW found! score: %d\n verbose: %d\n", score, verbose);
                ind = 0;
                next_state = COLLECT_PACKET;
            }             
        }

        state = next_state;      
    }

    fclose(fin);
    fclose(fout);

    return 0;
}
