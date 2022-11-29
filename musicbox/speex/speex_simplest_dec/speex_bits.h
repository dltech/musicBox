/* Copyright (C) 2002 Jean-Marc Valin */
/**
   @file speex_bits.h
   @brief Handles bit packing/unpacking
*/
/*
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   - Neither the name of the Xiph.org Foundation nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef BITS_H
#define BITS_H
/** @defgroup SpeexBits SpeexBits: Bit-stream manipulations
 *  This is the structure that holds the bit-stream when encoding or decoding
 * with Speex. It allows some manipulations as well.
 *  @{
 */

/** Bit-packing data structure representing (part of) a bit-stream. */
typedef struct SpeexBits {
   char *chars;   /**< "raw" data */
   int   nbBits;  /**< Total number of bits stored in the stream*/
   int   charPtr; /**< Position of the byte "cursor" */
   int   bitPtr;  /**< Position of the bit "cursor" within the current char */
   int   owner;   /**< Does the struct "own" the "raw" buffer (member "chars") */
   int   overflow;/**< Set to one if we try to read past the valid data */
   int   buf_size;/**< Allocated size for buffer */
   int   reserved1; /**< Reserved for future use */
   void *reserved2; /**< Reserved for future use */
} SpeexBits;

/** Initializes SpeexBits struct using a pre-allocated buffer*/
void speex_bits_init_buffer(SpeexBits *bits, void *buff, int buf_size);

/** Sets the bits in a SpeexBits struct to use data from an existing buffer (for decoding without copying data) */
void speex_bits_set_bit_buffer(SpeexBits *bits, void *buff, int buf_size);

/** Resets bits to initial value (just after initialization, erasing content)*/
void speex_bits_reset(SpeexBits *bits);

/** Initializes the bit-stream from the data in an area of memory */
void speex_bits_read_from(SpeexBits *bits, const char *bytes, int len);

/** Append bytes to the bit-stream
 *
 * @param bits Bit-stream to operate on
 * @param bytes pointer to the bytes what will be appended
 * @param len Number of bytes of append
 */
void speex_bits_read_whole_bytes(SpeexBits *bits, const char *bytes, int len);

/** Interpret the next bits in the bit-stream as an unsigned integer
 *
 * @param bits Bit-stream to operate on
 * @param nbBits Number of bits to interpret
 * @return An unsigned integer represented by the bits read
 */
unsigned int speex_bits_unpack_unsigned(SpeexBits *bits, int nbBits);

/** Advances the position of the "bit cursor" in the stream
 *
 * @param bits Bit-stream to operate on
 * @param n Number of bits to advance
 */
void speex_bits_advance(SpeexBits *bits, int n);

/** Returns the number of bits remaining to be read in a stream
 *
 * @param bits Bit-stream to operate on
 * @return Number of bits that can still be read from the stream
 */
int speex_bits_remaining(SpeexBits *bits);


#endif
