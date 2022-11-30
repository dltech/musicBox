/* Copyright (C) 2002 Jean-Marc Valin */
/**
   @file ltp.h
   @brief Long-Term Prediction functions
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

#ifndef LTP_H
#define LTP_H

#include "speex_bits.h"
#include "arch.h"

/** LTP parameters. */
typedef struct {
   const signed char *gain_cdbk;
   int     gain_bits;
   int     pitch_bits;
} ltp_params;

#define gain_3tap_to_1tap(g) (ABS(g[1]) + (g[0]>0 ? g[0] : -SHR16(g[0],1)) + (g[2]>0 ? g[2] : -SHR16(g[2],1)))


spx_word32_t inner_prod(const spx_word16_t *x, const spx_word16_t *y, int len);


// not exists (only meantioned)
void forced_pitch_unquant(
spx_word16_t exc[],             /* Input excitation */
spx_word32_t exc_out[],         /* Output excitation */
int   start,                    /* Smallest pitch value allowed */
int   end,                      /* Largest pitch value allowed */
spx_word16_t pitch_coef,        /* Voicing (pitch) coefficient */
const void *par,
int   nsf,                      /* Number of samples in subframe */
int *pitch_val,
spx_word16_t *gain_val,
SpeexBits *bits,
char *stack,
int lost,
int subframe_offset,
spx_word16_t last_pitch_gain,
int cdbk_offset
);

void pitch_unquant_3tap(
spx_word16_t exc[],             /* Input excitation */
spx_word32_t exc_out[],         /* Output excitation */
int   start,                    /* Smallest pitch value allowed */
int   end,                      /* Largest pitch value allowed */
spx_word16_t pitch_coef,        /* Voicing (pitch) coefficient */
const void *par,
int   nsf,                      /* Number of samples in subframe */
int *pitch_val,
spx_word16_t *gain_val,
SpeexBits *bits,
char *stack,
int lost,
int subframe_offset,
spx_word16_t last_pitch_gain,
int cdbk_offset
);


#endif /* LTP_H */
