/* Copyright (C) 2002-2006 Jean-Marc Valin
   File: ltp.c
   Long-Term Prediction functions

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

#include "string.h"
#include "ltp.h"
#include "stack_alloc.h"
#include "filters.h"
#include "math_approx.h"

#ifndef NULL
#define NULL 0
#endif

#ifndef OVERRIDE_INNER_PROD
spx_word32_t inner_prod(const spx_word16_t *x, const spx_word16_t *y, int len)
{
   spx_word32_t sum=0;
   len >>= 2;
   while(len--)
   {
      spx_word32_t part=0;
      part = MAC16_16(part,*x++,*y++);
      part = MAC16_16(part,*x++,*y++);
      part = MAC16_16(part,*x++,*y++);
      part = MAC16_16(part,*x++,*y++);
      /* HINT: If you had a 40-bit accumulator, you could shift only at the end */
      sum = ADD32(sum,SHR32(part,6));
   }
   return sum;
}
#endif

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
int count_lost,
int subframe_offset,
spx_word16_t last_pitch_gain,
int cdbk_offset
)
{
   int i;
   int pitch;
   int gain_index;
   spx_word16_t gain[3];
   const signed char *gain_cdbk;
   int gain_cdbk_size;
   const ltp_params *params;

   params = (const ltp_params*) par;
   gain_cdbk_size = 1<<params->gain_bits;
   gain_cdbk = params->gain_cdbk + 4*gain_cdbk_size*cdbk_offset;

   pitch = (int)speex_bits_unpack_unsigned(bits, params->pitch_bits);
   pitch += start;
   gain_index = (int)speex_bits_unpack_unsigned(bits, params->gain_bits);
   /*printf ("decode pitch: %d %d\n", pitch, gain_index);*/
   gain[0] = ADD16(32,(spx_word16_t)gain_cdbk[gain_index*4]);
   gain[1] = ADD16(32,(spx_word16_t)gain_cdbk[gain_index*4+1]);
   gain[2] = ADD16(32,(spx_word16_t)gain_cdbk[gain_index*4+2]);

   if (count_lost && pitch > subframe_offset)
   {
      spx_word16_t gain_sum;
      if (1) {

         spx_word16_t tmp = count_lost < 4 ? last_pitch_gain : SHR16(last_pitch_gain,1);
         if (tmp>62)
            tmp=62;
         gain_sum = (spx_word16_t)gain_3tap_to_1tap(gain);

         if (gain_sum > tmp)
         {
            spx_word16_t fact = DIV32_16(SHL32(EXTEND32(tmp),14),gain_sum);
            for (i=0;i<3;i++)
               gain[i]=(spx_word16_t)MULT16_16_Q14(fact,gain[i]);
         }

      }

   }

   *pitch_val = pitch;
   gain_val[0]=gain[0];
   gain_val[1]=gain[1];
   gain_val[2]=gain[2];
   gain[0] = SHL16(gain[0],7);
   gain[1] = SHL16(gain[1],7);
   gain[2] = SHL16(gain[2],7);
   memset(exc_out, 0, (unsigned)nsf*2);
   for (i=0;i<3;i++)
   {
      int j;
      int tmp1, tmp3;
      int pp=pitch+1-i;
      tmp1=nsf;
      if (tmp1>pp)
         tmp1=pp;
      for (j=0;j<tmp1;j++)
         exc_out[j]=MAC16_16(exc_out[j],gain[2-i],exc[j-pp]);
      tmp3=nsf;
      if (tmp3>pp+pitch)
         tmp3=pp+pitch;
      for (j=tmp1;j<tmp3;j++)
         exc_out[j]=MAC16_16(exc_out[j],gain[2-i],exc[j-pp-pitch]);
   }
   /*for (i=0;i<nsf;i++)
   exc[i]=PSHR32(exc32[i],13);*/
   (void)end;
   (void)pitch_coef;
   (void)stack;
}
