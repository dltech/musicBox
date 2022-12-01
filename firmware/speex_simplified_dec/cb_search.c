/* Copyright (C) 2002-2006 Jean-Marc Valin
   File: cb_search.c

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

#include "cb_search.h"
#include "filters.h"
//#include "vq.h"
#include "arch.h"
#include "math_approx.h"
#include "stack_alloc.h"


//#ifndef DISABLE_DECODER
void split_cb_shape_sign_unquant(
spx_sig_t *exc,
const void *par,                      /* non-overlapping codebook */
int   nsf,                      /* number of samples in subframe */
SpeexBits *bits,
char *stack,
spx_uint32_t *seed
)
{
   int i,j;
   VARDECL(int *ind);
   VARDECL(int *signs);
   const signed char *shape_cb;
   int subvect_size, nb_subvect;
   const split_cb_params *params;
   int have_sign;

   params = (const split_cb_params *) par;
   subvect_size = params->subvect_size;
   nb_subvect = params->nb_subvect;

   shape_cb = params->shape_cb;
   have_sign = params->have_sign;

   ALLOC(ind, nb_subvect, int);
   ALLOC(signs, nb_subvect, int);

   /* Decode codewords and gains */
   for (i=0;i<nb_subvect;i++)
   {
      if (have_sign)
         signs[i] = (int)speex_bits_unpack_unsigned(bits, 1);
      else
         signs[i] = 0;
      ind[i] = (int)speex_bits_unpack_unsigned(bits, params->shape_bits);
   }
   /* Compute decoded excitation */
   for (i=0;i<nb_subvect;i++)
   {
      spx_word16_t s=1;
      if (signs[i])
         s=-1;
      if (s==1)
      {
         for (j=0;j<subvect_size;j++)
            exc[subvect_size*i+j]=SHL32(EXTEND32(shape_cb[ind[i]*subvect_size+j]),SIG_SHIFT-5);
      } else {
         for (j=0;j<subvect_size;j++)
            exc[subvect_size*i+j]=NEG32(SHL32(EXTEND32(shape_cb[ind[i]*subvect_size+j]),SIG_SHIFT-5));
      }
   }
   (void)nsf;
   (void)seed;
   (void)par;
   (void)bits;
   (void)stack;
}
//#endif /* DISABLE_DECODER */


//#ifndef DISABLE_DECODER
void noise_codebook_unquant(
spx_sig_t *exc,
const void *par,                      /* non-overlapping codebook */
int   nsf,                      /* number of samples in subframe */
SpeexBits *bits,
char *stack,
spx_uint32_t *seed
)
{
   int i;
   /* FIXME: This is bad, but I don't think the function ever gets called anyway */
   for (i=0;i<nsf;i++)
      exc[i]=SHL32(EXTEND32(speex_rand(1, seed)),SIG_SHIFT);
   (void)par;
   (void)bits;
   (void)stack;
}
//#endif /* DISABLE_DECODER */
