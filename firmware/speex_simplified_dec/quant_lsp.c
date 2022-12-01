/* Copyright (C) 2002 Jean-Marc Valin
   File: quant_lsp.c
   LSP vector quantization

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


#include "quant_lsp.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "arch.h"
#include "inttypes.h"


#define LSP_LINEAR(i) (SHL16(i+1,11))
#define LSP_LINEAR_HIGH(i) (ADD16(MULT16_16_16(i,2560),6144))
#define LSP_DIV_256(x) (SHL16((spx_word16_t)x, 5))
#define LSP_DIV_512(x) (SHL16((spx_word16_t)x, 4))
#define LSP_DIV_1024(x) (SHL16((spx_word16_t)x, 3))
#define LSP_PI 25736


//#ifndef DISABLE_DECODER
void lsp_unquant_nb(spx_lsp_t *lsp, int order, SpeexBits *bits)
{
   uint32_t id, i;
   for (i=0;i<(uint32_t)order;i++)
      lsp[i]=LSP_LINEAR(i);


   id=speex_bits_unpack_unsigned(bits, 6);
   for (i=0;i<10;i++)
      lsp[i] = (spx_lsp_t)ADD32(lsp[i], LSP_DIV_256(cdbk_nb[id*10+i]));

   id=speex_bits_unpack_unsigned(bits, 6);
   for (i=0;i<5;i++)
      lsp[i] = (spx_lsp_t)ADD16(lsp[i], LSP_DIV_512(cdbk_nb_low1[id*5+i]));

   id=speex_bits_unpack_unsigned(bits, 6);
   for (i=0;i<5;i++)
      lsp[i] = (spx_lsp_t)ADD32(lsp[i], LSP_DIV_1024(cdbk_nb_low2[id*5+i]));

   id=speex_bits_unpack_unsigned(bits, 6);
   for (i=0;i<5;i++)
      lsp[i+5] = (spx_lsp_t)ADD32(lsp[i+5], LSP_DIV_512(cdbk_nb_high1[id*5+i]));

   id=speex_bits_unpack_unsigned(bits, 6);
   for (i=0;i<5;i++)
      lsp[i+5] = (spx_lsp_t)ADD32(lsp[i+5], LSP_DIV_1024(cdbk_nb_high2[id*5+i]));
}
//#endif /* DISABLE_DECODER */

//#ifndef DISABLE_DECODER
void lsp_unquant_lbr(spx_lsp_t *lsp, int order, SpeexBits *bits)
{
   uint32_t i,id;
   for (i=0;i<(uint32_t)order;i++)
      lsp[i]=LSP_LINEAR(i);


   id=speex_bits_unpack_unsigned(bits, 6);
   for (i=0;i<10;i++)
      lsp[i] += LSP_DIV_256(cdbk_nb[id*10+i]);

   id=speex_bits_unpack_unsigned(bits, 6);
   for (i=0;i<5;i++)
      lsp[i] += LSP_DIV_512(cdbk_nb_low1[id*5+i]);

   id=speex_bits_unpack_unsigned(bits, 6);
   for (i=0;i<5;i++)
      lsp[i+5] += LSP_DIV_512(cdbk_nb_high1[id*5+i]);

}
//#endif /* DISABLE_DECODER */

// #ifndef DISABLE_WIDEBAND
// extern const signed char high_lsp_cdbk[];
// extern const signed char high_lsp_cdbk2[];
//
// #ifndef DISABLE_DECODER
// void lsp_unquant_high(spx_lsp_t *lsp, int order, SpeexBits *bits)
// {
//
//    int i, id;
//    for (i=0;i<order;i++)
//       lsp[i]=LSP_LINEAR_HIGH(i);
//
//
//    id=speex_bits_unpack_unsigned(bits, 6);
//    for (i=0;i<order;i++)
//       lsp[i] += LSP_DIV_256(high_lsp_cdbk[id*order+i]);
//
//
//    id=speex_bits_unpack_unsigned(bits, 6);
//    for (i=0;i<order;i++)
//       lsp[i] += LSP_DIV_512(high_lsp_cdbk2[id*order+i]);
// }
// #endif /* DISABLE_DECODER */
//
// #endif /* DISABLE_WIDEBAND */
