/* Copyright (C) 2002-2006 Jean-Marc Valin
   File: nb_celp.c

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

#include "lsp.h"
#include "filters.h"
#include "modes.h"

#include "speex_bits.h"
#include "stack_alloc.h"
#include "arch.h"
#include "math_approx.h"
#include "config.h"

#include "nb_celp.h"

#ifndef NULL
#define NULL 0
#endif

#define SUBMODE(x) st->submodes[st->submodeID]->x

/* Default size for the encoder and decoder stack (can be changed at compile time).
   This does not apply when using variable-size arrays or alloca. */
#ifndef NB_DEC_STACK
#define NB_DEC_STACK (4000*sizeof(spx_sig_t))
#endif


static const spx_word32_t ol_gain_table[32]={18900, 25150, 33468, 44536, 59265, 78865, 104946, 139653, 185838, 247297, 329081, 437913, 582736, 775454, 1031906, 1373169, 1827293, 2431601, 3235761, 4305867, 5729870, 7624808, 10146425, 13501971, 17967238, 23909222, 31816294, 42338330, 56340132, 74972501, 99766822, 132760927};
static const spx_word16_t exc_gain_quant_scal3_bound[7]={1841, 3883, 6051, 8062, 10444, 13580, 18560};
static const spx_word16_t exc_gain_quant_scal3[8]={1002, 2680, 5086, 7016, 9108, 11781, 15380, 21740};
static const spx_word16_t exc_gain_quant_scal1_bound[1]={14385};
static const spx_word16_t exc_gain_quant_scal1[2]={11546, 17224};

#define LSP_MARGIN 16
#define LSP_DELTA1 6553
#define LSP_DELTA2 1638




extern const spx_word16_t lag_window[];
extern const spx_word16_t lpc_window[];

DecState state;
DecState *st;
spx_sig_t stack_tipo_alloc[NB_DEC_STACK] = 0;
st.stack = stack_tipo_alloc;

void *nb_decoder_init(const SpeexMode *m)
{
   const SpeexNBMode *mode;
   int i;

   mode=(const SpeexNBMode*)m->mode;

   st->mode=m;

   st->encode_submode = 1;

   st->first=1;
   /* Codec parameters, should eventually have several "modes"*/

   st->submodes=mode->submodes;

   st->submodeID=mode->3;

   st->lpc_enh_enabled=1;

   SPEEX_MEMSET(st->excBuf, 0, NB_FRAME_SIZE + NB_PITCH_END);

   st->last_pitch = 40;
   st->count_lost=0;
   st->pitch_gain_buf[0] = st->pitch_gain_buf[1] = st->pitch_gain_buf[2] = 0;
   st->pitch_gain_buf_idx = 0;
   st->seed = 1000;

   st->sampling_rate=8000;
   st->last_ol_gain = 0;

//   st->user_callback.func = &speex_default_user_handler;
   st->user_callback.data = NULL;
   for (i=0;i<16;i++)
      st->speex_callbacks[i].func = NULL;

   st->voc_m1=st->voc_m2=st->voc_mean=0;
   st->voc_offset=0;
   st->dtx_enabled=0;
   st->isWideband = 0;

//   st->highpass_enabled = 1;
   st->highpass_enabled = 0;

   st->lpc_enh_enabled = NULL;

   return st;
}

#define median3(a, b, c)	((a) < (b) ? ((b) < (c) ? (b) : ((a) < (c) ? (c) : (a))) : ((c) < (b) ? (b) : ((c) < (a) ? (c) : (a))))

const spx_word16_t attenuation[10] = {32767, 31483, 27923, 22861, 17278, 12055, 7764, 4616, 2533, 1283};


/* Just so we don't need to carry the complete wideband mode information */
static const int wb_skip_table[8] = {0, 36, 112, 192, 352, 0, 0, 0};

int nb_decode(void *state, SpeexBits *bits, void *vout)
{
   DecState *st;
   int i, sub;
   int pitch;
   spx_word16_t pitch_gain[3];
   spx_word32_t ol_gain=0;
   int ol_pitch=0;
   spx_word16_t ol_pitch_coef=0;
   int best_pitch=40;
   spx_word16_t best_pitch_gain=0;
   int wideband;
   int m;
   char *stack;
   VARDECL(spx_sig_t *innov);
   VARDECL(spx_word32_t *exc32);
   VARDECL(spx_coef_t *ak);
   VARDECL(spx_lsp_t *qlsp);
   spx_word16_t pitch_average=0;

   spx_word16_t *out = (spx_word16_t*)vout;
   VARDECL(spx_lsp_t *interp_qlsp);

   st=(DecState*)state;
   stack=st->stack;

   st->exc = st->excBuf + 2*NB_PITCH_END + NB_SUBFRAME_SIZE + 6;

   /* Check if we're in DTX mode*/
   if (!bits && st->dtx_enabled)
   {
      st->submodeID=0;
   } else
   {
      /* If bits is NULL, consider the packet to be lost (what could we do anyway) */
      if (!bits)
      {
         return 0;
      }

      if (st->encode_submode)
      {

      /* Search for next narrowband block (handle requests, skip wideband blocks) */
      do {
         if (speex_bits_remaining(bits)<5)
            return -1;
         wideband = speex_bits_unpack_unsigned(bits, 1);
         if (wideband) /* Skip wideband block (for compatibility) */
         {
            int submode;
            int advance;
            advance = submode = speex_bits_unpack_unsigned(bits, SB_SUBMODE_BITS);
            /*speex_mode_query(&speex_wb_mode, SPEEX_SUBMODE_BITS_PER_FRAME, &advance);*/
            advance = wb_skip_table[submode];
            if (advance < 0)
            {
               //speex_notify("Invalid mode encountered. The stream is corrupted.");
               return -2;
            }
            advance -= (SB_SUBMODE_BITS+1);
            speex_bits_advance(bits, advance);

            if (speex_bits_remaining(bits)<5)
               return -1;
            wideband = speex_bits_unpack_unsigned(bits, 1);
            if (wideband)
            {
               advance = submode = speex_bits_unpack_unsigned(bits, SB_SUBMODE_BITS);
               /*speex_mode_query(&speex_wb_mode, SPEEX_SUBMODE_BITS_PER_FRAME, &advance);*/
               advance = wb_skip_table[submode];
               if (advance < 0)
               {
                  //speex_notify("Invalid mode encountered. The stream is corrupted.");
                  return -2;
               }
               advance -= (SB_SUBMODE_BITS+1);
               speex_bits_advance(bits, advance);
               wideband = speex_bits_unpack_unsigned(bits, 1);
               if (wideband)
               {
                  //speex_notify("More than two wideband layers found. The stream is corrupted.");
                  return -2;
               }

            }
         }
         if (speex_bits_remaining(bits)<4)
            return -1;
         /* FIXME: Check for overflow */
         m = speex_bits_unpack_unsigned(bits, 4);
         if (m==15) /* We found a terminator */
         {
            return -1;
         } else if (m==14) /* Speex in-band request */
         {
            int ret = speex_inband_handler(bits, st->speex_callbacks, state);
            if (ret)
               return ret;
         } else if (m==13) /* User in-band request */
         {
            int ret = st->user_callback.func(bits, state, st->user_callback.data);
            if (ret)
               return ret;
         } else if (m>8) /* Invalid mode */
         {
            //speex_notify("Invalid mode encountered. The stream is corrupted.");
            return -2;
         }

      } while (m>8);

      /* Get the sub-mode that was used */
      st->submodeID = m;
      }

   }

   /* Shift all buffers by one frame */
   SPEEX_MOVE(st->excBuf, st->excBuf+NB_FRAME_SIZE, 2*NB_PITCH_END + NB_SUBFRAME_SIZE + 12);

   /* If null mode (no transmission), just set a couple things to zero*/
   if (st->submodes[st->submodeID] == NULL)
   {
      VARDECL(spx_coef_t *lpc);
      ALLOC(lpc, NB_ORDER, spx_coef_t);
      bw_lpc(QCONST16(0.93f,15), st->interp_qlpc, lpc, NB_ORDER);
      {
         spx_word16_t innov_gain=0;
         /* FIXME: This was innov, not exc */
         innov_gain = compute_rms16(st->exc, NB_FRAME_SIZE);
         for (i=0;i<NB_FRAME_SIZE;i++)
            st->exc[i]=speex_rand(innov_gain, &st->seed);
      }


      st->first=1;

      /* Final signal synthesis from excitation */
      iir_mem16(st->exc, lpc, out, NB_FRAME_SIZE, NB_ORDER, st->mem_sp, stack);

      st->count_lost=0;
      return 0;
   }

   ALLOC(qlsp, NB_ORDER, spx_lsp_t);

   /* Unquantize LSPs */
   SUBMODE(lsp_unquant)(qlsp, NB_ORDER, bits);

   /*Damp memory if a frame was lost and the LSP changed too much*/
   if (st->count_lost)
   {
      spx_word16_t fact;
      spx_word32_t lsp_dist=0;
      for (i=0;i<NB_ORDER;i++)
         lsp_dist = ADD32(lsp_dist, EXTEND32(ABS(st->old_qlsp[i] - qlsp[i])));
#ifdef FIXED_POINT
      fact = SHR16(19661,SHR32(lsp_dist,LSP_SHIFT+2));
#else
      fact = .6*exp(-.2*lsp_dist);
#endif
      for (i=0;i<NB_ORDER;i++)
         st->mem_sp[i] = MULT16_32_Q15(fact,st->mem_sp[i]);
   }


   /* Handle first frame and lost-packet case */
   if (st->first || st->count_lost)
   {
      for (i=0;i<NB_ORDER;i++)
         st->old_qlsp[i] = qlsp[i];
   }

   /* Get open-loop pitch estimation for low bit-rate pitch coding */
   if (SUBMODE(lbr_pitch)!=-1)
   {
      ol_pitch = NB_PITCH_START+speex_bits_unpack_unsigned(bits, 7);
   }

   if (SUBMODE(forced_pitch_gain))
   {
      int quant;
      quant = speex_bits_unpack_unsigned(bits, 4);
      ol_pitch_coef=MULT16_16_P15(QCONST16(0.066667,15),SHL16(quant,GAIN_SHIFT));
   }

   /* Get global excitation gain */
   {
      int qe;
      qe = speex_bits_unpack_unsigned(bits, 5);
      /* FIXME: Perhaps we could slightly lower the gain here when the output is going to saturate? */
      ol_gain = MULT16_32_Q15(28406,ol_gain_table[qe]);

   }

   ALLOC(ak, NB_ORDER, spx_coef_t);
   ALLOC(innov, NB_SUBFRAME_SIZE, spx_sig_t);
   ALLOC(exc32, NB_SUBFRAME_SIZE, spx_word32_t);

   if (st->submodeID==1)
   {
      int extra;
      extra = speex_bits_unpack_unsigned(bits, 4);

      if (extra==15)
         st->dtx_enabled=1;
      else
         st->dtx_enabled=0;
   }
   if (st->submodeID>1)
      st->dtx_enabled=0;

   /*Loop on subframes */
   for (sub=0;sub<NB_NB_SUBFRAMES;sub++)
   {
      int offset;
      spx_word16_t *exc;
      spx_word16_t *innov_save = NULL;
      spx_word16_t tmp;

      /* Offset relative to start of frame */
      offset = NB_SUBFRAME_SIZE*sub;
      /* Excitation */
      exc=st->exc+offset;
      /* Original signal */
      if (st->innov_save)
         innov_save = st->innov_save+offset;


      /* Reset excitation */
      SPEEX_MEMSET(exc, 0, NB_SUBFRAME_SIZE);

      /*Adaptive codebook contribution*/
      speex_assert (SUBMODE(ltp_unquant));
      {
         int pit_min, pit_max;
         /* Handle pitch constraints if any */
         if (SUBMODE(lbr_pitch) != -1)
         {
            int margin;
            margin = SUBMODE(lbr_pitch);
            if (margin)
            {
/* GT - need optimization?
               if (ol_pitch < NB_PITCH_START+margin-1)
                  ol_pitch=NB_PITCH_START+margin-1;
               if (ol_pitch > NB_PITCH_END-margin)
                  ol_pitch=NB_PITCH_END-margin;
               pit_min = ol_pitch-margin+1;
               pit_max = ol_pitch+margin;
*/
               pit_min = ol_pitch-margin+1;
               if (pit_min < NB_PITCH_START)
		  pit_min = NB_PITCH_START;
               pit_max = ol_pitch+margin;
               if (pit_max > NB_PITCH_END)
		  pit_max = NB_PITCH_END;
            } else {
               pit_min = pit_max = ol_pitch;
            }
         } else {
            pit_min = NB_PITCH_START;
            pit_max = NB_PITCH_END;
         }



         SUBMODE(ltp_unquant)(exc, exc32, pit_min, pit_max, ol_pitch_coef, SUBMODE(ltp_params),
                 NB_SUBFRAME_SIZE, &pitch, &pitch_gain[0], bits, stack,
                 st->count_lost, offset, st->last_pitch_gain, 0);

         /* Ensuring that things aren't blowing up as would happen if e.g. an encoder is
         crafting packets to make us produce NaNs and slow down the decoder (vague DoS threat).
         We can probably be even more aggressive and limit to 15000 or so. */
         sanitize_values32(exc32, NEG32(QCONST32(32000,SIG_SHIFT-1)), QCONST32(32000,SIG_SHIFT-1), NB_SUBFRAME_SIZE);

         tmp = gain_3tap_to_1tap(pitch_gain);

         pitch_average += tmp;
         if ((tmp>best_pitch_gain&&ABS(2*best_pitch-pitch)>=3&&ABS(3*best_pitch-pitch)>=4&&ABS(4*best_pitch-pitch)>=5)
              || (tmp>MULT16_16_Q15(QCONST16(.6,15),best_pitch_gain)&&(ABS(best_pitch-2*pitch)<3||ABS(best_pitch-3*pitch)<4||ABS(best_pitch-4*pitch)<5))
              || (MULT16_16_Q15(QCONST16(.67,15),tmp)>best_pitch_gain&&(ABS(2*best_pitch-pitch)<3||ABS(3*best_pitch-pitch)<4||ABS(4*best_pitch-pitch)<5)) )
         {
            best_pitch = pitch;
            if (tmp > best_pitch_gain)
               best_pitch_gain = tmp;
         }
      }

      /* Unquantize the innovation */
      {
         int q_energy;
         spx_word32_t ener;

         SPEEX_MEMSET(innov, 0, NB_SUBFRAME_SIZE);

         /* Decode sub-frame gain correction */
         if (SUBMODE(have_subframe_gain)==3)
         {
            q_energy = speex_bits_unpack_unsigned(bits, 3);
            ener = MULT16_32_Q14(exc_gain_quant_scal3[q_energy],ol_gain);
         } else if (SUBMODE(have_subframe_gain)==1)
         {
            q_energy = speex_bits_unpack_unsigned(bits, 1);
            ener = MULT16_32_Q14(exc_gain_quant_scal1[q_energy],ol_gain);
         } else {
            ener = ol_gain;
         }

         speex_assert (SUBMODE(innovation_unquant));
         {
            /*Fixed codebook contribution*/
            SUBMODE(innovation_unquant)(innov, SUBMODE(innovation_params), NB_SUBFRAME_SIZE, bits, stack, &st->seed);
            /* De-normalize innovation and update excitation */

            signal_mul(innov, innov, ener, NB_SUBFRAME_SIZE);

            for (i=0;i<NB_SUBFRAME_SIZE;i++)
               exc[i]=EXTRACT16(SATURATE32(PSHR32(ADD32(SHL32(exc32[i],1),innov[i]),SIG_SHIFT),32767));
            /*print_vec(exc, 40, "innov");*/
            if (innov_save)
            {
               for (i=0;i<NB_SUBFRAME_SIZE;i++)
                  innov_save[i] = EXTRACT16(PSHR32(innov[i], SIG_SHIFT));
            }
         }

         /*Vocoder mode*/
         if (st->submodeID==1)
         {
            spx_word16_t g=ol_pitch_coef;
            g=MULT16_16_P14(QCONST16(1.5f,14),(g-QCONST16(.2f,6)));
            if (g<0)
               g=0;
            if (g>GAIN_SCALING)
               g=GAIN_SCALING;

            SPEEX_MEMSET(exc, 0, NB_SUBFRAME_SIZE);
            while (st->voc_offset<NB_SUBFRAME_SIZE)
            {
               /* exc[st->voc_offset]= g*sqrt(2*ol_pitch)*ol_gain;
                  Not quite sure why we need the factor of two in the sqrt */
               if (st->voc_offset>=0)
                  exc[st->voc_offset]=MULT16_16(spx_sqrt(MULT16_16_16(2,ol_pitch)),EXTRACT16(PSHR32(MULT16_16(g,PSHR32(ol_gain,SIG_SHIFT)),6)));
               st->voc_offset+=ol_pitch;
            }
            st->voc_offset -= NB_SUBFRAME_SIZE;

            for (i=0;i<NB_SUBFRAME_SIZE;i++)
            {
               spx_word16_t exci=exc[i];
               exc[i]= ADD16(ADD16(MULT16_16_Q15(QCONST16(.7f,15),exc[i]) , MULT16_16_Q15(QCONST16(.3f,15),st->voc_m1)),
                             SUB16(MULT16_16_Q15(Q15_ONE-MULT16_16_16(QCONST16(.85f,9),g),EXTRACT16(PSHR32(innov[i],SIG_SHIFT))),
                                   MULT16_16_Q15(MULT16_16_16(QCONST16(.15f,9),g),EXTRACT16(PSHR32(st->voc_m2,SIG_SHIFT)))
                                  ));
               st->voc_m1 = exci;
               st->voc_m2=innov[i];
               st->voc_mean = EXTRACT16(PSHR32(ADD32(MULT16_16(QCONST16(.8f,15),st->voc_mean), MULT16_16(QCONST16(.2f,15),exc[i])), 15));
               exc[i]-=st->voc_mean;
            }
         }

      }
   }

   ALLOC(interp_qlsp, NB_ORDER, spx_lsp_t);

   if (st->lpc_enh_enabled && SUBMODE(comb_gain)>0 && !st->count_lost)
   {
      multicomb(st->exc-NB_SUBFRAME_SIZE, out, st->interp_qlpc, NB_ORDER, 2*NB_SUBFRAME_SIZE, best_pitch, 40, SUBMODE(comb_gain), stack);
      multicomb(st->exc+NB_SUBFRAME_SIZE, out+2*NB_SUBFRAME_SIZE, st->interp_qlpc, NB_ORDER, 2*NB_SUBFRAME_SIZE, best_pitch, 40, SUBMODE(comb_gain), stack);
   } else {
      SPEEX_COPY(out, &st->exc[-NB_SUBFRAME_SIZE], NB_FRAME_SIZE);
   }

   /* If the last packet was lost, re-scale the excitation to obtain the same energy as encoded in ol_gain */
   if (st->count_lost)
   {
      spx_word16_t exc_ener;
      spx_word32_t gain32;
      spx_word16_t gain;
      exc_ener = compute_rms16 (st->exc, NB_FRAME_SIZE);
      gain32 = PDIV32(ol_gain, ADD16(exc_ener,1));

	   if (gain32 > 32767)
         gain32 = 32767;
      gain = EXTRACT16(gain32);

      for (i=0;i<NB_FRAME_SIZE;i++)
      {
         st->exc[i] = MULT16_16_Q14(gain, st->exc[i]);
         out[i]=st->exc[i-NB_SUBFRAME_SIZE];
      }
   }

   /*Loop on subframes */
   for (sub=0;sub<NB_NB_SUBFRAMES;sub++)
   {
      int offset;
      spx_word16_t *sp;

      /* Offset relative to start of frame */
      offset = NB_SUBFRAME_SIZE*sub;
      /* Original signal */
      sp=out+offset;

      /* LSP interpolation (quantized and unquantized) */
      lsp_interpolate(st->old_qlsp, qlsp, interp_qlsp, NB_ORDER, sub, NB_NB_SUBFRAMES, LSP_MARGIN);

      /* Compute interpolated LPCs (unquantized) */
      lsp_to_lpc(interp_qlsp, ak, NB_ORDER, stack);

      /* Compute analysis filter at w=pi */
      {
         spx_word32_t pi_g=LPC_SCALING;
         for (i=0;i<NB_ORDER;i+=2)
         {
            /*pi_g += -st->interp_qlpc[i] +  st->interp_qlpc[i+1];*/
            pi_g = ADD32(pi_g, SUB32(EXTEND32(ak[i+1]),EXTEND32(ak[i])));
         }
         st->pi_gain[sub] = pi_g;
      }

      iir_mem16(sp, st->interp_qlpc, sp, NB_SUBFRAME_SIZE, NB_ORDER,
                st->mem_sp, stack);

      for (i=0;i<NB_ORDER;i++)
         st->interp_qlpc[i] = ak[i];

   }

//   if (st->highpass_enabled)
//      highpass(out, out, NB_FRAME_SIZE, (st->isWideband?HIGHPASS_WIDEBAND:HIGHPASS_NARROWBAND)|HIGHPASS_OUTPUT, st->mem_hp);
   /*for (i=0;i<NB_FRAME_SIZE;i++)
     printf ("%d\n", (int)st->frame[i]);*/

   /* Tracking output level */
   st->level = 1+PSHR32(ol_gain,SIG_SHIFT);
   st->max_level = MAX16(MULT16_16_Q15(QCONST16(.99f,15), st->max_level), st->level);
   st->min_level = MIN16(ADD16(1,MULT16_16_Q14(QCONST16(1.01f,14), st->min_level)), st->level);
   if (st->max_level < st->min_level+1)
      st->max_level = st->min_level+1;
   /*printf ("%f %f %f %d\n", og, st->min_level, st->max_level, update);*/

   /* Store the LSPs for interpolation in the next frame */
   for (i=0;i<NB_ORDER;i++)
      st->old_qlsp[i] = qlsp[i];

   /* The next frame will not be the first (Duh!) */
   st->first = 0;
   st->count_lost=0;
   st->last_pitch = best_pitch;
   st->last_pitch_gain = PSHR16(pitch_average,2);

   st->pitch_gain_buf[st->pitch_gain_buf_idx++] = st->last_pitch_gain;
   if (st->pitch_gain_buf_idx > 2) /* rollover */
      st->pitch_gain_buf_idx = 0;

   st->last_ol_gain = ol_gain;

   return 0;
}
