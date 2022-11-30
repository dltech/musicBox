/* Copyright (C) 2002-2006 Jean-Marc Valin */
/**
    @file nb_celp.h
    @brief Narrowband CELP encoder/decoder
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

#ifndef NB_CELP_H
#define NB_CELP_H

#include "modes.h"
#include "speex_bits.h"
//#include "speex/speex_callbacks.h"
#include "filters.h"


#define NB_ORDER 10
#define NB_FRAME_SIZE 160
#define NB_SUBFRAME_SIZE 40
#define NB_NB_SUBFRAMES 4
#define NB_PITCH_START 17
#define NB_PITCH_END 144

#define NB_WINDOW_SIZE (NB_FRAME_SIZE+NB_SUBFRAME_SIZE)
#define NB_EXCBUF (NB_FRAME_SIZE+NB_PITCH_END+2)
#define NB_DEC_BUFFER (NB_FRAME_SIZE+2*NB_PITCH_END+NB_SUBFRAME_SIZE+12)

/**Structure representing the full state of the narrowband decoder*/
typedef struct DecState {
   const SpeexMode *mode;       /**< Mode corresponding to the state */
   int    first;                /**< Is this the first frame? */
   int    count_lost;           /**< Was the last frame lost? */
   spx_int32_t sampling_rate;

   spx_word16_t  last_ol_gain;  /**< Open-loop gain for previous frame */

   char  *stack;                /**< Pseudo-stack allocation for temporary memory */
   spx_word16_t excBuf[NB_DEC_BUFFER];        /**< Excitation buffer */
   spx_word16_t *exc;           /**< Start of excitation frame */
   spx_lsp_t old_qlsp[NB_ORDER];         /**< Quantized LSPs for previous frame */
   spx_coef_t interp_qlpc[NB_ORDER];     /**< Interpolated quantized LPCs */
   spx_mem_t mem_sp[NB_ORDER];           /**< Filter memory for synthesis signal */
   spx_mem_t mem_hp[2];         /**< High-pass filter memory */
   spx_word32_t pi_gain[NB_NB_SUBFRAMES];       /**< Gain of LPC filter at theta=pi (fe/2) */
   spx_word16_t *innov_save;    /** If non-NULL, innovation is copied here */

   spx_word16_t level;
   spx_word16_t max_level;
   spx_word16_t min_level;

   /* This is used in packet loss concealment */
   int    last_pitch;           /**< Pitch of last correctly decoded frame */
   spx_word16_t  last_pitch_gain; /**< Pitch gain of last correctly decoded frame */
   spx_word16_t  pitch_gain_buf[3]; /**< Pitch gain of last decoded frames */
   int    pitch_gain_buf_idx;   /**< Tail of the buffer */
   spx_uint32_t seed;            /** Seed used for random number generation */

   int    encode_submode;
   const SpeexSubmode * const *submodes; /**< Sub-mode data */
   int    submodeID;            /**< Activated sub-mode */
   int    lpc_enh_enabled;      /**< 1 when LPC enhancer is on, 0 otherwise */
//   SpeexCallback speex_callbacks[SPEEX_MAX_CALLBACKS];

//   SpeexCallback user_callback;

   /*Vocoder data*/
   spx_word16_t  voc_m1;
   spx_word32_t  voc_m2;
   spx_word16_t  voc_mean;
   int    voc_offset;

   int    dtx_enabled;
   int    isWideband;            /**< Is this used as part of the embedded wideband codec */
   int    highpass_enabled;        /**< Is the input filter enabled */
} DecState;


/** Initializes decoder state*/
void *nb_decoder_init(const SpeexMode *m);

/** Decodes one frame*/
int nb_decode(void *state, SpeexBits *bits, void *out);


#define speex_assert(cond) {if (!(cond)){;}}

#endif
