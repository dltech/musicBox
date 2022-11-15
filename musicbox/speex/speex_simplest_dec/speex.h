/* Copyright (C) 2002-2006 Jean-Marc Valin*/
/**
  @file speex.h
  @brief Describes the different modes of the codec
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


/** Returns a handle to a newly created decoder state structure. For now,
 * the mode argument can be &nb_mode or &wb_mode . In the future, more modes
 * may be added.  Note that for now if you have more than one channels to
 * decode, you need one state per channel.
 *
 * @param mode Speex mode (one of speex_nb_mode or speex_wb_mode)
 * @return A newly created decoder state or NULL if state allocation fails
 */
void *speex_decoder_init(const SpeexMode *mode);


/** Uses an existing decoder state to decode one frame of speech from
 * bit-stream bits. The output speech is saved written to out.
 *
 * @param state Decoder state
 * @param bits Bit-stream from which to decode the frame (NULL if the packet was lost)
 * @param out Where to write the decoded frame
 * @return return status (0 for no error, -1 for end of stream, -2 corrupt stream)
 */
int speex_decode(void *state, SpeexBits *bits, float *out);