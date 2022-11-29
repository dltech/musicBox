/* Copyright (C) 2002 Jean-Marc Valin
   File: speex_bits.c

   Handles bit packing/unpacking

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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "speex/speex_bits.h"
#include "arch.h"
#include "os_support.h"

/* Maximum size of the bit-stream (for fixed-size allocation) */
#ifndef MAX_CHARS_PER_FRAME
#define MAX_CHARS_PER_FRAME (2000/BYTES_PER_CHAR)
#endif

EXPORT void speex_bits_init_buffer(SpeexBits *bits, void *buff, int buf_size)
{
   bits->chars = (char*)buff;
   bits->buf_size = buf_size;

   bits->owner=0;

   speex_bits_reset(bits);
}

EXPORT void speex_bits_set_bit_buffer(SpeexBits *bits, void *buff, int buf_size)
{
   bits->chars = (char*)buff;
   bits->buf_size = buf_size;

   bits->owner=0;

   bits->nbBits=buf_size<<LOG2_BITS_PER_CHAR;
   bits->charPtr=0;
   bits->bitPtr=0;
   bits->overflow=0;

}

EXPORT void speex_bits_reset(SpeexBits *bits)
{
   /* We only need to clear the first byte now */
   bits->chars[0]=0;
   bits->nbBits=0;
   bits->charPtr=0;
   bits->bitPtr=0;
   bits->overflow=0;
}

EXPORT void speex_bits_read_from(SpeexBits *bits, const char *chars, int len)
{
   int i;
   int nchars = len / BYTES_PER_CHAR;
   if (nchars > bits->buf_size)
   {
      speex_notify("Packet is larger than allocated buffer");
      if (bits->owner)
      {
         char *tmp = (char*)speex_realloc(bits->chars, nchars);
         if (tmp)
         {
            bits->buf_size=nchars;
            bits->chars=tmp;
         } else {
            nchars=bits->buf_size;
            speex_warning("Could not resize input buffer: truncating input");
         }
      } else {
         speex_warning("Do not own input buffer: truncating oversize input");
         nchars=bits->buf_size;
      }
   }
#if (BYTES_PER_CHAR==2)
/* Swap bytes to proper endian order (could be done externally) */
#define HTOLS(A) ((((A) >> 8)&0xff)|(((A) & 0xff)<<8))
#else
#define HTOLS(A) (A)
#endif
   for (i=0;i<nchars;i++)
      bits->chars[i]=HTOLS(chars[i]);

   bits->nbBits=nchars<<LOG2_BITS_PER_CHAR;
   bits->charPtr=0;
   bits->bitPtr=0;
   bits->overflow=0;
}

EXPORT void speex_bits_read_whole_bytes(SpeexBits *bits, const char *chars, int nbytes)
{
   int i,pos;
   int nchars = nbytes/BYTES_PER_CHAR;

   if (((bits->nbBits+BITS_PER_CHAR-1)>>LOG2_BITS_PER_CHAR)+nchars > bits->buf_size)
   {
      /* Packet is larger than allocated buffer */
      if (bits->owner)
      {
         char *tmp = (char*)speex_realloc(bits->chars, (bits->nbBits>>LOG2_BITS_PER_CHAR)+nchars+1);
         if (tmp)
         {
            bits->buf_size=(bits->nbBits>>LOG2_BITS_PER_CHAR)+nchars+1;
            bits->chars=tmp;
         } else {
            nchars=bits->buf_size-(bits->nbBits>>LOG2_BITS_PER_CHAR)-1;
            speex_warning("Could not resize input buffer: truncating oversize input");
         }
      } else {
         speex_warning("Do not own input buffer: truncating oversize input");
         nchars=bits->buf_size;
      }
   }

   speex_bits_flush(bits);
   pos=bits->nbBits>>LOG2_BITS_PER_CHAR;
   for (i=0;i<nchars;i++)
      bits->chars[pos+i]=HTOLS(chars[i]);
   bits->nbBits+=nchars<<LOG2_BITS_PER_CHAR;
}

EXPORT unsigned int speex_bits_unpack_unsigned(SpeexBits *bits, int nbBits)
{
   unsigned int d=0;
   if ((bits->charPtr<<LOG2_BITS_PER_CHAR)+bits->bitPtr+nbBits>bits->nbBits)
      bits->overflow=1;
   if (bits->overflow)
      return 0;
   while(nbBits)
   {
      d<<=1;
      d |= (bits->chars[bits->charPtr]>>(BITS_PER_CHAR-1 - bits->bitPtr))&1;
      bits->bitPtr++;
      if (bits->bitPtr==BITS_PER_CHAR)
      {
         bits->bitPtr=0;
         bits->charPtr++;
      }
      nbBits--;
   }
   return d;
}

EXPORT void speex_bits_advance(SpeexBits *bits, int n)
{
    if (((bits->charPtr<<LOG2_BITS_PER_CHAR)+bits->bitPtr+n>bits->nbBits) || bits->overflow){
      bits->overflow=1;
      return;
    }
   bits->charPtr += (bits->bitPtr+n) >> LOG2_BITS_PER_CHAR; /* divide by BITS_PER_CHAR */
   bits->bitPtr = (bits->bitPtr+n) & (BITS_PER_CHAR-1);       /* modulo by BITS_PER_CHAR */
}

EXPORT int speex_bits_remaining(SpeexBits *bits)
{
   if (bits->overflow)
      return -1;
   else
      return bits->nbBits-((bits->charPtr<<LOG2_BITS_PER_CHAR)+bits->bitPtr);
}
