#include "usbcom.h"
#include "../speex/speex_simplest_dec/nb_celp.h"

SpeexBits bits;

spx_word16_t output_frame[NB_FRAME_SIZE];

int main(void) {
	usbComInit();
	// speex
	speex_bits_init(&bits);
	void *dec_state = speex_decoder_init(&speex_nb_mode);
	speex_bits_read_from(&bits, input_bytes, nbBytes);

	nb_decode(dec_state, &bits, output_frame);

	while(1);
}
