#include "inttypes.h"
#include "usbcom.h"
#include "../speex_simplified_dec/nb_celp.h"
#include "../speex_simplified_dec/modes.h"

extern const SpeexMode speex_nb_mode;
SpeexBits bits;
uint8_t music[50];
//spx_word16_t output_frame[100];
spx_word16_t output_frame[NB_FRAME_SIZE];

int main(void) {
	usbComInit();
	// speex
	speex_bits_init_buffer(&bits, music, 50);
	void *dec_state = nb_decoder_init(&speex_nb_mode);
	speex_bits_read_from(&bits, 0, 10);

	nb_decode(dec_state, &bits, output_frame);

	while(1);
}
