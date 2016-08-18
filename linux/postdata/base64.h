#include <stdint.h>

void base64_encode(const unsigned char *data, size_t input_length, size_t *output_length, unsigned char *encoded_data);

unsigned char *base64_decode(const unsigned char *data, size_t input_length, size_t *output_length);
