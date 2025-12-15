// Public API for VOC signal processing helpers.
#pragma once

enum {
	WINDOW_COUNT = 4,
	WINDOW_WIDTH = 6,
	BUFFER_SIZE = WINDOW_COUNT * WINDOW_WIDTH
};

uint16_t buf_push(uint16_t voc);
uint16_t median_u16(const uint16_t *src, size_t n, uint16_t *work_buf, size_t work_buf_size);
int median_list(uint16_t *out); 
int voc_flag();
int get_state_s();
