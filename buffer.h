/*
 * buffer.h
 *
 *  Created on: 03-07-2015
 *      Author: Maciek
 */

#ifndef BUFFER_H_
#define BUFFER_H_

#include <stdint.h>


#define BIN_COUNT 16
#define BUFF_COUNT 2

/** type depends of how many data are
 * cumulated within statistics.
 */
typedef uint8_t bin_t;


typedef enum {
	FREE = 0,
	IN_WRITE,
	FULL,
	IN_READ
}stat_t;

/**
 * Single buffer consists of histogram bins
 * and number of total samples used in hist.
 */
typedef struct histogram {
	bin_t bin[BIN_COUNT];
	bin_t total;
	stat_t status;
} histogram_t;

/**
 * Stack of 'units' for buffering histograms.
 */
typedef struct h_buffer {
	histogram_t buff[BUFF_COUNT];
	uint8_t in_read_idx;	//index of buff
	uint8_t in_write_idx;	//index of buff
	bin_t	max_total;
} h_buff_t;

void h_buff_clean_unit(h_buff_t *buff, uint8_t unit_nr);
void h_buff_clean_all(h_buff_t *buff);
void h_buff_proc_data(h_buff_t *buff, uint8_t data);
void h_buff_to_read_ptr(h_buff_t *buff, histogram_t ** single_unit);




#endif /* BUFFER_H_ */
