/*
 * buffer.c
 *
 *  Created on: 03-07-2015
 *      Author: Maciek
 */
#include "buffer.h"



void h_buff_clean_unit(h_buff_t *buff, uint8_t unit_nr)
{
	for(bin_t j = 0; j < BIN_COUNT; j++)
	{
		buff->buff[unit_nr].bin[j] = 0;
	}
	buff->buff[unit_nr].total = 0;
	buff->buff[unit_nr].status = FREE;
}

void h_buff_clean_all(h_buff_t *buff)
{
	//for each buffer unit
	for(uint8_t i = 0; i < BUFF_COUNT; i++)
	{
		h_buff_clean_unit(buff, i);
	}
}

void h_buff_proc_data(h_buff_t *buff, uint8_t data)
{
	if(BUFF_COUNT > buff->in_write_idx)
	{
		//assert((data >> 4) < BIN_COUNT);
		buff->buff[buff->in_write_idx].bin[data >> 4]++;
		if(buff->buff[buff->in_write_idx].total >= buff->max_total)
		{
			buff->buff[buff->in_write_idx].status = FULL;
			buff->in_write_idx = BUFF_COUNT; //status 'invalid'
			for(uint8_t i = 0; i < BUFF_COUNT; i++)
			{
				if(FREE == buff->buff[i].status)
				{
					buff->buff[i].status = IN_WRITE;
					buff->in_write_idx = i;
					break;
				}
			}
		}
	}
	else
	{
		for(uint8_t i = 0; i < BUFF_COUNT; i++)
		{
			if(FREE == buff->buff[i].status)
			{
				buff->buff[i].status = IN_WRITE;
				buff->in_write_idx = i;
				break;
			}
		}
	}
	//for each buffer unit


}

void h_buff_to_read_ptr(h_buff_t *buff, histogram_t ** single_unit)
{
	if(BUFF_COUNT > buff->in_read_idx)
	{
		*single_unit = &(buff->buff[buff->in_read_idx]);
	}
	else
	{
		for(uint8_t i = 0; i < BUFF_COUNT; i++)
		{
			if(FULL == buff->buff[i].status)
			{
				buff->buff[i].status = IN_READ;
				buff->in_read_idx = i;
				break;
			}
		}
	}
}
