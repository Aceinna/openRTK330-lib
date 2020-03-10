
#include <string.h>

#include "utils.h"


void fifo_init(fifo_type* fifo, uint8_t* buffer, uint16_t size)
{
	fifo->buffer = buffer;
	fifo->in = 0;
	fifo->out = 0;
	fifo->size = size;
}

void fifo_push(fifo_type* fifo, uint8_t* buffer, uint16_t size)
{
	uint16_t i;

    for (i = 0; i < size; i++)
    {
        fifo->buffer[(fifo->in+i)%fifo->size] = buffer[i];
    }
    fifo->in = (fifo->in + size)%fifo->size;
}

uint16_t fifo_get(fifo_type* fifo, uint8_t* buffer, uint16_t len)
{
	uint16_t lenght;
	uint16_t in = fifo->in;	
	uint16_t i;
	lenght = (in + fifo->size - fifo->out)%fifo->size;
	if(lenght > len)
		lenght = len;
	for(i = 0; i < lenght; i++)
	{
		buffer[i] = fifo->buffer[(fifo->out + i)%fifo->size];
	}
	fifo->out = (fifo->out + lenght)%fifo->size;
	return lenght;
}

uint16_t fifo_status(fifo_type* fifo)
{
	uint16_t lenght;
	lenght = (fifo->in + fifo->size - fifo->out)%fifo->size;
	return lenght;
}