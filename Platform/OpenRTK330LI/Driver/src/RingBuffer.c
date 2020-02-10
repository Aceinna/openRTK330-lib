// RingBuffer.c needed by park_algo_jack.c  writed by jacksun 2017.11.12
#include "RingBuffer.h"
#include <string.h>


void FifoInit(FIFO_Type* fifo, uint8_t* buffer, uint16_t size)
{
	fifo->buffer = buffer;
	fifo->in = 0;
	fifo->out = 0;
	fifo->size = size;
}

void FifoPush(FIFO_Type* fifo, uint8_t* buffer, uint16_t size)
{
	uint16_t i;

    for (i = 0; i < size; i++)
    {
        fifo->buffer[(fifo->in+i)%fifo->size] = buffer[i];
    }
    fifo->in = (fifo->in + size)%fifo->size;
}

uint16_t FifoGet(FIFO_Type* fifo, uint8_t* buffer, uint16_t len)
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

uint16_t FifoStatus(FIFO_Type* fifo)
{
	uint16_t lenght;
	lenght = (fifo->in + fifo->size - fifo->out)%fifo->size;
	return lenght;
}

  

int InitRingBuffer(RingBuffer *pRing,ELEMENT_TYPE * buff,u16 len)
{
    memset(pRing->pRing_buf,0,sizeof(ELEMENT_TYPE) * pRing->length);
    pRing->pRing_buf    =buff;
    pRing->write_index  =0;
    pRing->read_index   =0;
    pRing->length       =len;
    return 0;
}
int testdata=0;
int ReadRingBuffer(RingBuffer *pRing,ELEMENT_TYPE *pReadbuf,u16 rd_len)
{
    if(rd_len > pRing->length)
        return -1;
    
    
    if((pRing->read_index+rd_len) >= pRing->length)
    {
        memcpy(pReadbuf,&pRing->pRing_buf[pRing->read_index],sizeof(ELEMENT_TYPE) * (pRing->length - pRing->read_index));
        memcpy(&pReadbuf[pRing->length - pRing->read_index],&pRing->pRing_buf[0],sizeof(ELEMENT_TYPE) * (rd_len - (pRing->length - pRing->read_index)) );
        
        pRing->read_index = ((pRing->read_index+rd_len )- pRing->length)%pRing->length ;
    }
    else
    {
        memcpy(pReadbuf,&pRing->pRing_buf[pRing->read_index],sizeof(ELEMENT_TYPE) * rd_len);
        pRing->read_index= (pRing->read_index+rd_len)%pRing->length;
    }
    return 0;
}
int ReadRecentDataInRingBuffer(RingBuffer *pRing,ELEMENT_TYPE *pReadbuf,u16 rd_len)
{

     int num,rd_tail_num=0;

     if(rd_len > pRing->length)
        return -1;
     if(rd_len <= pRing->write_index)
     {
         memcpy(pReadbuf,&pRing->pRing_buf[pRing->write_index-rd_len],sizeof(ELEMENT_TYPE) *rd_len);
     }
     else
     {
        rd_tail_num=rd_len-pRing->write_index;
         
        memcpy(pReadbuf,&pRing->pRing_buf[pRing->length-rd_tail_num],sizeof(ELEMENT_TYPE) * rd_tail_num);
        memcpy(&pReadbuf[rd_tail_num],&pRing->pRing_buf[0],sizeof(ELEMENT_TYPE) * pRing->write_index);         
     }
    
      num = rd_len;

    return num;
}

int ReadAllDataNoRead(RingBuffer *pRing,ELEMENT_TYPE *pReadbuf)
{
    int num;

    if(pRing->write_index==pRing->read_index)
    {
        return 0;
    }
    
    if(pRing->write_index>pRing->read_index)
    {
        memcpy(pReadbuf,&pRing->pRing_buf[pRing->read_index],sizeof(ELEMENT_TYPE) * (pRing->write_index-pRing->read_index));
        num = pRing->write_index-pRing->read_index;
        if(num==1200)
        {
            testdata=1;

        }


    }
    else
    {
        memcpy(pReadbuf,&pRing->pRing_buf[pRing->read_index],sizeof(ELEMENT_TYPE) * (pRing->length - pRing->read_index));
        memcpy(&pReadbuf[pRing->length - pRing->read_index],&pRing->pRing_buf[0],sizeof(ELEMENT_TYPE) * pRing->write_index);
        num = pRing->write_index+( pRing->length - pRing->read_index);
        if(num==1200)
        {
            testdata=2;
        }
    }
    pRing->read_index = pRing->write_index;

    return num;
}

int ReadAllDataNoReadLength(RingBuffer *pRing)
{
    int num;
    
    if(pRing->write_index>=pRing->read_index)
    {
        num = pRing->write_index-pRing->read_index;
    }
    else
    {
        num = pRing->write_index+( pRing->length - pRing->read_index);
    }

    return num;
}


int WriteOneElementRingBuffer(RingBuffer *pRing,ELEMENT_TYPE element)
{
 
  if(pRing->write_index<pRing->length)    
    pRing->pRing_buf[pRing->write_index++]=element;
  else
     pRing->write_index=0; 
    
    return 0;
}

u16 GetLastWriteIndex(RingBuffer *pRing)
{
   u16 index=0;
  if(pRing->write_index==0)    
      index=pRing->length-1;
  else
     index=pRing->write_index-1; 
    
    return index;
    
}


int WriteRingBuffer(RingBuffer *pRing,ELEMENT_TYPE *pWrbuf,u16 wr_len)
{
    if(wr_len>pRing->length)
    {
        return -1;

    }
    if(wr_len==0)
    {
        return 0;
    }
 
    if((pRing->write_index + wr_len) > pRing->length)
    {
       memcpy(&pRing->pRing_buf[pRing->write_index], pWrbuf, sizeof(ELEMENT_TYPE) *(pRing->length - pRing->write_index));
       memcpy(pRing->pRing_buf,&pWrbuf[ pRing->length - pRing->write_index],sizeof(ELEMENT_TYPE) *(wr_len-(pRing->length - pRing->write_index)));
       pRing->write_index=((pRing->write_index + wr_len) -pRing->length)%pRing->length;
    }
    else
    {
       memcpy(&pRing->pRing_buf[pRing->write_index], pWrbuf, sizeof(ELEMENT_TYPE) *wr_len); 
       pRing->write_index = (pRing->write_index + wr_len)%pRing->length;
    }   
    
    return wr_len;
}
