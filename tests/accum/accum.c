#include "rocc.h"
#include <stdio.h>

static inline void accum_write(int idx, unsigned long data)
{
	ROCC_INSTRUCTION_SS(2, data, idx, 0);
}

static inline unsigned long accum_read(int idx)
{
	unsigned long value;
	ROCC_INSTRUCTION_DSS(2, value, 0, idx, 1);
	return value;
}

static inline void accum_load(int idx, void *ptr)
{
	asm volatile ("fence");
	ROCC_INSTRUCTION_SS(2, (uintptr_t) ptr, idx, 2);
}

static inline void accum_add(int idx, unsigned long addend)
{
	ROCC_INSTRUCTION_SS(2, addend, idx, 3);
}

unsigned long data = 0x3421L;

int main(void)
{
	unsigned long result;
	printf("before load data %lx &data %p\n",data,(void *) &data);
	accum_load(0, &data);
	printf("after load data %lx &data %p\n",data,(void *) &data);
	accum_add(0, 2);
	printf("after add data %lx &data %p\n",data,(void *) &data);
	result = accum_read(0);
	printf("after read result  %lx \n",result);
	printf("should be result %lx == data %lx +2 \n",result, data);
	printf("before write %lx &data %p\n",data, (void *) &data);
	accum_write(0, 3);
	printf("after write data %lx &data %p\n",data,(void *) &data);
	accum_add(0, 1);
	printf("after add data %lx &data %p\n",data,(void *) &data);

	result = accum_read(0);
	printf("after read result  %lx \n",result);
	printf("should be result %lx == 4  \n",result);
	/*
		before load data 3421 &data 0x1caa0
		after load data 3421 &data 0x1caa0
		after add data 3421 &data 0x1caa0
		after read result  3423 
		should be result 3423 == data 3421 +2 
		before write 3421 &data 0x1caa0
		after write data 3421 &data 0x1caa0
		after add data 3421 &data 0x1caa0
		after read result  4 
		should be result 4 == 4 
	*/
	if (result != 4)
		return 2;

	return 0;
}
