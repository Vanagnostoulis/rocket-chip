#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "encoding.h"
#include "example.h"

#define LOAD     0
#define STORE1   1
#define STORE2   2
#define COMPUTE  3
#define ACCELERATOR_OPCODE 0

int main()
{
  uint64_t *array = (uint64_t *)calloc(15 , sizeof(uint64_t)); 
  uint64_t *end   = array + 14;
  for (int i = 0; i < 15; ++i)
  {
    array[i] = i;
  }
  uint64_t res = 0;
  printf("INFO BEFORE LOAD array: %p end: %p 0:%ld 1:%ld 2: %ld 3: %ld 4: %ld ... 14: %ld\n", array, end, array[0],array[1],array[2],array[3],array[4],array[14]);
  asm volatile ("fence");
  ROCC_INSTRUCTION_DSS(ACCELERATOR_OPCODE, res, (uintptr_t) array, (uintptr_t) end, LOAD);
  asm volatile ("fence");
  printf("INFO AFTER LOAD array: %p end: %p 0:%ld 1:%ld 2: %ld 3: %ld 4: %ld ... 14: %ld\n", array, end, array[0],array[1],array[2],array[3],array[4],array[14]);

printf("INFO BEFORE STORE LOAD array: %p end: %p 0:%ld 1:%ld 2: %ld 3: %ld 4: %ld ... 14: %ld\n", array, end, array[0],array[1],array[2],array[3],array[4],array[14]);
  asm volatile ("fence");
  ROCC_INSTRUCTION_DSS(ACCELERATOR_OPCODE, res, (uintptr_t) array, (uintptr_t) end, STORE1);
  asm volatile ("fence");
  printf("INFO AFTER STORE array: %p end: %p 0:%ld 1:%ld 2: %ld 3: %ld 4: %ld ... 14: %ld\n", array, end, array[0],array[1],array[2],array[3],array[4],array[14]);
  return 0;


}

