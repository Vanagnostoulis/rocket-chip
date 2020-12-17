#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "encoding.h"
#include "example.h"
/*
LOAD command is used in order to load one matrix into accelerator's internal memory 
COMPUTE command is used after loading 2 matrixes into memory. It calculates the multiplication of 2 matrixes
STORE command is used in order to save the results back to L1 cache (save responding )
*/
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
  array[0] = 2111;
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

  asm volatile ("fence");
  ROCC_INSTRUCTION_DS(ACCELERATOR_OPCODE, res, (uintptr_t) &array[0], STORE2);
  asm volatile ("fence");
  printf("INFO *****res %ld with array %p \n",res , &array[0]);

  asm volatile ("fence");
  ROCC_INSTRUCTION_DS(ACCELERATOR_OPCODE, res, (uintptr_t) &array[1], STORE2);
  asm volatile ("fence");
  printf("INFO *****res %ld with array %p \n",res ,  &array[1]);
  

  asm volatile ("fence");
  ROCC_INSTRUCTION_DS(ACCELERATOR_OPCODE, res, (uintptr_t) &array[2], STORE2);
  asm volatile ("fence");
  printf("INFO *****res %ld with array %p \n",res ,  &array[2]);
  
  asm volatile ("fence");
  ROCC_INSTRUCTION_DS(ACCELERATOR_OPCODE, res, (uintptr_t) &array[3], STORE2);
  asm volatile ("fence");
  printf("INFO *****res %ld with array %p \n",res ,  &array[3]);
  
  asm volatile ("fence");
  ROCC_INSTRUCTION_DS(ACCELERATOR_OPCODE, res, (uintptr_t) &array[8], STORE2);
  asm volatile ("fence");
  printf("INFO *****res %ld with array %p \n",res ,  &array[8]);

  asm volatile ("fence");
  ROCC_INSTRUCTION_DS(ACCELERATOR_OPCODE, res, (uintptr_t) &array[8], STORE2);
  asm volatile ("fence");
  printf("INFO *****res %ld with array %p \n",res ,  &array[8]);

  printf("INFO BEFORE LOAD array: %p end: %p 0:%ld 1:%ld 2: %ld 3: %ld 4: %ld ... 14: %ld\n", array, end, array[0],array[1],array[2],array[3],array[4],array[14]);
  asm volatile ("fence");
  ROCC_INSTRUCTION_DSS(ACCELERATOR_OPCODE, res, (uintptr_t) array, (uintptr_t) end, LOAD);
  asm volatile ("fence");
  printf("INFO AFTER LOAD array: %p end: %p 0:%ld 1:%ld 2: %ld 3: %ld 4: %ld ... 14: %ld\n", array, end, array[0],array[1],array[2],array[3],array[4],array[14]);

  
  return 0;


}

