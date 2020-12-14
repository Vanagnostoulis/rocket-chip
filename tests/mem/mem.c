#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "encoding.h"
#include "example.h"

#define STORE  0
#define LOAD   1
#define ACCELERATOR_OPCODE 0

int main()
{
  // WHAT THIS EXAMPLE DOES:
  // STORE THE VALUE OF RS2 INTO RS1 USING RS1's MEMORY ADDRESS
  // THEN LOAD FROM MEMORY THE VALUE OF RS1 AND PUT IT INTO RESULT1
  
  // memory translation is done by ROCC's memory interface by defining  
  // io.mem.req.bits.phys  := false.B
  uint64_t rs1     =0x5;
  uint64_t rs2     =0x3;
  uint64_t result1 =0x0;

  asm volatile ("fence");
  printf("\n BEFORE STORE rs1 %lx addr %p  rs2  %lx addr %p result1 %lx addr %p\n",rs1, (void *) &rs1, rs2, (void *) &rs2, result1, (void *) &result1);
  ROCC_INSTRUCTION_SS(ACCELERATOR_OPCODE, (uintptr_t) &rs1, rs2, STORE);
  printf("\n AFTER  STORE rs1 %lx addr %p  rs2  %lx addr %p result1 %lx addr %p\n",rs1, (void *) &rs1, rs2, (void *) &rs2, result1, (void *) &result1);
  if (rs1 != rs2 )
    return 1;
  
  printf("\n BEFORE LOAD rs1 %lx addr %p  rs2  %lx addr %p result1 %lx addr %p\n",rs1, (void *) &rs1, rs2, (void *) &rs2, result1, (void *) &result1);
  ROCC_INSTRUCTION_DS(ACCELERATOR_OPCODE, result1, (uintptr_t) &rs1, LOAD);
  printf("\n AFTER  LOAD rs1 %lx addr %p  rs2  %lx addr %p result1 %lx addr %p\n",rs1, (void *) &rs1, rs2, (void *) &rs2, result1, (void *) &result1);
  
  if (rs1 != result1 )
    return 2;

  return 0;
}

