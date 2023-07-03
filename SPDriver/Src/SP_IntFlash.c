/*
 * SP_IntFlash.c
 *
 *  Created on: Jul 3, 2023
 *      Author: soren
 */
#include "SP_IntFlash.h"


#define INT_FLASH_PAGE_SIZE   4096
#define INT_FLASH_NUM_PAGES   256
#define INT_FLASH_START_ADDRESS  (uint32_t*)0x08030000
#define INT_FLASH_END_ADDRESS  (uint32_t*)0x080FFFFF

#define INT_FLASH_UNLOCK_KEY1  0x45670123
#define INT_FLASH_UNLOCK_KEY2  0xCDEF89AB
static volatile uint32_t* FLASH_PTR = INT_FLASH_START_ADDRESS;

/* The erase operation can be performed at page level (page erase), or on the whole memory
(mass erase). Mass erase does not affect the Information block (system flash, OTP and
option bytes). */

/*
 * The flash memory programming sequence in standard mode is as follows:
1. Check that no flash main memory operation is ongoing by checking the BSY bit in the
Flash memory status register (FLASH_SR) or Flash memory CPU2 status register
(FLASH_C2SR).
Check that flash memory program and erase operations are allowed by checking the
PESD bit in the Flash memory status register (FLASH_SR) or Flash memory CPU2
status register (FLASH_C2SR) (these checks are recommended even if status may
change due to flash memory operation requests by the other CPU, to limit the risk of
receiving a bus error when starting programming).

2. Check and clear all error programming flags due to a previous programming. If not,
PGSERR is set.

3. Check that bit CFGBSY in the FLASH_xxSR register is cleared.

4. Set the PG bit in the Flash memory control register (FLASH_CR) or Flash memory
CPU2 control register (FLASH_C2CR).

5. Perform the data write operation at the desired memory address, inside main memory
block or OTP area. Only double word (64 bits) can be programmed.
a) Write a first word in an address aligned with double word
b) Write the second word.

6. Wait until bit CFGBSY is cleared.

7. Check that EOP flag is set in the FLASH_xxSR register (meaning that the
programming operation has succeeded), and clear it by software.

8. Clear the PG bit in the FLASH_xxSR register if there are no more programming
requests anymore.
 */


bool_t drv_IntFlash_Write(void)
{
  bool_t bStatus = FALSE;
  if(!drv_IntFlash_checkBusy() && drv_IntFlash_checkOperationsAllowed() )
  {
    if(!(FLASH->SR & FLASH_SR_PGSERR)) /* No current errors */
    {
      if(!(FLASH->SR & FLASH_SR_CFGBSY))
      {
	  // Ensure double-word alignment
	if (((uint32_t)FLASH_PTR & 0x07) == 0)
	{

	  drv_IntFlash_Unlock();
	  FLASH->CR |= 1 << 0;
	  // Perform double-word programming
	  *(__IO uint64_t*)FLASH_PTR = 0xDEADBEEFCAFEBABE;

	  if(FLASH->SR & FLASH_SR_OPTVERR)
	  {
	    FLASH->SR |= FLASH_SR_OPTVERR;
	  }

	  FLASH->CR |= FLASH_CR_LOCK;
	}

      }
    }
  }

  while(FLASH->SR & FLASH_SR_CFGBSY)
  {
    // Wait for CFGBSY flag to clear
  }

  if(FLASH->SR & FLASH_SR_EOP)
  {
    FLASH->SR &= ~(FLASH_SR_EOP);
    FLASH->C2CR &= ~(FLASH_C2CR_PG);
    bStatus = TRUE;
  }

  return bStatus;
}


void drv_IntFlash_Unlock(void)
{
  FLASH->KEYR |= (INT_FLASH_UNLOCK_KEY1 );
  FLASH->KEYR |= (INT_FLASH_UNLOCK_KEY2);
}


bool_t drv_IntFlash_checkOperationsAllowed(void)
{
  bool_t bStatus = TRUE;
  if(FLASH->SR & FLASH_SR_PESD)
  {
      bStatus = FALSE;
  }

  return bStatus;
}


/* This bit is set at the beginning of a flash memory operation, and
reset when the operation finishes or when an error occurs. */
bool_t drv_IntFlash_checkBusy(void)
{
  bool_t bStatus = TRUE;
  if(!(FLASH->SR & FLASH_SR_BSY))
  {
      bStatus = FALSE;
  }

  return bStatus;
}
