///
/// Author: Daniil Bragin
///

#include "ti_can.h"
#include "ti_can_registers.h"
#include <assert.h>
#include <string.h>


void spiRegisterWrite(uint16_t addr,
                      uint32_t regValue)
{

}

void initCAN (const BitTimingParams bTParams)
{

    // Bit Timing Init
    if (CAN_MODE == 0)
    {
        
    }
    else if (CAN_MODE == 1)
    {
        
    }

    // MRAM Init
    
    // Initialization Mode
    uint8_t init = CAN_CCCR_CCE | CAN_CCCR_INIT;
    spiRegisterWrite(CCCR, (uint32_t)init);

    // SID Filters
    

}


