///
/// Author: Daniil Bragin
///

#include "ti_can.h"
#include "ti_can_registers.h"
#include <assert.h>
#include <string.h>


uint8_t spiRegisterWrite(uint16_t addr,
                      uint32_t regValue)
{

    /**
     * 
     *  TODO: Implement SPI write opration based on the host MCU
     * 
    */

    return 0;
}

uint32_t spiRegisterRead(uint16_t addr)
{   
    uint32_t regValue = 0;

    /**
     * 
     *  TODO: Implement SPI read opration based on the host MCU
     * 
    */

    return regValue;
}

uint8_t initCAN (const BitTimingParams bTParams)
{
    // Standby Mode Check
    if ((spiRegisterRead(MODE_SEL)) != STANDBY_MODE)
    {
        spiRegisterWrite(MODE_SEL, STANDBY_MODE);
    }

    // Initialization Mode
    // TODO: Check whether CSR needs to be written before CCE and INIT
    uint32_t init;
    init |= (CAN_CCCR_CCE | CAN_CCCR_INIT);
    init &= ~CAN_CCCR_CSR; 
    
    // FD/BRS & Bit Timing Init Cofiguration
    if (CAN_MODE == 0)
    {
        init |= 

        spiRegisterWrite(CCCR, init);    
    }
    else if (CAN_MODE == 1)
    {
        if (BRS)    init |= CCCR_BRSE;
        if (FD)     init |= CCCR_FDOE;



        spiRegisterWrite(CCCR, init);
    }

    // MRAM Init
    
    

    // SID Filters
    
    // Put the TCAN45xx device into "NORMAL" mode
    spiRegisterWrite(MODE_SEL, NORMAL_MODE);


    return 0;
}


