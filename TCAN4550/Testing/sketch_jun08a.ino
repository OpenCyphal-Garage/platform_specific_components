#include <stdio.h>
#include <Arduino.h>
#include "ti_can.h"
#include <SPI.h>

extern "C" void debug_print(const char* format, ...) {
    char buffer[128]; // Adjust size as needed
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.println(buffer);
}

extern "C" uint8_t spiRegisterWrite(uint32_t addr, uint32_t regValue) {
    // Begin SPI transaction
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    // Select your SPI slave device, if necessary
    digitalWrite(SS, LOW);

    // Send address and register value
    SPI.transfer(addr >> 8);  // Send high byte of address
    SPI.transfer(addr & 0xFF); // Send low byte of address
    SPI.transfer(regValue >> 24); // Send high byte of register value
    SPI.transfer((regValue >> 16) & 0xFF); // Send second byte of register value
    SPI.transfer((regValue >> 8) & 0xFF); // Send third byte of register value
    SPI.transfer(regValue & 0xFF); // Send low byte of register value

    // Deselect your SPI slave device, if necessary
    digitalWrite(SS, HIGH);

    // End SPI transaction
    SPI.endTransaction();

    return 0;
}

extern "C" uint32_t spiRegisterRead(uint32_t addr) {
    uint32_t regValue = 0;

    // Begin SPI transaction
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    // Select your SPI slave device, if necessary
    digitalWrite(SS, LOW);

    // Send address to read from
    SPI.transfer(addr >> 8);  // Send high byte of address
    SPI.transfer(addr & 0xFF); // Send low byte of address

    // Receive register value
    regValue |= ((uint32_t)SPI.transfer(0) << 24); // Receive high byte of register value
    regValue |= ((uint32_t)SPI.transfer(0) << 16); // Receive second byte of register value
    regValue |= ((uint32_t)SPI.transfer(0) << 8);  // Receive third byte of register value
    regValue |= SPI.transfer(0); // Receive low byte of register value

    // Deselect your SPI slave device, if necessary
    digitalWrite(SS, HIGH);

    // End SPI transaction
    SPI.endTransaction();

    return regValue;
}



int main(void)
{
  init();
  Serial.begin(9600);
  while(!Serial);

  // Test of lib START

  BitTimingParams  * bTParams = NULL;
  TiMRAMParams     * MRAM = NULL;

  bTParams -> prescaler       = 3;
  bTParams -> prop_and_phase1 = 6;
  bTParams -> phase2          = 1;
  bTParams -> sync_jump_width = bTParams -> phase2;
  //btParams -> tdc             = ;

  

  // Test of lib END



  delay(1000);  
  return 0;  
}}
