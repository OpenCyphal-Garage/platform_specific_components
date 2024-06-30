///
/// Author: Daniil Bragin
/// Reference: 

#include "ti_can.h"
#include "ti_can_registers.h"
#include <assert.h>
#include <string.h>


void spi_init() 
{

/**
 * TODO: Implement SPI init 
*/

}

uint8_t spiRegisterWrite(uint32_t addr, 
                         uint32_t regValue)
{

    /**
     * 
     *  TODO: Implement SPI write operation based on the host MCU
     * 
    */

   //

    return 0;
}

uint32_t spiRegisterRead(uint32_t addr)
{   
    uint32_t regValue = 0;

    /**
     * 
     *  TODO: Implement SPI read operation based on the host MCU
     * 
    */

    return regValue;
}

uint8_t initCAN (const BitTimingParams  * bTParams, 
                 const TiMRAMParams     * MRAM)
{

    // TODO: Cover edge cases for values in structs + error reporting

    // spi_init();

    // Standby Mode Check
    uint32_t mode = 0;

    mode = spiRegisterRead(MODE_SEL);

    // Standby Mode Check
    if ((mode & 0xC0) != STANDBY_MODE)
    {   
        printf("Device not in STANDBY_MODE, MODE: %lu, setting STANDBY_MODE\n", mode & 0xC0);
        
        mode &= ~CLEAN_MODE;
        mode |= STANDBY_MODE;
        
        spiRegisterWrite(MODE_SEL, mode);
        printf("STANDBY_MODE set successfully\n");

        mode = spiRegisterRead(MODE_SEL);
        printf("New device mode: %lX, NEEDED: %X\n", mode & 0xC0, STANDBY_MODE);
    }
    else
    {
        printf("Device in STANDBY_MODE, mode: %lX\n", mode);
    }

    // Initialization Mode
    // TODO: Check whether CSR needs to be written before CCE and INIT
    
    uint32_t init = spiRegisterRead(CCCR);

    printf("Initial CCCR content: %lX\n", init);

    init &= ~(CAN_CCCR_CCE | CAN_CCCR_INIT);
    init |= (CAN_CCCR_CCE | CAN_CCCR_INIT);
    init &= ~CAN_CCCR_CSR;

    
    uint32_t bit_timing = 0;
    uint32_t trans_delay_comp = 0;

    uint8_t prescaler       =   bTParams -> prescaler;
    uint8_t prop_and_phase1 =   bTParams -> prop_and_phase1;
    uint8_t phase2          =   bTParams -> phase2;
    uint8_t sync_jump_width =   bTParams -> sync_jump_width;
    uint8_t tdc             =   bTParams -> tdc;


    // FD/BRS & Bit Timing Init Cofiguration
    if (CAN_MODE == 0)
    {
        printf("Device in CAN mode\n");
        spiRegisterWrite(CCCR, init);
    
        bit_timing = spiRegisterRead(NBTP); 

        printf("Initial NBTP content: %lu\n", bit_timing); 

        // Reset the NBTP register values
        bit_timing &= ~(CAN_NBTP_NSJW_MASK      | 
                        CAN_NBTP_NTSEG1_MASK    | 
                        CAN_NBTP_NTSEG2_MASK    | 
                        CAN_NBTP_NBRP_MASK);

        // Set the NBTP register values based on the provided settings
        bit_timing |= CAN_SYNC_JUMP_WIDTH(sync_jump_width);
        bit_timing |= CAN_TIME_SEG_1(prop_and_phase1);
        bit_timing |= CAN_TIME_SEG_2(phase2);
        bit_timing |= CAN_PRESCALER(prescaler);

        printf("Intended NBTP value: %lX\n", bit_timing);

        spiRegisterWrite(NBTP, bit_timing);

    }
    else if (CAN_MODE == 1)
    {
        // Check BRS and FD settings
        if (BIT_RATE_SWITCH)    init |= CCCR_BRSE;
        if (FD)     init |= CCCR_FDOE;

        spiRegisterWrite(CCCR, init);

        bit_timing       =  spiRegisterRead(DBTP);
        trans_delay_comp =  spiRegisterRead(TDCR);
        
        // Reset the DBTP register values
        bit_timing &= ~(CANFD_DBTP_DSJW_MASK      | 
                        CANFD_DBTP_DTSEG1_MASK    | 
                        CANFD_DBTP_DTSEG2_MASK    | 
                        CANFD_DBTP_DBRP_MASK);
        
        trans_delay_comp &= ~CANFD_TDCR_TDCO_MASK;
        
        // Set the DBTP register values based on the provided settings
        bit_timing |= CANFD_SYNC_JUMP_WIDTH(sync_jump_width);
        bit_timing |= CANFD_TIME_SEG_1_WIDTH(prop_and_phase1);
        bit_timing |= CANFD_TIME_SEG_2_WIDTH(phase2);
        bit_timing |= CANFD_PRESCALER(prescaler);

        trans_delay_comp |= CANFD_DELAY_COMPENSATION_OFFSET(tdc);

        spiRegisterWrite(DBTP, bit_timing);

        spiRegisterWrite(TDCR, trans_delay_comp);
    }

    // Zero out MRAM

    spiRegisterWrite(SIDFC, 0x0);
    spiRegisterWrite(XIDFC, 0x0);
    spiRegisterWrite(RXF0C, 0x0);
    spiRegisterWrite(RXF1C, 0x0);
    spiRegisterWrite(RXBC, 0x0);
    spiRegisterWrite(RXESC, 0x0);
    spiRegisterWrite(TXEFC, 0x0);
    spiRegisterWrite(TXBC, 0x0);
    spiRegisterWrite(TXESC, 0x0);
    
    // MRAM Init

    uint32_t sid        = spiRegisterRead(SIDFC);
    uint32_t xid        = spiRegisterRead(XIDFC);
    uint32_t rxf0       = spiRegisterRead(RXF0C);
    uint32_t rxf1       = spiRegisterRead(RXF1C);
    uint32_t rxb        = spiRegisterRead(RXBC);
    uint32_t rx         = spiRegisterRead(RXESC);
    uint32_t tx_fifo    = spiRegisterRead(TXEFC);
    uint32_t txb        = spiRegisterRead(TXBC);
    uint32_t tx         = spiRegisterRead(TXESC);

    sid &= ~(SID_LSS_MASK |
             SID_FLSS_MASK);

    sid |= SID_LSS(MRAM -> SID_LSS);
    sid |= SID_FLSS(MRAM -> SID_FLSS);

    
    xid &= ~(XID_LSE_MASK |
             XID_FLSEA_MASK);

    xid |= XID_LSE(MRAM -> XID_LSE);
    xid |= XID_FLSEA(MRAM -> XID_FLSEA);
    
    
    rxf0 &= ~(RXF0_F0OM_MASK    |
              RXF0_F0WM_MASK    |
              RXF0_F0S_MASK     |
              RXF0_F0SA_MASK);    

    rxf0 |= RXF0_F0OM(MRAM -> RXF0_F0OM);
    rxf0 |= RXF0_F0WM(MRAM -> RXF0_F0WM);
    rxf0 |= RXF0_F0S(MRAM -> RXF0_F0S);
    rxf0 |= RXF0_F0SA(MRAM -> RXF0_F0SA);


    rxf1 &= ~(RXF1_F1OM_MASK    |
              RXF1_F1WM_MASK    |
              RXF1_F1S_MASK     |
              RXF1_F1SA_MASK);

    rxf1 |= RXF1_F1OM(MRAM -> RXF1_F1OM);
    rxf1 |= RXF1_F1WM(MRAM -> RXF1_F1WM);
    rxf1 |= RXF1_F1S(MRAM -> RXF1_F1S);
    rxf1 |= RXF1_F1SA(MRAM -> RXF1_F1SA);


    rxb &= ~(RXB_RBSA_MASK);

    rxb |= RXB_RBSA(MRAM -> RXB_RBSA);


    rx &= ~(RX_RBDS_MASK    |
            RX_F1DS_MASK    |
            RX_F0DS_MASK);
    
    rx |= RX_RBDS(MRAM -> RX_RBDS);
    rx |= RX_F1DS(MRAM -> RX_F1DS);
    rx |= RX_F0DS(MRAM -> RX_F0DS);


    tx_fifo &= ~(TXEVF_EFWM_MASK    |
                 TXEVF_EFS_MASK     |
                 TXEVF_EFSA_MASK);

    tx_fifo |= TXEVF_EFWM(MRAM -> TXEVF_EFWM);
    tx_fifo |= TXEVF_EFS(MRAM -> TXEVF_EFS);
    tx_fifo |= TXEVF_EFSA(MRAM -> TXEVF_EFSA);


    txb &= ~(TXB_TFQM_MASK  |
             TXB_TFQS_MASK  |
             TXB_NDTB_MASK  |
             TXB_TBSA_MASK);
    
    txb |= TXB_TFQM(MRAM -> TXB_TFQM);
    txb |= TXB_TFQS(MRAM -> TXB_TFQS);
    txb |= TXB_NDTB(MRAM -> TXB_NDTB);
    txb |= TXB_TBSA(MRAM -> TXB_TBSA);


    tx &= ~(TX_TBDS_MASK);

    tx |= TX_TBDS(MRAM -> TX_TBDS);


    spiRegisterWrite(SIDFC, sid);
    spiRegisterWrite(XIDFC, xid);
    spiRegisterWrite(RXF0C, rxf0);
    spiRegisterWrite(RXF1C, rxf1);
    spiRegisterWrite(RXBC, rxb);
    spiRegisterWrite(RXESC, rx);
    spiRegisterWrite(TXEFC, tx_fifo);
    spiRegisterWrite(TXBC, txb);
    spiRegisterWrite(TXESC, tx);


    uint32_t selected_mode = spiRegisterRead(MODE_SEL);

    printf("Initial device mode: %lX\n", selected_mode);

    selected_mode &= ~CLEAN_MODE;
    selected_mode |= NORMAL_MODE;

    printf("Intended mode: %lX\n", selected_mode);

    spiRegisterWrite(MODE_SEL, selected_mode);
    printf("Device in NORMAL_MODE, init_can successful\n");

    return 0;
}

uint8_t setSIDFilters(SID_filter * filters, TiMRAMParams * MRAM) 
{
    size_t size = (size_t) MRAM -> SID_LSS;
    uint32_t * filter_addr = (uint32_t *)MRAM -> SID_FLSS;
    uint32_t filter = 0;

    for (size_t i = 0; i < size; i++)
    {
        filter = 0;
        
        filter |= SID_SFT   (filters[i].SFT);
        filter |= SID_SFEC  (filters[i].SFEC);
        filter |= SID_SFID1 (filters[i].SFID_1);
        filter |= SID_SFID2 (filters[i].SFID_2);

        spiRegisterWrite(filter_addr + i * sizeof(uint32_t), filter);
    }

    return 0;
}

uint8_t setXIDFilters(XID_filter * filters, TiMRAMParams * MRAM)
{
    size_t   size           = (size_t) MRAM -> XID_LSE;
    uint32_t filter_addr    = MRAM -> XID_FLSEA;
    uint64_t filter         = 0; // Two words needed for XID
    uint32_t filter_1       = 0;
    uint32_t filter_2       = 0;

    for (size_t i = 0; i < size; i++)
    {
        filter      = 0;
        filter_1    = 0;
        filter_2    = 0;
        
        filter_1 |= XID_EFID2(filters[i].EFID1);
        filter_1 |= XID_EFT(filters[i].EFT);

        filter_2 |= XID_EFID2(filters[i].EFID2);
        filter_2 |= XID_EFEC(filters[i].EFEC);

        filter |= (filter_1 | filter_2);

        spiRegisterWrite(filter_addr + i, filter);
    }

    return 0;
}

uint8_t sendCAN(TiMRAMParams * MRAM, TXFIFOElement * TXE)
{
    // Check that any TX Buffer is vacant
    uint32_t free_level = spiRegisterRead(TXFQS);

    uint32_t tx_buffer_start_addr = MRAM -> TXB_TBSA;
    uint32_t tx_buffer_element_size = MRAM -> TX_TBDS + 8; // Additional 8 bytes of header


    
    if (!(TFFL(free_level)))
    {
        printf("ERROR: No TX BUffers Available, return 1\n");
        return 1;
    }

    uint32_t index = TFQPI(free_level);

    uint32_t memory_offset = tx_buffer_start_addr + (tx_buffer_element_size * index);


    uint32_t word_0 = 0;
    uint32_t word_1 = 0;
    uint32_t word_2 = 0;

    if (TXE -> ESI) word_0 |= (1 << ESI_SHFT);
    if (TXE -> RTR) word_0 |= (1 << RTR_SHFT);
    if (TXE -> XTD)
    {
        word_0 |= (1 << XTD_SHFT);
        word_0 |= ((TXE -> ID) << XID_SHFT);
    } 
    else
    {
        word_0 |= ((TXE -> ID) << SID_SHFT);
    }

    spiRegisterWrite(memory_offset, word_0);

    word_1 |= ((TXE -> MM) << MM_SHFT);

    if (TXE -> EFC) word_1 |= (1 << EFC_SHFT);
    if (TXE -> FDF) word_1 |= (1 << FDF_SHFT);
    if (TXE -> BRS) word_1 |= (1 << BRS_SHFT);
    
    word_1 |= ((TXE -> DLC) << DLC_SHFT);

    spiRegisterWrite(memory_offset + sizeof(word_0), word_1);

    word_2 |= ( ((TXE -> data_byte_0) << DB0_SHFT)   |
                ((TXE -> data_byte_1) << DB1_SHFT)   |
                ((TXE -> data_byte_2) << DB2_SHFT)   |
                ((TXE -> data_byte_3) << DB3_SHFT)   );
    
    spiRegisterWrite(memory_offset + sizeof(word_0) + sizeof(word_1), word_2);

    uint32_t add_request = 0;

    add_request |= (1U << index);

    spiRegisterWrite(TXBUF_AR, add_request);

    return 0;
}

