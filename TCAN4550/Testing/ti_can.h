///                            ____                   ______            __          __
///                           / __ `____  ___  ____  / ____/_  ______  / /_  ____  / /
///                          / / / / __ `/ _ `/ __ `/ /   / / / / __ `/ __ `/ __ `/ /
///                         / /_/ / /_/ /  __/ / / / /___/ /_/ / /_/ / / / / /_/ / /
///                         `____/ .___/`___/_/ /_/`____/`__, / .___/_/ /_/`__,_/_/
///                             /_/                     /____/_/
///
/// A low-level driver with the main purpose of helping new hardware revisions or new designs adopt CAN FD
/// without having to port an existing codebase to a new platform.
///
/// Author: Daniil Bragin
///


#pragma once

#include <Arduino.h> // For testing purposes
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////////////////






///////////////////////////////////////////////////////////////////////////////////////////

/// BIT TIMING VALUES

// FOR CAN
#define NBTP 0x101C

#define CAN_NBTP_NSJW_SHFT (25U)
#define CAN_NBTP_NSJW_MASK (0x7FU << CAN_NBTP_NSJW_SHFT)
#define CAN_SYNC_JUMP_WIDTH(x) ((uint8_t)(x) << CAN_NBTP_NSJW_SHFT)

#define CAN_NBTP_NTSEG1_SHFT (8U)
#define CAN_NBTP_NTSEG1_MASK (0xFFU << CAN_NBTP_NTSEG1_SHFT)
#define CAN_TIME_SEG_1(x) ((uint8_t)(x) << CAN_NBTP_NTSEG1_SHFT)

#define CAN_NBTP_NTSEG2_SHFT (0U)
#define CAN_NBTP_NTSEG2_MASK (0x7FU << CAN_NBTP_NTSEG2_SHFT)
#define CAN_TIME_SEG_2(x) ((uint8_t)(x) << CAN_NBTP_NTSEG2_SHFT)

#define CAN_NBTP_NBRP_SHFT (24U)
#define CAN_NBTP_NBRP_MASK (0x1U << CAN_NBTP_NBRP_SHFT)
#define CAN_PRESCALER(x) ((uint8_t)(x) << CAN_NBTP_NBRP_SHFT)

// FOR CAN-FD
#define DBTP 0x100C 
#define TDCR 0x1048 // Transmission Delay Compensation Register

#define CANFD_DBTP_DSJW_SHFT (0U)
#define CANFD_DBTP_DSJW_MASK (0xFU << CANFD_DBTP_DSJW_SHFT)
#define CANFD_SYNC_JUMP_WIDTH(x) ((uint8_t)(x) << CANFD_DBTP_DSJW_SHFT)

#define CANFD_DBTP_DTSEG1_SHFT (8U)
#define CANFD_DBTP_DTSEG1_MASK (0x1FU << CANFD_DBTP_DTSEG1_SHFT)
#define CANFD_TIME_SEG_1_WIDTH(x) ((uint8_t)(x) << CANFD_DBTP_DTSEG1_SHFT)

#define CANFD_DBTP_DTSEG2_SHFT (4U)
#define CANFD_DBTP_DTSEG2_MASK (0xFU << CANFD_DBTP_DTSEG2_SHFT)
#define CANFD_TIME_SEG_2_WIDTH(x) ((uint8_t)(x) << CANFD_DBTP_DTSEG2_SHFT)

#define CANFD_DBTP_DBRP_SHFT (16U)
#define CANFD_DBTP_DBRP_MASK (0xFU << CANFD_DBTP_DBRP_SHFT)
#define CANFD_PRESCALER(x) ((uint8_t)(x) << CANFD_DBTP_DBRP_SHFT)

#define CANFD_TDCR_TDCO_SHFT (8U)
#define CANFD_TDCR_TDCO_MASK (0x7FU << CANFD_TDCR_TDCO_SHFT)
#define CANFD_DELAY_COMPENSATION_OFFSET(x) ((uint8_t)(x) << CANFD_TDCR_TDCO_SHFT)

// Bit Timing Parameters
typedef struct
{
    uint8_t prescaler;
    uint8_t prop_and_phase1;
    uint8_t phase2;
    uint8_t sync_jump_width;
    uint8_t tdc;
} BitTimingParams;

///////////////////////////////////////////////////////////////////////////////////////////

/// MESSAGE RAM

// MRAM Registers' Addresses
#define SIDFC 0x1084 // 11-bit Filter (SID Filter) 
#define XIDFC 0x1088 // 29-bit Filter (XID Filter)
#define RXF0C 0x10A0 // Rx FIFO 0
#define RXF1C 0x10B0 // Rx FIFO 1
#define RXBC  0x10AC // Rx Buffers
#define RXESC 0x10BC // Rx Element Size Config
#define TXEFC 0x10F0 // Tx Event FIFO
#define TXBC  0x10C0 // Tx Buffers
#define TXESC 0x10C8 // Tx Element Size Config

// MRAM Params Bits

// SID
#define SID_LSS_SHFT (16U)
#define SID_LSS_MASK (0xFFU << SID_LSS_SHFT)
#define SID_LSS(x) ((uint8_t)(x) << SID_LSS_SHFT)

#define SID_FLSS_SHFT (0U)
#define SID_FLSS_MASK (0xFFFFU << SID_LSS_SHFT)
#define SID_FLSS(x) ((uint8_t)(x) << SID_LSS_SHFT)


// XID
#define XID_LSE_SHFT (16U)
#define XID_LSE_MASK (0x7FU << XID_LSE_SHFT)
#define XID_LSE(x) ((uint8_t)(x) << XID_LSE_SHFT)


#define XID_FLSEA_SHFT (16U)
#define XID_FLSEA_MASK (0xFFFFU << XID_FLSEA_SHFT)
#define XID_FLSEA(x) ((uint8_t)(x) << XID_FLSEA_SHFT)


// Rx FIFO 0
#define RXF0_F0OM_SHFT (31U)
#define RXF0_F0OM_MASK (0x1U << RXF0_F0OM_SHFT)
#define RXF0_F0OM(x) ((uint8_t)(x) << RXF0_F0OM_SHFT)

#define RXF0_F0WM_SHFT (24U)
#define RXF0_F0WM_MASK (0x7FU << RXF0_F0WM_SHFT)
#define RXF0_F0WM(x) ((uint8_t)(x) << RXF0_F0WM_SHFT)

#define RXF0_F0S_SHFT (16U)
#define RXF0_F0S_MASK (0x7FU << RXF0_F0S_SHFT)
#define RXF0_F0S(x) ((uint8_t)(x) << RXF0_F0S_SHFT)

#define RXF0_F0SA_SHFT (0U)
#define RXF0_F0SA_MASK (0xFFFFU << RXF0_F0SA_SHFT)
#define RXF0_F0SA(x) ((uint8_t)(x) << RXF0_F0SA_SHFT)


// Rx FIFO 1
#define RXF1_F1OM_SHFT (31U)
#define RXF1_F1OM_MASK (0x1U << RXF1_F1OM_SHFT)
#define RXF1_F1OM(x) ((uint8_t)(x) << RXF1_F1OM_SHFT)

#define RXF1_F1WM_SHFT (24U)
#define RXF1_F1WM_MASK (0x7FU << RXF1_F1WM_SHFT)
#define RXF1_F1WM(x) ((uint8_t)(x) << RXF1_F1WM_SHFT)

#define RXF1_F1S_SHFT (16U)
#define RXF1_F1S_MASK (0x7FU << RXF1_F1S_SHFT)
#define RXF1_F1S(x) ((uint8_t)(x) << RXF1_F1S_SHFT)

#define RXF1_F1SA_SHFT (0U)
#define RXF1_F1SA_MASK (0xFFFFU << RXF1_F1SA_SHFT)
#define RXF1_F1SA(x) ((uint8_t)(x) << RXF1_F1SA_SHFT)


// Rx Buffer
#define RXB_RBSA_SHFT (0U)
#define RXB_RBSA_MASK (0xFFFFU << RXB_RBSA_SHFT)
#define RXB_RBSA(x) ((uint8_t)(x) << RXB_RBSA_SHFT)


// Rx Element Size Config
#define RX_RBDS_SHFT (8U)
#define RX_RBDS_MASK (0x7U << RX_RBDS_SHFT)
#define RX_RBDS(x) ((uint8_t)(x) << RX_RBDS_SHFT)

#define RX_F1DS_SHFT (4U)
#define RX_F1DS_MASK (0x7U << RX_F1DS_SHFT)
#define RX_F1DS(x) ((uint8_t)(x) << RX_F1DS_SHFT)

#define RX_F0DS_SHFT (0U)
#define RX_F0DS_MASK (0x7U << RX_F0DS_SHFT)
#define RX_F0DS(x) ((uint8_t)(x) << RX_F0DS_SHFT)


// Tx Event FIFO Config
#define TXEVF_EFWM_SHFT (24U)
#define TXEVF_EFWM_MASK (0x3FU << TXEVF_EFWM_SHFT)
#define TXEVF_EFWM(x) ((uint8_t)(x) << TXEVF_EFWM_SHFT)

#define TXEVF_EFS_SHFT (16U)
#define TXEVF_EFS_MASK (0x3FU << TXEVF_EFS_SHFT)
#define TXEVF_EFS(x) ((uint8_t)(x) << TXEVF_EFS_SHFT)

#define TXEVF_EFSA_SHFT (0U)
#define TXEVF_EFSA_MASK (0xFFFFU << TXEVF_EFSA_SHFT)
#define TXEVF_EFSA(x) ((uint8_t)(x) << TXEVF_EFSA_SHFT)


// Tx Buffer Config
#define TXB_TFQM_SHFT (30U)
#define TXB_TFQM_MASK (0x1U << TXB_TFQM_SHFT)
#define TXB_TFQM(x) ((uint8_t)(x) << TXB_TFQM_SHFT)

#define TXB_TFQS_SHFT (24U)
#define TXB_TFQS_MASK (0x3FU << TXB_TFQS_SHFT)
#define TXB_TFQS(x) ((uint8_t)(x) << TXB_TFQS_SHFT)

#define TXB_NDTB_SHFT (16U)
#define TXB_NDTB_MASK (0x3FU << TXB_NDTB_SHFT)
#define TXB_NDTB(x) ((uint8_t)(x) << TXB_NDTB_SHFT)

#define TXB_TBSA_SHFT (0U)
#define TXB_TBSA_MASK (0xFFFFU << TXB_TBSA_SHFT)
#define TXB_TBSA(x) ((uint8_t)(x) << TXB_TBSA_SHFT)


// TX Element Size Config
#define TX_TBDS_SHFT (0U)
#define TX_TBDS_MASK (0x7U << TX_TBDS_SHFT)
#define TX_TBDS(x) ((uint8_t)(x) << TX_TBDS_SHFT)



// Message RAM Design Parameters 

// MDS = Maximum Data Size
// BPE = Bytes Per Element
// NOE = Number Of Elements

typedef struct
{
    // SID
    uint32_t SID_LSS;
    uint32_t SID_FLSS;

    // XID
    uint32_t XID_LSE;
    uint32_t XID_FLSEA;

    // Rx FIFO 0
    uint32_t RXF0_F0OM; 
    uint32_t RXF0_F0WM; 
    uint32_t RXF0_F0S; 
    uint32_t RXF0_F0SA;

    // Rx FIFO 1
    uint32_t RXF1_F1OM;
    uint32_t RXF1_F1WM;
    uint32_t RXF1_F1S;
    uint32_t RXF1_F1SA;
    
    // Rx Buffer
    uint32_t RXB_RBSA;

    // Rx Element Size Config
    uint32_t RX_RBDS;
    uint32_t RX_F1DS;
    uint32_t RX_F0DS;

    // Tx Event FIFO Config
    uint32_t TXEVF_EFWM;
    uint32_t TXEVF_EFS;
    uint32_t TXEVF_EFSA;

    // Tx Buffer Config
    uint32_t TXB_TFQM; 
    uint32_t TXB_TFQS; 
    uint32_t TXB_NDTB; 
    uint32_t TXB_TBSA;

    // TX Element Size Config
    uint32_t TX_TBDS;

} TiMRAMParams;

// Filters Configuration: SID and XID (Standard and Extended)

typedef struct
{
    // In classic mode: SFID_1 = ID, SFID_2 = MASK

    uint16_t SFID_1; 
    uint16_t SFID_2;
    uint8_t  SFT;
    uint8_t  SFEC;

} SID_filter;


// SID Filter Type
#define SID_SFT_SHFT (30U)
#define SID_SFT(x) ((uint8_t)(x) << SID_SFT_SHFT)

// Options for the SID & XID filter type
#define RANGE_FILTER            0b00U
#define DUAL_ID_FILTER          0b01U
#define CLASSIC_FILTER          0b10U
#define FILTER_ELEMENT_DISABLED 0b11U

// Standard Element Filter Configuration
#define SID_SFEC_SHFT (27U)
#define SID_SFEC(x) ((uint8_t)(x) << SID_SFEC_SHFT)

// Options for the element configuration
#define DISABLE_FILTER_ELEMENT  0b000U
#define STORE_RX_FIFO_0         0b001U  
#define STORE_RX_FIFO_1         0b010U
#define REJECT_MESSAGE          0b011U
#define SET_AS_PRIO_MSG_DFLT    0b100U
#define SET_AS_PRIO_FIFO_0      0b101U
#define SET_AS_PRIO_FIFO_1      0b110U

// Store into Rx Buffer or as debug message. If this is used, SFT is ignored and SFID1 is the filter. SFID2[10:9]
// describes where to store message, SFID2[5:0] describes which Rx Buffer to put the message (must be within
// the Rx Buffer configuration
#define STORE_RX_BUF            0b111U


#define SID_SFID1_SHFT (16U)
#define SID_SFID1_MASK (0x7FFU)
#define SID_SFID1(x) (((uint32_t)(x) & SID_SFID1_MASK) << SID_SFID1_SHFT)

#define SID_SFID2_SHFT (0U)
#define SID_SFID2_MASK (0x7FFU)
#define SID_SFID2(x) (((uint32_t)(x) & SID_SFID2_MASK) << SID_SFID2_SHFT)


typedef struct 
{
    
    uint32_t EFID1;
    uint32_t EFID2;
    uint8_t  EFT;   // For type use same options as for SID
    uint8_t  EFEC;

} XID_filter;

// Word 0

#define XID_EFEC_SHFT (29U)
#define XID_EFEC(x) ((uint8_t)(x) << XID_EFEC_SHFT)

#define XID_EFID1_SHFT (0U)
#define XID_EFID1_MASK (0x1FFFFFFF)
#define XID_EFID1(x) (((uint32_t)(x) & XID_EFID1_MASK) << XID_EFID1_SHFT)

// Word 1

#define XID_EFT_SHFT (30U)
#define XID_EFT(x) ((uint8_t)(x) << XID_EFT_SHFT)

#define XID_EFID2_SHFT (0U)
#define XID_EFID2_MASK (0x1FFFFFFF)
#define XID_EFID2(x) (((uint32_t)(x) & XID_EFID2_MASK) << XID_EFID2_SHFT)




///////////////////////////////////////////////////////////////////////////////////////////

/// Device Initialization Mode

// Device Mode Selection Register Address
#define MODE_SEL 0x0800

// Available Device Modes
#define STANDBY_MODE (0b01 << 6U)
#define NORMAL_MODE (0b10 << 6U)

// CC Control Register Address
#define CCCR 0x1018 

// Bits Necessary For Initialization
#define CAN_CCCR_INIT (1U << 0U)
#define CAN_CCCR_CCE  (1U << 1U)
#define CAN_CCCR_CSR  (1U << 4U)

/// 0 - CAN, 1 - CAN FD
#define CAN_MODE 0

/// 0 - Bit Rate Switching disabled, 1 - enabled
#define BIT_RATE_SWITCH 1
#define CCCR_BRSE (1U << 9U)
 
/// 0 - Flexible Datarate disabled, 1 - enabled
#define FD 1
#define CCCR_FDOE (1U << 8U)

///////////////////////////////////////////////////////////////////////////////////////////

/// Message Sending

#define TXFQS 0x10C4    // Tx FIFO/Queue Status register 

#define TXBUF_AR 0x10D0 // Tx Buffer Add Request Register

// Set on of the bits in TXBUF_AR to request the 
// data to be sent from the corresponding TX Buffer


#define TFFL_MASK 0x1FU
#define TFFL(reg) ((reg) & TFFL_MASK)

#define TFQPI_MAKS 0x1F0000
#define TFQPI(reg) (((reg) & TFQPI_MAKS) >> 16U)


typedef struct
{
    // Word 0
    bool ESI;
    bool XTD;
    bool RTR;
    uint16_t ID;

    // Word 1
    uint8_t MM;
    bool EFC;
    bool FDF;
    bool BRS;
    uint8_t DLC;

    // Word 2
    uint8_t data_byte_3;
    uint8_t data_byte_2;
    uint8_t data_byte_1;
    uint8_t data_byte_0;

} TXElement;

#define ESI_SHFT 31U
#define XTD_SHFT 30U
#define RTR_SHFT 29U
#define SID_SHFT 18U
#define XID_SHFT  0U

#define MM_SHFT  24U
#define EFC_SHFT 23U
#define FDF_SHFT 21U
#define BRS_SHFT  20U
#define DLC_SHFT 16U

#define DB3_SHFT 24U
#define DB2_SHFT 16U
#define DB1_SHFT 8U
#define DB0_SHFT 0U

#define WORD     0x4U

void test_func();
void debug_print(const char* format, ...);
uint8_t spiRegisterWrite(uint32_t addr, 
                         uint32_t regValue);

uint32_t spiRegisterRead(uint32_t addr);
uint8_t initCAN (const BitTimingParams  * bTParams, 
                 const TiMRAMParams     * MRAM);
uint8_t setSIDFilters(SID_filter * filters, TiMRAMParams * MRAM);
uint8_t setXIDFilters(XID_filter * filters, TiMRAMParams * MRAM);
uint8_t sendCAN(TiMRAMParams * MRAM, TXElement * TXE);
