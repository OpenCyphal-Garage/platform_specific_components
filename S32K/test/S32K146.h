/*
 * Copyright (c) 2020, NXP. All rights reserved.
 * Distributed under The MIT License.
 */

/**
 * @file
 * Dummy file to support unit testing.
 */

#ifndef S32K146_H_INCLUDED
#define S32K146_H_INCLUDED

#ifndef UAVCAN_NODE_BOARD_USED
#    define UAVCAN_NODE_BOARD_USED 1
#endif

#include <stdint.h>

#define MCU_S32K146


/* IO definitions (access restrictions to peripheral registers) */
/**
 *   IO Type Qualifiers are used
 *   \li to specify the access to peripheral variables.
 *   \li for automatic generation of peripheral register debug information.
 */
#    ifndef __IO
#        ifdef __cplusplus
#            define __I volatile /*!< Defines 'read only' permissions                 */
#        else
#            define __I volatile const /*!< Defines 'read only' permissions                 */
#        endif
#        define __O volatile  /*!< Defines 'write only' permissions                */
#        define __IO volatile /*!< Defines 'read / write' permissions              */
#    endif


#define DISABLE_INTERRUPTS()
#define ENABLE_INTERRUPTS()

// +--------------------------------------------------------------------------+
// | FAKE CAN PERIPHERAL
// +--------------------------------------------------------------------------+

/** CAN - Size of Registers Arrays */
#define CAN_RAMn_COUNT 128u
#define CAN_RXIMR_COUNT 16u
#define CAN_WMB_COUNT 4u

/** CAN - Register Layout Typedef */
typedef struct
{
    __IO uint32_t MCR;   /**< Module Configuration Register, offset: 0x0 */
    __IO uint32_t CTRL1; /**< Control 1 register, offset: 0x4 */
    __IO uint32_t TIMER; /**< Free Running Timer, offset: 0x8 */
    uint8_t       RESERVED_0[4];
    __IO uint32_t RXMGMASK; /**< Rx Mailboxes Global Mask Register, offset: 0x10 */
    __IO uint32_t RX14MASK; /**< Rx 14 Mask register, offset: 0x14 */
    __IO uint32_t RX15MASK; /**< Rx 15 Mask register, offset: 0x18 */
    __IO uint32_t ECR;      /**< Error Counter, offset: 0x1C */
    __IO uint32_t ESR1;     /**< Error and Status 1 register, offset: 0x20 */
    uint8_t       RESERVED_1[4];
    __IO uint32_t IMASK1; /**< Interrupt Masks 1 register, offset: 0x28 */
    uint8_t       RESERVED_2[4];
    __IO uint32_t IFLAG1; /**< Interrupt Flags 1 register, offset: 0x30 */
    __IO uint32_t CTRL2;  /**< Control 2 register, offset: 0x34 */
    __I uint32_t ESR2;    /**< Error and Status 2 register, offset: 0x38 */
    uint8_t      RESERVED_3[8];
    __I uint32_t CRCR;      /**< CRC Register, offset: 0x44 */
    __IO uint32_t RXFGMASK; /**< Rx FIFO Global Mask register, offset: 0x48 */
    __I uint32_t RXFIR;     /**< Rx FIFO Information Register, offset: 0x4C */
    __IO uint32_t CBT;      /**< CAN Bit Timing Register, offset: 0x50 */
    uint8_t       RESERVED_4[44];
    __IO uint32_t RAMn[CAN_RAMn_COUNT]; /**< Embedded RAM, array offset: 0x80, array step: 0x4 */
    uint8_t       RESERVED_5[1536];
    __IO uint32_t RXIMR[CAN_RXIMR_COUNT]; /**< Rx Individual Mask Registers, array offset: 0x880, array step: 0x4 */
    uint8_t       RESERVED_6[576];
    __IO uint32_t CTRL1_PN;       /**< Pretended Networking Control 1 Register, offset: 0xB00 */
    __IO uint32_t CTRL2_PN;       /**< Pretended Networking Control 2 Register, offset: 0xB04 */
    __IO uint32_t WU_MTC;         /**< Pretended Networking Wake Up Match Register, offset: 0xB08 */
    __IO uint32_t FLT_ID1;        /**< Pretended Networking ID Filter 1 Register, offset: 0xB0C */
    __IO uint32_t FLT_DLC;        /**< Pretended Networking DLC Filter Register, offset: 0xB10 */
    __IO uint32_t PL1_LO;         /**< Pretended Networking Payload Low Filter 1 Register, offset: 0xB14 */
    __IO uint32_t PL1_HI;         /**< Pretended Networking Payload High Filter 1 Register, offset: 0xB18 */
    __IO uint32_t FLT_ID2_IDMASK; /**< Pretended Networking ID Filter 2 Register / ID Mask Register, offset: 0xB1C */
    __IO uint32_t PL2_PLMASK_LO;  /**< Pretended Networking Payload Low Filter 2 Register / Payload Low Mask Register,
                                     offset: 0xB20 */
    __IO uint32_t PL2_PLMASK_HI;  /**< Pretended Networking Payload High Filter 2 low order bits / Payload High Mask
                                     Register, offset: 0xB24 */
    uint8_t RESERVED_7[24];
    struct
    {                         /* offset: 0xB40, array step: 0x10 */
        __I uint32_t WMBn_CS; /**< Wake Up Message Buffer Register for C/S, array offset: 0xB40, array step: 0x10 */
        __I uint32_t WMBn_ID; /**< Wake Up Message Buffer Register for ID, array offset: 0xB44, array step: 0x10 */
        __I          uint32_t
                     WMBn_D03; /**< Wake Up Message Buffer Register for Data 0u-3, array offset: 0xB48, array step: 0x10 */
        __I uint32_t WMBn_D47; /**< Wake Up Message Buffer Register Data 4-7, array offset: 0xB4C, array step: 0x10 */
    } WMB[CAN_WMB_COUNT];
    uint8_t RESERVED_8[128];
    __IO uint32_t FDCTRL; /**< CAN FD Control Register, offset: 0xC00 */
    __IO uint32_t FDCBT;  /**< CAN FD Bit Timing Register, offset: 0xC04 */
    __I uint32_t FDCRC;   /**< CAN FD CRC Register, offset: 0xC08 */
} CAN_Type, *CAN_MemMapPtr;

#define CAN0 (reinterpret_cast<CAN_Type*>(0x0Au))
#define CAN1 (reinterpret_cast<CAN_Type*>(0x0Au))

#define CAN_BASE_PTRS   \
{                       \
    CAN0, CAN1          \
}

#define CAN_WMBn_ID_ID_MASK 0u
#define CAN_RAMn_DATA_BYTE_1(X) X
#define CAN_MCR_LPMACK_MASK 0u
#define CAN_MCR_MDIS_MASK 0u
#define CAN_MCR_NOTRDY_MASK 0u
#define CAN_MCR_HALT_MASK 0u
#define CAN_IMASK1_BUF31TO0M(X) X
#define CAN_MCR_FRZACK_MASK 0u
#define CAN_RAMn_DATA_BYTE_0(X) X
#define CAN_MCR_IRMQ_MASK 0u
#define CAN_MCR_MAXMB(X) X
#define CAN_MCR_SRXDIS_MASK 0u
#define CAN_MCR_MAXMB_MASK 0u
#define CAN_FDCBT_FRJW(X) X
#define CAN_FDCBT_FPSEG2(X) X
#define CAN_FDCTRL_TDCEN_MASK 0u
#define CAN_FDCTRL_TDCOFF(X) X
#define CAN_FDCBT_FPSEG1(X) X
#define CAN_FDCTRL_MBDSR0(X) X
#define CAN_FDCBT_FPROPSEG(X) X
#define CAN_FDCBT_FPRESDIV(X) X
#define CAN_CBT_ERJW(X) X
#define CAN_CBT_EPROPSEG(X) X
#define CAN_CBT_EPRESDIV(X) X
#define CAN_CBT_BTF_MASK 0u
#define CAN_CBT_EPSEG1(X) X
#define CAN_CBT_EPSEG2(X) X
#define CAN_FDCTRL_FDRATE_MASK 0u
#define CAN_CTRL1_CLKSRC_MASK 0u
#define CAN_MCR_FDEN_MASK 0u
#define CAN_MCR_FRZ_MASK 0u
#define CAN_CTRL2_ISOCANFDEN_MASK 0u
#define CAN_ESR2_IMB_MASK 0u
#define CAN_ESR2_VPS_MASK 0u
#define CAN_WMBn_CS_DLC_MASK 0u
#define CAN_WMBn_CS_DLC_SHIFT 0u
#define CAN_WMBn_CS_DLC(X) X
#define CAN_ESR2_LPTM_SHIFT 0u
#define CAN_ESR2_LPTM_MASK 0u


// +--------------------------------------------------------------------------+
// | FAKE TIMERS
// +--------------------------------------------------------------------------+

/** LPIT - Size of Registers Arrays */
#define LPIT_TMR_COUNT 4u

/** LPIT - Register Layout Typedef */
typedef struct
{
    __I uint32_t VERID;   /**< Version ID Register, offset: 0x0 */
    __I uint32_t PARAM;   /**< Parameter Register, offset: 0x4 */
    __IO uint32_t MCR;    /**< Module Control Register, offset: 0x8 */
    __IO uint32_t MSR;    /**< Module Status Register, offset: 0xC */
    __IO uint32_t MIER;   /**< Module Interrupt Enable Register, offset: 0x10 */
    __IO uint32_t SETTEN; /**< Set Timer Enable Register, offset: 0x14 */
    __IO uint32_t CLRTEN; /**< Clear Timer Enable Register, offset: 0x18 */
    uint8_t       RESERVED_0[4];
    struct
    {                        /* offset: 0x20, array step: 0x10 */
        __IO uint32_t TVAL;  /**< Timer Value Register, array offset: 0x20, array step: 0x10 */
        __I uint32_t CVAL;   /**< Current Timer Value, array offset: 0x24, array step: 0x10 */
        __IO uint32_t TCTRL; /**< Timer Control Register, array offset: 0x28, array step: 0x10 */
        uint8_t       RESERVED_0[4];
    } TMR[LPIT_TMR_COUNT];
} LPIT_Type, *LPIT_MemMapPtr;

#define LPIT0 (reinterpret_cast<LPIT_Type*>(0x0Au))

#define LPIT_SETTEN_SET_T_EN_0(X) X
#define LPIT_CLRTEN_CLR_T_EN_2(X) X
#define LPIT_SETTEN_SET_T_EN_2(X) X
#define LPIT_TMR_CVAL_TMR_CUR_VAL_MASK 0u
#define LPIT_TMR_TVAL_TMR_VAL_MASK 0u
#define LPIT_CLRTEN_CLR_T_EN_3(X) X
#define LPIT_SETTEN_SET_T_EN_3(X) X
#define LPIT_TMR_TCTRL_CHAIN(X) X
#define LPIT_MCR_M_CEN(X) X
#define LPIT_TMR_TCTRL_MODE(X) X
#define LPIT_SETTEN_SET_T_EN_1(X) X

// +--------------------------------------------------------------------------+
// | FAKE GPIO
// +--------------------------------------------------------------------------+

/** PCC - Size of Registers Arrays */
#define PCC_PCCn_COUNT 122u

/** PCC - Register Layout Typedef */
typedef struct
{
    __IO uint32_t
         PCCn[PCC_PCCn_COUNT]; /**< PCC Reserved Register 0u..PCC ENET Register, array offset: 0x0, array step: 0x4 */
} PCC_Type, *PCC_MemMapPtr;

#define PCC (reinterpret_cast<PCC_Type*>(0x0Au))

#define PCC_PCCn_CGC_MASK 0u
#define PCC_LPIT_INDEX 0u
#define PCC_PCCn_PCS(X) X
#define PCC_PCCn_CGC(X) X

/** GPIO - Register Layout Typedef */
typedef struct
{
    __IO uint32_t PDOR; /**< Port Data Output Register, offset: 0x0 */
    __O uint32_t PSOR;  /**< Port Set Output Register, offset: 0x4 */
    __O uint32_t PCOR;  /**< Port Clear Output Register, offset: 0x8 */
    __O uint32_t PTOR;  /**< Port Toggle Output Register, offset: 0xC */
    __I uint32_t PDIR;  /**< Port Data Input Register, offset: 0x10 */
    __IO uint32_t PDDR; /**< Port Data Direction Register, offset: 0x14 */
    __IO uint32_t PIDR; /**< Port Input Disable Register, offset: 0x18 */
} GPIO_Type, *GPIO_MemMapPtr;

#define PTE (reinterpret_cast<GPIO_Type*>(0x0Au))

/** PORT - Size of Registers Arrays */
#define PORT_PCR_COUNT 32u

/** PORT - Register Layout Typedef */
typedef struct
{
    __IO uint32_t PCR[PORT_PCR_COUNT]; /**< Pin Control Register n, array offset: 0x0, array step: 0x4 */
    __O uint32_t GPCLR;                /**< Global Pin Control Low Register, offset: 0x80 */
    __O uint32_t GPCHR;                /**< Global Pin Control High Register, offset: 0x84 */
    uint8_t      RESERVED_0[24];
    __IO uint32_t ISFR; /**< Interrupt Status Flag Register, offset: 0xA0 */
    uint8_t       RESERVED_1[28];
    __IO uint32_t DFER; /**< Digital Filter Enable Register, offset: 0xC0 */
    __IO uint32_t DFCR; /**< Digital Filter Clock Register, offset: 0xC4 */
    __IO uint32_t DFWR; /**< Digital Filter Width Register, offset: 0xC8 */
} PORT_Type, *PORT_MemMapPtr;

#define PCC_PORTA_INDEX 0u
#define PCC_PORTE_INDEX 1u

#define PORTA (reinterpret_cast<PORT_Type*>(0x0Au))
#define PORTE (reinterpret_cast<PORT_Type*>(0x0Au))

#define PORT_PCR_MUX(X) X

// +--------------------------------------------------------------------------+
// | FAKE NVIC
// +--------------------------------------------------------------------------+

/** S32_NVIC - Size of Registers Arrays */
#    define S32_NVIC_ISER_COUNT 4u
#    define S32_NVIC_ICER_COUNT 4u
#    define S32_NVIC_ISPR_COUNT 4u
#    define S32_NVIC_ICPR_COUNT 4u
#    define S32_NVIC_IABR_COUNT 4u
#    define S32_NVIC_IP_COUNT 123u

/** S32_NVIC - Register Layout Typedef */
typedef struct
{
    __IO uint32_t ISER[S32_NVIC_ISER_COUNT]; /**< Interrupt Set Enable Register n, array offset: 0x0, array step: 0x4 */
    uint8_t       RESERVED_0[112];
    __IO          uint32_t
                  ICER[S32_NVIC_ICER_COUNT]; /**< Interrupt Clear Enable Register n, array offset: 0x80, array step: 0x4 */
    uint8_t       RESERVED_1[112];
    __IO          uint32_t
                  ISPR[S32_NVIC_ISPR_COUNT]; /**< Interrupt Set Pending Register n, array offset: 0x100, array step: 0x4 */
    uint8_t       RESERVED_2[112];
    __IO          uint32_t
                  ICPR[S32_NVIC_ICPR_COUNT]; /**< Interrupt Clear Pending Register n, array offset: 0x180, array step: 0x4 */
    uint8_t       RESERVED_3[112];
    __IO          uint32_t
                  IABR[S32_NVIC_IABR_COUNT]; /**< Interrupt Active bit Register n, array offset: 0x200, array step: 0x4 */
    uint8_t       RESERVED_4[240];
    __IO uint8_t IP[S32_NVIC_IP_COUNT]; /**< Interrupt Priority Register n, array offset: 0x300, array step: 0x1 */
    uint8_t      RESERVED_5[2693];
    __O uint32_t STIR; /**< Software Trigger Interrupt Register, offset: 0xE00 */
} S32_NVIC_Type, *S32_NVIC_MemMapPtr;

#define S32_NVIC (reinterpret_cast<S32_NVIC_Type*>(0x0Au))

#endif // S32K146_H_INCLUDED
