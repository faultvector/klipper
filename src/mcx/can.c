// CAN support for NXP MCXA366
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include <string.h>
#include "autoconf.h"
#include "command.h"
#include "MCXA366.h"
#include "generic/armcm_boot.h"
#include "generic/canbus.h"
#include "generic/canserial.h"
#include "sched.h"

#define CAN_RX_MB 0
#define CAN_TX_MB 1

#define CAN_CLOCK_FREQ 240000000U

#define CAN_MB_CODE_SHIFT 24
#define CAN_MB_DLC_SHIFT 16
#define CAN_MB_RTR (1U << 20)
#define  CAN_MB_IDE (1U << 21)

#define CAN_MB_CODE_RX_EMPTY   (4U << CAN_MB_CODE_SHIFT)
#define CAN_MB_CODE_TX_INACTIVE (8U << CAN_MB_CODE_SHIFT)
#define CAN_MB_CODE_TX_DATA    (12U << CAN_MB_CODE_SHIFT)

#define CAN_MB_STD_ID(id) (((uint32_t)(id) & 0x7ffU) << 18)

#define CAN_RX_IFLAG (1U << CAN_RX_MB)
#define CAN_TX_IFLAG (1U << CAN_TX_MB)

static volatile uint8_t can_tx_busy;

/****************************************************************
 * Clock and pin setup
 ****************************************************************/

static void can_clock_setup(void)
{
    /*
     * FLEXCAN0 uses FRO_HF_DIV, matching the working NXP SDK
     * FRDM-MCXA366 example.  Divider is 1.
     */
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    MRCC0->MRCC_GLB_CC1_SET =
        MRCC_MRCC_GLB_CC1_PORT1_MASK
        | MRCC_MRCC_GLB_CC1_FLEXCAN0_MASK;

    MRCC0->MRCC_GLB_RST1_SET =
        MRCC_MRCC_GLB_RST1_PORT1_MASK
        | MRCC_MRCC_GLB_RST1_FLEXCAN0_MASK;

    MRCC0->MRCC_FLEXCAN0_CLKSEL = 0U;

    MRCC0->MRCC_FLEXCAN0_CLKDIV =
        MRCC_MRCC_FLEXCAN0_CLKDIV_RESET_MASK
        | MRCC_MRCC_FLEXCAN0_CLKDIV_HALT_MASK;

    MRCC0->MRCC_FLEXCAN0_CLKDIV =
        MRCC_MRCC_FLEXCAN0_CLKDIV_HALT_MASK
        | MRCC_MRCC_FLEXCAN0_CLKDIV_DIV(0U);

    MRCC0->MRCC_FLEXCAN0_CLKDIV &=
        ~MRCC_MRCC_FLEXCAN0_CLKDIV_HALT_MASK;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}

static void
can_pin_setup(void)
{
    /*
     * FRDM-MCXA366:
     *
     * P1_11 ALT11 = CAN0_RXD
     * P1_2  ALT11 = CAN0_TXD
     */
    PORT1->PCR[11] =
        PORT_PCR_MUX(11U)
        | PORT_PCR_IBE_MASK;

    PORT1->PCR[2] =
        PORT_PCR_MUX(11U)
        | PORT_PCR_IBE_MASK;
}

/****************************************************************
 * Bit timing
 ****************************************************************/

static uint32_t
can_make_ctrl1(uint32_t bitrate)
{
    /*
     * Use 16 time quanta per bit:
     *
     * Sync      = 1 TQ
     * PROPSEG   = 7 TQ
     * PSEG1     = 6 TQ
     * PSEG2     = 2 TQ
     *
     * Sample point = 14 / 16 = 87.5%
     *
     * FlexCAN fields encode segment length minus one.
     */
    uint32_t clocks_per_bit = CAN_CLOCK_FREQ / bitrate;
    uint32_t prescaler = clocks_per_bit / 16U;

    if (!prescaler
        || prescaler > 256U
        || prescaler * 16U * bitrate != CAN_CLOCK_FREQ)
        shutdown("Unsupported CAN bitrate");

    return CAN_CTRL1_PRESDIV(prescaler - 1U)
        | CAN_CTRL1_RJW(1U)
        | CAN_CTRL1_PSEG1(5U)
        | CAN_CTRL1_PSEG2(1U)
        | CAN_CTRL1_PROPSEG(6U)
        | CAN_CTRL1_ERRMSK_MASK
        | CAN_CTRL1_BOFFMSK_MASK;
}


/****************************************************************
 * Receive filtering
 ****************************************************************/

void
canhw_set_filter(uint32_t id)
{
    /*
     * First-light implementation: accept all standard frames.
     *
     * Generic Klipper will discard messages that do not belong to
     * this node.  Once discovery works, we can tighten this to the
     * admin ID and assigned node IDs.
     */
    CAN0->RXMGMASK = 0U;

    CAN0->MB[CAN_RX_MB].CS = 0U;
    CAN0->MB[CAN_RX_MB].ID = 0U;
    CAN0->MB[CAN_RX_MB].WORD0 = 0U;
    CAN0->MB[CAN_RX_MB].WORD1 = 0U;
    CAN0->MB[CAN_RX_MB].CS = CAN_MB_CODE_RX_EMPTY;
}


/****************************************************************
 * Transmit
 ****************************************************************/

static uint32_t
pack_word(const uint8_t *data, uint32_t offset, uint32_t len)
{
    uint32_t word = 0;

    for (uint32_t i = 0; i < 4; i++) {
        uint32_t pos = offset + i;
        if (pos < len)
            word |= (uint32_t)data[pos] << (24 - 8 * i); 
    }

    return word;
}

int
canhw_send(struct canbus_msg *msg)
{
    if (can_tx_busy)
        return -1;

    uint32_t len = CANMSG_DATA_LEN(msg);

    can_tx_busy = 1;

    CAN0->MB[CAN_TX_MB].CS = CAN_MB_CODE_TX_INACTIVE;
    CAN0->MB[CAN_TX_MB].ID = CAN_MB_STD_ID(msg->id);
    CAN0->MB[CAN_TX_MB].WORD0 = pack_word(msg->data, 0, len);
    CAN0->MB[CAN_TX_MB].WORD1 = pack_word(msg->data, 4, len);

    uint32_t cs = CAN_MB_CODE_TX_DATA | (len << CAN_MB_DLC_SHIFT);

    if (msg->id & CANMSG_ID_RTR)
        cs |= CAN_MB_RTR;

    CAN0->MB[CAN_TX_MB].CS = cs;

    return len;
}


/****************************************************************
 * Status
 ****************************************************************/

void
canhw_get_status(struct canbus_status *status)
{
    uint32_t ecr = CAN0->ECR;
    uint32_t esr = CAN0->ESR1;

    status->tx_error = 
        (ecr & CAN_ECR_TXERRCNT_MASK) >> CAN_ECR_TXERRCNT_SHIFT;
    status->rx_error = 
        (ecr & CAN_ECR_RXERRCNT_MASK) >> CAN_ECR_RXERRCNT_SHIFT;
    status->tx_retries = 0;

    uint32_t fault =
        (esr & CAN_ESR1_FLTCONF_MASK) >> CAN_ESR1_FLTCONF_SHIFT;

    if (fault >= 2)
        status->bus_state = CANBUS_STATE_OFF;
    else if (fault == 1)
        status->bus_state = CANBUS_STATE_PASSIVE;
    else if (esr & (CAN_ESR1_RXWRN_MASK | CAN_ESR1_TXWRN_MASK))
        status->bus_state = CANBUS_STATE_WARN;
    else
        status->bus_state = CANBUS_STATE_ACTIVE;
}

/****************************************************************
 * Interrupt handling
 ****************************************************************/

static uint8_t
can_data_byte(uint32_t word, uint32_t index)
{
    return (word >> (24 - 8 * index)) & 0xff;
}

void
CAN0_IRQHandler(void)
{
    uint32_t flags = CAN0->IFLAG1;

    if (flags & CAN_RX_IFLAG) {
        /*
         * Reading CS locks the FlexCAN receive mailbox.  Reading TIMER
         * after ID/data unlocks it.
         */
        uint32_t cs = CAN0->MB[CAN_RX_MB].CS;
        uint32_t id = CAN0->MB[CAN_RX_MB].ID;
        uint32_t word0 = CAN0->MB[CAN_RX_MB].WORD0;
        uint32_t word1 = CAN0->MB[CAN_RX_MB].WORD1;
        (void)CAN0->TIMER;

        struct canbus_msg msg;
        msg.id = (id >> 18) & 0x7ffU;
        msg.dlc = (cs >> CAN_MB_DLC_SHIFT) & 0x0fU;

        msg.data[0] = can_data_byte(word0, 0);
        msg.data[1] = can_data_byte(word0, 1);
        msg.data[2] = can_data_byte(word0, 2);
        msg.data[3] = can_data_byte(word0, 3);
        msg.data[4] = can_data_byte(word1, 0);
        msg.data[5] = can_data_byte(word1, 1);
        msg.data[6] = can_data_byte(word1, 2);
        msg.data[7] = can_data_byte(word1, 3);

        CAN0->IFLAG1 = CAN_RX_IFLAG;

        /* Re-arm RX mailbox */
        CAN0->MB[CAN_RX_MB].CS = CAN_MB_CODE_RX_EMPTY;

        canbus_process_data(&msg);
    }

    if (flags & CAN_TX_IFLAG) {
        CAN0->IFLAG1 = CAN_TX_IFLAG;
        can_tx_busy = 0;
        CAN0->MB[CAN_TX_MB].CS = CAN_MB_CODE_TX_INACTIVE;
        canbus_notify_tx();
    }
}


/****************************************************************
 * Initialization
 ****************************************************************/

void
can_init(void)
{
    can_clock_setup();
    can_pin_setup();

    /*
    * Enable FlexCAN.
    */
    CAN0->MCR &= ~CAN_MCR_MDIS_MASK;
    
    /*
    * Reset the FlexCAN protocol engine to a known state.
    */
    CAN0->MCR |= CAN_MCR_SOFTRST_MASK;
    while (CAN0->MCR & CAN_MCR_SOFTRST_MASK)
        ;
    
    /*
    * Enter freeze mode for configuration.
    */
    CAN0->MCR |= CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK;
    
    while (!(CAN0->MCR & CAN_MCR_FRZACK_MASK))
        ;
    
    /*
    * Initialize all message-buffer RAM.  This matters on FlexCAN
    * implementations with ECC-protected message memory.
    */
    for (uint32_t i = 0; i < CAN_MB_SIZE_MB_GROUP_MB_COUNT; i++) {
        CAN0->MB[i].CS = 0U;
        CAN0->MB[i].ID = 0U;
        CAN0->MB[i].WORD0 = 0U;
        CAN0->MB[i].WORD1 = 0U;
    }
    
    for (uint32_t i = 0; i < CAN_RXIMR_COUNT; i++)
        CAN0->RXIMR[i] = 0U;
    
    /*
     * Classic CAN, two message buffers, self reception disabled.
     */
    CAN0->MCR =
        (CAN0->MCR
         & ~(CAN_MCR_MAXMB_MASK | CAN_MCR_RFEN_MASK
             | CAN_MCR_FDEN_MASK))
        | CAN_MCR_MAXMB(CAN_TX_MB)
        | CAN_MCR_SRXDIS_MASK
        | CAN_MCR_FRZ_MASK
        | CAN_MCR_HALT_MASK;

    CAN0->CTRL1 = can_make_ctrl1(CONFIG_CANBUS_FREQUENCY);

    canhw_set_filter(0);

    CAN0->MB[CAN_TX_MB].CS = CAN_MB_CODE_TX_INACTIVE;
    CAN0->MB[CAN_TX_MB].ID = 0U;
    CAN0->MB[CAN_TX_MB].WORD0 = 0U;
    CAN0->MB[CAN_TX_MB].WORD1 = 0U;

    CAN0->IFLAG1 = 0xffffffffU;
    CAN0->IMASK1 = CAN_RX_IFLAG | CAN_TX_IFLAG;
    
    armcm_enable_irq(CAN0_IRQHandler, CAN0_IRQn, 1);

    /*
     * TEMPORARY bring-up UUID seed.
     *
     * Replace with the MCXA366 hardware unique ID once the CAN transport
     * is proven.
     */
    static uint8_t raw_uuid[] = {
        0x4d, 0x43, 0x58, 0x41, 0x33, 0x36, 0x36, 0x01
    };
    canserial_set_uuid(raw_uuid, sizeof(raw_uuid));

    /*
     * Leave freeze mode and enter normal operation.
     */
    CAN0->MCR &= ~CAN_MCR_HALT_MASK;

    while(CAN0->MCR & CAN_MCR_FRZACK_MASK)
        ;
}
DECL_INIT(can_init);