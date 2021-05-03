#ifndef _N_RF24L01_H_
#define _N_RF24L01_H_

#include <stdint.h>

// commands
#define CMD_R_REGISTER          (uint8_t)0x00 // 000A AAAA
#define CMD_W_REGISTER(reg)     (uint8_t)(reg | (1<<5)) // 001A AAAA
#define CMD_R_RX_PAYLOAD        (uint8_t)0x61 // 0110 0001
#define CMD_W_TX_PAYLOAD        (uint8_t)0xA0 // 1010 0000
#define CMD_FLUSH_TX            (uint8_t)0xE1 // 1110 0001
#define CMD_FLUSH_RX            (uint8_t)0xE2 // 1110 0010
#define CMD_REUSE_TX_PL         (uint8_t)0xE3 // 1110 0011
#define CMD_ACTIVATE            (uint8_t)0x50 // 0101 0000 - This write command followed by data 0x73
#define CMD_R_RX_PL_WID         (uint8_t)0x60 // 0110 0000 -  Read RX-payload
#define CMD_W_ACK_PAYLOAD(pipe) (uint8_t)((pipe&0x7) | 0xA8) // 1010 1PPP - Write Payload to be transmitted together with n PIPE PPP
#define CMD_W_TX_PAYLOAD_NO_ACK (uint8_t)0x58 // 1011 000 - Used in TX mode. Disables AUTOACK on this specific packet
#define CMD_NOP                 (uint8_t)0xFF // 1111 1111 -  No Operation. Might be used to read the STATUSregister


// registers addresses defines
#define REG_CONFIG        (uint8_t)0x00
#define REG_EN_AA         (uint8_t)0x01
#define REG_EN_RXADDR     (uint8_t)0x02
#define REG_SETUP_AW      (uint8_t)0x03
#define REG_SETUP_REPR    (uint8_t)0x04
#define REG_RF_CH         (uint8_t)0x05
#define REG_RF_SETUP      (uint8_t)0x06
#define REG_STATUS        (uint8_t)0x07
#define REG_OBSERVE_TX    (uint8_t)0x08
#define REG_CD            (uint8_t)0x09
#define REG_RX_ADDR_P0    (uint8_t)0x0A
#define REG_RX_ADDR_P1    (uint8_t)0x0B
#define REG_RX_ADDR_P2    (uint8_t)0x0C
#define REG_RX_ADDR_P3    (uint8_t)0x0D
#define REG_RX_ADDR_P4    (uint8_t)0x0E
#define REG_RX_ADDR_P5    (uint8_t)0x0F
#define REG_TX_ADDR       (uint8_t)0x10
#define REG_RX_PW_P0      (uint8_t)0x11
#define REG_RX_PW_P1      (uint8_t)0x12
#define REG_RX_PW_P2      (uint8_t)0x13
#define REG_RX_PW_P3      (uint8_t)0x14
#define REG_RX_PW_P4      (uint8_t)0x15
#define REG_RX_PW_P5      (uint8_t)0x16
#define REG_FIFO_STATUS   (uint8_t)0x17
#define REG_DYNPD         (uint8_t)0x1C
#define REG_FEATURE       (uint8_t)0x1D

typedef union
{
  struct
  {
    unsigned PRIM_RX      : 1; // RX/TX control (1: PRX, 0: PTX)
    unsigned PWR_UP       : 1; // 1: POWER UP, 0:POWER DOWN
    unsigned CRCO         : 1; // CRC encoding scheme ('0' - 1 byte, '1' – 2 bytes)
    unsigned EN_CRC       : 1; // Enable CRC. Forced high if one of the bits in the EN_AA is high
    unsigned MASK_MAX_RT  : 1; //  Mask interrupt caused by MAX_RT (1: Interrupt not reflected on the IRQ pin, 0: Reflect MAX_RT as active low interrupt on the IRQ pin)
    unsigned MASK_TX_DS   : 1; // Mask interrupt caused by TX_DS(1: Interrupt not reflected on the IRQ pin, 0: Reflect TX_DS as active low interrupt on the IRQ pin)
    unsigned MASK_RX_DR   : 1; //  Mask interrupt caused by RX_DR(1: Interrupt not reflected on the IRQ pin, 0: Reflect RX_DR as active low interrupt on the IRQ pin)
    unsigned reserved_0   : 1; // 
  } CONFIG;
  uint8_t byte;
} t_nRfConfig; // Configuration Register


typedef union
{
  struct
  {
    unsigned ENAA_P0  : 1; // Enable auto acknowledgement data pipe 0
    unsigned ENAA_P1  : 1; // Enable auto acknowledgement data pipe 1
    unsigned ENAA_P2  : 1; // Enable auto acknowledgement data pipe 2
    unsigned ENAA_P3  : 1; // Enable auto acknowledgement data pipe 3
    unsigned ENAA_P4  : 1; // Enable auto acknowledgement data pipe 4
    unsigned ENAA_P5  : 1; // Enable auto acknowledgement data pipe 5
    unsigned reserved : 2;
  } en_aa;
  uint8_t byte;
} t_nRF_EN_AA; // Enable ‘Auto Acknowledgment’


typedef union
{
  struct
  {
    unsigned ERX_P0   : 1; // Enable data pipe 0
    unsigned ERX_P1   : 1; // Enable data pipe 1
    unsigned ERX_P2   : 1; // Enable data pipe 2
    unsigned ERX_P3   : 1; // Enable data pipe 3
    unsigned ERX_P4   : 1; // Enable data pipe 4
    unsigned ERX_P5   : 1; // Enable data pipe 5
    unsigned reserved : 2;
  } en_rxaddr;
  uint8_t byte;
} t_nRf_EN_RXADDR; // Enabled RX Addresses

typedef union
{
  struct
  {
    unsigned AW       : 2;  // RX/TX Address field width 
                            //'00' - Illegal
                            //'01' - 3 bytes 
                            //'10' - 4 bytes 
                            //'11' – 5 bytes
    unsigned reserved : 6;
  } SETUP_AW;
  uint8_t byte;
} t_nRf_SETUP_AW; // Setup of Address Widths 


typedef union
{
  struct
  {
    unsigned ARC  : 4; // Auto Retransmit Count (‘0000’,..., ‘1111’ – Up to 15 Re-Transmit)
    unsigned ARD  : 4; // Auto Retransmit Delay (‘0000’ – Wait 250µS, ..., ‘1111’ – Wait 4000µS)
  } SETUP_RETR;
  uint8_t byte;
} t_nRf_SETUP_RETR; // Setup of Automatic Retransmission


typedef union
{
  struct
  {
    unsigned RF_CH    : 7; // Sets the frequency channel nRF24L01 operates on
    unsigned reserved : 1;
  } RF_CH;
  uint8_t byte;
} t_nRF_RF_CH; // RF Channel


typedef union
{
  struct
  {
    unsigned LNA_HCURR  : 1; //  Setup LNA gain
    unsigned RF_PWR     : 2; // Set RF output power in TX mode
    unsigned RF_DR      : 1; // Air Data Rate(‘0’ – 1Mbps, ‘1’ – 2Mbps)
    unsigned PLL_LOCK   : 1; // Force PLL lock signal. Only used in test
    unsigned reserved   : 3;
  } RF_SETUP;
  uint8_t byte;
} t_nRF_RF_SETUP; // RF Setup Register

typedef union
{
  struct
  {
    unsigned TX_FULL  : 1; // TX FIFO full flag. (1: TX FIFO full. , 0: Available locations in TX FIFO.)
    unsigned RX_P_NO  : 3; // Data pipe number for the payload available for reading from RX_FIFO
    unsigned MAX_RT   : 1; // Maximum number of TX retransmits interrupt
    unsigned TX_DS    : 1; // Data Sent TX FIFO interrupt.
    unsigned RX_DR    : 1; // Data Ready RX FIFO interrupt.
    unsigned reserved : 1;
  } STATUS;
  uint8_t byte;
} t_nRF_STATUS;


typedef union
{
  struct
  {
    unsigned ARC_CNT  : 4; // Count retransmitted packets.
    unsigned PLOS_CNT : 4; // Count lost packets
  } OBSERVE_TX;
  uint8_t byte;
} t_nRF_OBSERVE_TX; // Transmit observe register


typedef union
{
  struct
  {
    unsigned CD       : 1; // Carrier Detect.
    unsigned reserved : 7;
  } CD;
  uint8_t byte;
} t_nRF_CD; // Carrier Detect.


typedef struct
{
  uint8_t addr;
  uint8_t *reg_union;
} t_register;


#endif /* _N_RF24L01_H_ */
