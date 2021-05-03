#include "nRF24L01.h"


//#define          (uint8_t)0x01
//#define      (uint8_t)0x02
//#define       (uint8_t)0x03
//#define     (uint8_t)0x04
//#define          (uint8_t)0x05
//#define       (uint8_t)0x06
//#define         (uint8_t)0x07
//#define     (uint8_t)0x08
//#define             (uint8_t)0x09
//#define REG_RX_ADDR_P0    (uint8_t)0x0A
//#define REG_RX_ADDR_P1    (uint8_t)0x0B
//#define REG_RX_ADDR_P2    (uint8_t)0x0C
//#define REG_RX_ADDR_P3    (uint8_t)0x0D
//#define REG_RX_ADDR_P4    (uint8_t)0x0E
//#define REG_RX_ADDR_P5    (uint8_t)0x0F
//#define REG_TX_ADDR       (uint8_t)0x10
//#define REG_RX_PW_P0      (uint8_t)0x11
//#define REG_RX_PW_P1      (uint8_t)0x12
//#define REG_RX_PW_P2      (uint8_t)0x13
//#define REG_RX_PW_P3      (uint8_t)0x14
//#define REG_RX_PW_P4      (uint8_t)0x15
//#define REG_RX_PW_P5      (uint8_t)0x16
//#define REG_FIFO_STATUS   (uint8_t)0x17
//#define REG_DYNPD         (uint8_t)0x1C
//#define REG_FEATURE       (uint8_t)0x1D

void nRF_Setup(t_nRF24L01 *p_nRF,
              void (*csnSetHi)(void),
              void (*csnSetLo)(void),
              void (*spiTransmit)(uint8_t *data, uint16_t size),
              void (*spiReceive)(uint8_t *data, uint16_t size))
{
  int i = 0;
  p_nRF->pollingCounter = 0;
  
  p_nRF->nRfConfigReg.addr = REG_CONFIG; // присвоение адреса регистра
  p_nRF->nRfConfigReg.reg_union = &p_nRF->nRfConfigStruct.byte; // присвоение объединения для расшифровки регистра
  p_nRF->PollingRegistersArray[i++] = &p_nRF->nRfConfigReg;  // добавление указателя регистра в массив для автоматического опроса регистров

  p_nRF->nRfEnAaReg.addr = REG_EN_AA;
  p_nRF->nRfEnAaReg.reg_union = &p_nRF->nRfEnAaStruct.byte;
  p_nRF->PollingRegistersArray[i++] = &p_nRF->nRfEnAaReg;

  p_nRF->nRfEnRxAddrReg.addr = REG_EN_RXADDR;
  p_nRF->nRfEnRxAddrReg.reg_union = &p_nRF->nRfEnRxAddrStruct.byte;
  p_nRF->PollingRegistersArray[i++] = &p_nRF->nRfEnRxAddrReg;

  p_nRF->nRfSetupAwReg.addr = REG_SETUP_AW;
  p_nRF->nRfSetupAwReg.reg_union = &p_nRF->nRfSetupAwStruct.byte;
  p_nRF->PollingRegistersArray[i++] = &p_nRF->nRfSetupAwReg;

  p_nRF->nRfSetupRetrReg.addr = REG_SETUP_REPR;
  p_nRF->nRfSetupRetrReg.reg_union = &p_nRF->nRfSetupRetrStruct.byte;
  p_nRF->PollingRegistersArray[i++] = &p_nRF->nRfSetupRetrReg;

  p_nRF->nRfRfChReg.addr = REG_RF_CH;
  p_nRF->nRfRfChReg.reg_union = &p_nRF->nRfRfChStruct.byte;
  p_nRF->PollingRegistersArray[i++] = &p_nRF->nRfRfChReg;

  p_nRF->nRfRfSetupReg.addr = REG_RF_SETUP;
  p_nRF->nRfRfSetupReg.reg_union = &p_nRF->nRfRfSetupStruct.byte;
  p_nRF->PollingRegistersArray[i++] = &p_nRF->nRfRfSetupReg;

  p_nRF->nRfStatusReg.addr = REG_STATUS;
  p_nRF->nRfStatusReg.reg_union = &p_nRF->nRfStatusStruct.byte;
  p_nRF->PollingRegistersArray[i++] = &p_nRF->nRfStatusReg;

  p_nRF->nRfObserveTxReg.addr = REG_OBSERVE_TX;
  p_nRF->nRfObserveTxReg.reg_union = &p_nRF->nRfObserveTxStruct.byte;
  p_nRF->PollingRegistersArray[i++] = &p_nRF->nRfObserveTxReg;

  p_nRF->nRfCdReg.addr = REG_CD;
  p_nRF->nRfCdReg.reg_union = &p_nRF->nRfCdStruct.byte;
  p_nRF->PollingRegistersArray[i++] = &p_nRF->nRfCdReg;

  p_nRF->csnSetHi = csnSetHi;
  p_nRF->csnSetLo = csnSetLo;
  p_nRF->spiReceive = spiReceive;
  p_nRF->spiTransmit = spiTransmit;

  nRfRegisterRead(p_nRF, &p_nRF->nRfConfigReg); // нужно произвести чтение, чтобы модуль ожил
// CONFIG 0x0A - 1010 - EN_CRC, PWR_UP

  p_nRF->nRfConfigStruct.CONFIG.EN_CRC = 1;
  p_nRF->nRfConfigStruct.CONFIG.PWR_UP = 1;
  nRfRegisterWrite(p_nRF, &p_nRF->nRfConfigReg);
// pause 5ms
// EN_AA 0x02 - 10 - ENAA_P1
  p_nRF->nRfEnAaStruct.byte = 0;
  p_nRF->nRfEnAaStruct.en_aa.ENAA_P1 = 1;
  nRfRegisterWrite(p_nRF, &p_nRF->nRfEnAaReg);
// REG_EN_RXADDR 0x02 - 10 - ERX_P1
// SETUP_AW 0x01 - RX/TX Address field width '01' - 3 bytes 
// SETUP_RETR 0x5F - 1011111 - ARD(Auto Retransmit Delay), ARC(Auto Retransmit Count)
// FEATURE 0
// DYNPD 0
// STATUS 0x70 - reset irq flags - 1110000 ?
// RF_SETUP 0x06 - 1Mbit, 0dBm - 110
// TX_ADDR адрес передатчика
// RX_ADDR_P1 - адрес приёмника
// RX_PW_P1 - Number of bytes in RX payload in data pipe 1 (1 to 32 bytes).
}

// функция опроса регистров
void nRfPollingRegisters(t_nRF24L01 *p_nRf)
{
  t_register *p_reg = p_nRf->PollingRegistersArray[p_nRf->pollingCounter];
  ++p_nRf->pollingCounter;
  if(p_nRf->pollingCounter >= REGISTERS_COUNT)
    p_nRf->pollingCounter = 0;
  nRfRegisterRead(p_nRf, p_reg);
}

void nRfRegisterRead(t_nRF24L01 *p_nRf, t_register *p_reg)
{
  p_nRf->csnSetLo();

  p_nRf->spiTransmit(&p_reg->addr, 1);
  p_nRf->spiReceive(p_reg->reg_union, 1);

  p_nRf->csnSetHi();
}

void nRfRegisterWrite(t_nRF24L01 *p_nRf, t_register *p_reg)
{
  uint8_t buf[2] = {0};

  buf[0] = CMD_W_REGISTER(p_reg->addr);
  buf[1] = *p_reg->reg_union;

  p_nRf->csnSetLo();

  p_nRf->spiTransmit(buf, 2);

  p_nRf->csnSetHi();
}
