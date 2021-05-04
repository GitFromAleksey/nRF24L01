#include "nRF24L01.h"


// добавляет регистр в список для автоматического опроса регистров
void nRF_AddPollingRegister(t_nRF24L01 *p_nRF, t_register *p_reg)
{
  t_register *tmp;
  if(p_nRF->PollingRegistersList == NULL)
  {
    p_nRF->PollingRegistersList = p_reg;
    p_reg->next_register = NULL;
    p_nRF->PollingCurrentRegister = p_nRF->PollingRegistersList;
  }
  else
  {
    tmp = p_nRF->PollingRegistersList;
    while(tmp->next_register != NULL)
      tmp = tmp->next_register;
    tmp->next_register = p_reg;
    p_reg->next_register = NULL;
  }
}

void nRF_Setup(t_nRF24L01 *p_nRF,
              void (*csnSetHi)(void),
              void (*csnSetLo)(void),
              void (*spiTransmit)(uint8_t *data, uint16_t size),
              void (*spiReceive)(uint8_t *data, uint16_t size))
{
  int i = 0;
//  p_nRF->pollingCounter = 0;
  p_nRF->PollingRegistersList = NULL;
  
  p_nRF->nRfConfigReg.addr = REG_CONFIG; // присвоение адреса регистра
  p_nRF->nRfConfigReg.reg_union = &p_nRF->nRfConfigStruct.byte; // присвоение объединения для расшифровки регистра
  p_nRF->nRfConfigReg.size = sizeof(p_nRF->nRfConfigStruct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfConfigReg);// добавление указателя регистра в список для автоматического опроса регистров

  p_nRF->nRfEnAaReg.addr = REG_EN_AA;
  p_nRF->nRfEnAaReg.reg_union = &p_nRF->nRfEnAaStruct.byte;
  p_nRF->nRfEnAaReg.size = sizeof(p_nRF->nRfEnAaStruct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfEnAaReg);

  p_nRF->nRfEnRxAddrReg.addr = REG_EN_RXADDR;
  p_nRF->nRfEnRxAddrReg.reg_union = &p_nRF->nRfEnRxAddrStruct.byte;
  p_nRF->nRfEnRxAddrReg.size = sizeof(p_nRF->nRfEnRxAddrStruct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfEnRxAddrReg);

  p_nRF->nRfSetupAwReg.addr = REG_SETUP_AW;
  p_nRF->nRfSetupAwReg.reg_union = &p_nRF->nRfSetupAwStruct.byte;
  p_nRF->nRfSetupAwReg.size = sizeof(p_nRF->nRfSetupAwStruct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfSetupAwReg);

  p_nRF->nRfSetupRetrReg.addr = REG_SETUP_REPR;
  p_nRF->nRfSetupRetrReg.reg_union = &p_nRF->nRfSetupRetrStruct.byte;
  p_nRF->nRfSetupRetrReg.size = sizeof(p_nRF->nRfSetupRetrStruct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfSetupRetrReg);

  p_nRF->nRfRfChReg.addr = REG_RF_CH;
  p_nRF->nRfRfChReg.reg_union = &p_nRF->nRfRfChStruct.byte;
  p_nRF->nRfRfChReg.size = sizeof(p_nRF->nRfRfChStruct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfRfChReg);

  p_nRF->nRfRfSetupReg.addr = REG_RF_SETUP;
  p_nRF->nRfRfSetupReg.reg_union = &p_nRF->nRfRfSetupStruct.byte;
  p_nRF->nRfRfSetupReg.size = sizeof(p_nRF->nRfRfSetupStruct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfRfSetupReg);

  p_nRF->nRfStatusReg.addr = REG_STATUS;
  p_nRF->nRfStatusReg.reg_union = &p_nRF->nRfStatusStruct.byte;
  p_nRF->nRfStatusReg.size = sizeof(p_nRF->nRfStatusStruct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfStatusReg);

  p_nRF->nRfObserveTxReg.addr = REG_OBSERVE_TX;
  p_nRF->nRfObserveTxReg.reg_union = &p_nRF->nRfObserveTxStruct.byte;
  p_nRF->nRfObserveTxReg.size = sizeof(p_nRF->nRfObserveTxStruct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfObserveTxReg);

  p_nRF->nRfCdReg.addr = REG_CD;
  p_nRF->nRfCdReg.reg_union = &p_nRF->nRfCdStruct.byte;
  p_nRF->nRfCdReg.size = sizeof(p_nRF->nRfCdStruct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfCdReg);
  
  p_nRF->nRfRxAddrP0Reg.addr = REG_RX_ADDR_P0;
  p_nRF->nRfRxAddrP0Reg.reg_union = p_nRF->nRfRxAddrP0Struct.buf;
  p_nRF->nRfRxAddrP0Reg.size = sizeof(p_nRF->nRfRxAddrP0Struct.buf);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfRxAddrP0Reg);

  p_nRF->nRfRxAddrP1Reg.addr = REG_RX_ADDR_P1;
  p_nRF->nRfRxAddrP1Reg.reg_union = p_nRF->nRfRxAddrP1Struct.buf;
  p_nRF->nRfRxAddrP1Reg.size = sizeof(p_nRF->nRfRxAddrP1Struct.buf);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfRxAddrP1Reg);
  
  p_nRF->nRfTxAddrReg.addr = REG_TX_ADDR;
  p_nRF->nRfTxAddrReg.reg_union = p_nRF->nRfTxAddrStruct.buf;
  p_nRF->nRfTxAddrReg.size = sizeof(p_nRF->nRfTxAddrStruct.buf);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfTxAddrReg);

  p_nRF->nRfRxPwP1Reg.addr = REG_RX_PW_P1;
  p_nRF->nRfRxPwP1Reg.reg_union = &p_nRF->nRfRxPwP1Struct.byte;
  p_nRF->nRfRxPwP1Reg.size = sizeof(p_nRF->nRfRxPwP1Struct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfRxPwP1Reg);



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
  p_nRF->nRfEnRxAddrStruct.byte = 0;
  p_nRF->nRfEnRxAddrStruct.en_rxaddr.ERX_P1 = 1;
  nRfRegisterWrite(p_nRF, &p_nRF->nRfEnRxAddrReg);
// SETUP_AW 0x01 - RX/TX Address field width '01' - 3 bytes 
  p_nRF->nRfSetupAwStruct.byte = 0;
  p_nRF->nRfSetupAwStruct.SETUP_AW.AW = 0x01;
  nRfRegisterWrite(p_nRF, &p_nRF->nRfSetupAwReg);
// SETUP_RETR 0x5F - 1011111 - ARD(Auto Retransmit Delay), ARC(Auto Retransmit Count)
  p_nRF->nRfSetupRetrStruct.byte = 0;
  p_nRF->nRfSetupRetrStruct.SETUP_RETR.ARD = 5;
  p_nRF->nRfSetupRetrStruct.SETUP_RETR.ARC = 15;
  nRfRegisterWrite(p_nRF, &p_nRF->nRfSetupRetrReg);
// FEATURE 0
// DYNPD 0
// STATUS 0x70 - reset irq flags - 1110000 ?
  p_nRF->nRfStatusStruct.byte = 0;
  p_nRF->nRfStatusStruct.STATUS.TX_DS = 1;
  p_nRF->nRfStatusStruct.STATUS.RX_DR = 1;
  p_nRF->nRfStatusStruct.STATUS.MAX_RT = 1;
  nRfRegisterWrite(p_nRF, &p_nRF->nRfStatusReg);
// RF_SETUP 0x06 - 1Mbit, 0dBm - 110
  p_nRF->nRfRfSetupStruct.byte = 0;
  p_nRF->nRfRfSetupStruct.RF_SETUP.RF_DR = 0;
  p_nRF->nRfRfSetupStruct.RF_SETUP.RF_PWR = 3;
  nRfRegisterWrite(p_nRF, &p_nRF->nRfRfSetupReg);
// TX_ADDR адрес передатчика
  p_nRF->nRfTxAddrStruct.buf[0] = 0x01;
  p_nRF->nRfTxAddrStruct.buf[1] = 0x02;
  p_nRF->nRfTxAddrStruct.buf[2] = 0x03;
  p_nRF->nRfTxAddrStruct.buf[3] = 0x04;
  p_nRF->nRfTxAddrStruct.buf[4] = 0x05;
  nRfRegisterWrite(p_nRF, &p_nRF->nRfTxAddrReg);
// RX_ADDR_P1 - адрес приёмника
  p_nRF->nRfRxAddrP1Struct.buf[0] = 0x10;
  p_nRF->nRfRxAddrP1Struct.buf[1] = 0x20;
  p_nRF->nRfRxAddrP1Struct.buf[2] = 0x30;
  p_nRF->nRfRxAddrP1Struct.buf[3] = 0x40;
  p_nRF->nRfRxAddrP1Struct.buf[4] = 0x50;
  nRfRegisterWrite(p_nRF, &p_nRF->nRfRxAddrP1Reg);
// RX_PW_P1 - Number of bytes in RX payload in data pipe 1 (1 to 32 bytes).
  p_nRF->nRfRxPwP1Struct.byte = 0;
  p_nRF->nRfRxPwP1Struct.RX_PW_Px.RX_PW_Px = 32;
  nRfRegisterWrite(p_nRF, &p_nRF->nRfRxPwP1Reg);
}

// функция автоматического опроса регистров из списка
void nRfPollingRegisters(t_nRF24L01 *p_nRf)
{
  t_register *p_reg = p_nRf->PollingCurrentRegister;
  if(p_nRf->PollingCurrentRegister->next_register == NULL)
    p_nRf->PollingCurrentRegister = p_nRf->PollingRegistersList;
  else
    p_nRf->PollingCurrentRegister = p_nRf->PollingCurrentRegister->next_register;
  nRfRegisterRead(p_nRf, p_reg);
}

void nRfRegisterRead(t_nRF24L01 *p_nRf, t_register *p_reg)
{
  p_nRf->csnSetLo();

  p_nRf->spiTransmit(&p_reg->addr, 1);
  p_nRf->spiReceive(p_reg->reg_union, p_reg->size);

  p_nRf->csnSetHi();
}

void nRfRegisterWrite(t_nRF24L01 *p_nRf, t_register *p_reg)
{
  uint8_t buf = 0;

  buf = CMD_W_REGISTER(p_reg->addr);

  p_nRf->csnSetLo();

  p_nRf->spiTransmit(&buf, 1);
  p_nRf->spiTransmit(p_reg->reg_union, p_reg->size);

  p_nRf->csnSetHi();
}

