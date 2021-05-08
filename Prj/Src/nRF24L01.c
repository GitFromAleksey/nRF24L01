#include "nRF24L01.h"
#include "stm32f1xx_hal.h"

#define PAYLOAD_DATA_LEN      32u

#define RECEIVE_BUFFER_SIZE   32u
uint8_t ReceiveBuffer[RECEIVE_BUFFER_SIZE];

void nRfRegisterRead(t_nRF24L01 *p_nRf, t_register *p_reg);
void nRfRegisterWrite(t_nRF24L01 *p_nRf, t_register *p_reg);
void nRf_WriteCMD(t_nRF24L01 *p_nRf, uint8_t cmd, uint8_t *p_data, uint8_t size);
void nRf_SwitchReceiveMode(t_nRF24L01 *p_nRf);

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
//-----------------------------------------------------------------------------
void nRF_RegistersInit(t_nRF24L01 *p_nRF)
{
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

  p_nRF->nRfRxPwP0Reg.addr = REG_RX_PW_P0;
  p_nRF->nRfRxPwP0Reg.reg_union = &p_nRF->nRfRxPwP0Struct.byte;
  p_nRF->nRfRxPwP0Reg.size = sizeof(p_nRF->nRfRxPwP0Struct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfRxPwP0Reg);

  p_nRF->nRfRxPwP1Reg.addr = REG_RX_PW_P1;
  p_nRF->nRfRxPwP1Reg.reg_union = &p_nRF->nRfRxPwP1Struct.byte;
  p_nRF->nRfRxPwP1Reg.size = sizeof(p_nRF->nRfRxPwP1Struct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfRxPwP1Reg);

  p_nRF->nRfDynpdReg.addr = REG_DYNPD;
  p_nRF->nRfDynpdReg.reg_union = &p_nRF->nRfDynpdStruct.byte;
  p_nRF->nRfDynpdReg.size = sizeof(p_nRF->nRfDynpdStruct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfDynpdReg);

  p_nRF->nRfFeaturesReg.addr = REG_FEATURE;
  p_nRF->nRfFeaturesReg.reg_union = &p_nRF->nRfFeaturesStruct.byte;
  p_nRF->nRfFeaturesReg.size = sizeof(p_nRF->nRfFeaturesStruct.byte);
  nRF_AddPollingRegister(p_nRF, &p_nRF->nRfFeaturesReg);
}
//-----------------------------------------------------------------------------
void nRf_AsMasterSetup(t_nRF24L01 *p_nRf)
{
// EN_AA 0x02 - 10 - ENAA_P0 - Enable ‘Auto Acknowledgment’
  p_nRf->nRfEnAaStruct.byte = 0;
  p_nRf->nRfEnAaStruct.en_aa.ENAA_P0 = 1;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfEnAaReg);
// REG_EN_RXADDR 0x02 - 10 - ERX_P0
  p_nRf->nRfEnRxAddrStruct.byte = 0;
  p_nRf->nRfEnRxAddrStruct.en_rxaddr.ERX_P0 = 1;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfEnRxAddrReg);
// RX_PW_P0 - Number of bytes in RX payload in data pipe 1 (1 to 32 bytes).
  p_nRf->nRfRxPwP0Struct.byte = 0;
  p_nRf->nRfRxPwP0Struct.RX_PW_Px.RX_PW_Px = PAYLOAD_DATA_LEN;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfRxPwP0Reg);

// RX_ADDR_P0 - адрес приёмника
  p_nRf->nRfRxAddrP0Struct.buf[0] = 0xAA;
  p_nRf->nRfRxAddrP0Struct.buf[1] = 0xA5;
  p_nRf->nRfRxAddrP0Struct.buf[2] = 0xA5;
  p_nRf->nRfRxAddrP0Struct.buf[3] = 0xA5;
  p_nRf->nRfRxAddrP0Struct.buf[4] = 0xA5;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfRxAddrP0Reg);
}
//-----------------------------------------------------------------------------
void nRf_AsSlaveSetup(t_nRF24L01 *p_nRf)
{
// EN_AA 0x02 - 10 - ENAA_P1 - Enable ‘Auto Acknowledgment’
  p_nRf->nRfEnAaStruct.byte = 0;
  p_nRf->nRfEnAaStruct.en_aa.ENAA_P1 = 1;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfEnAaReg);
// REG_EN_RXADDR 0x02 - 10 - ERX_P1
  p_nRf->nRfEnRxAddrStruct.byte = 0;
  p_nRf->nRfEnRxAddrStruct.en_rxaddr.ERX_P1 = 1;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfEnRxAddrReg);
// RX_PW_P1 - Number of bytes in RX payload in data pipe 1 (1 to 32 bytes).
  p_nRf->nRfRxPwP1Struct.byte = 0;
  p_nRf->nRfRxPwP1Struct.RX_PW_Px.RX_PW_Px = PAYLOAD_DATA_LEN;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfRxPwP1Reg);
  
// RX_ADDR_P1 - адрес приёмника
  p_nRf->nRfRxAddrP1Struct.buf[0] = 0xA5;
  p_nRf->nRfRxAddrP1Struct.buf[1] = 0xA5;
  p_nRf->nRfRxAddrP1Struct.buf[2] = 0xA5;
  p_nRf->nRfRxAddrP1Struct.buf[3] = 0xA5;
  p_nRf->nRfRxAddrP1Struct.buf[4] = 0xA5;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfRxAddrP1Reg);
}
//-----------------------------------------------------------------------------
void nRF_ModuleInit(t_nRF24L01 *p_nRf)
{
  // CE_RESET
  nRfRegisterRead(p_nRf, &p_nRf->nRfConfigReg); // нужно произвести чтение, чтобы модуль ожил
// CONFIG 0x0A - 1010 - EN_CRC, PWR_UP
  p_nRf->nRfConfigStruct.byte = 0;
  p_nRf->nRfConfigStruct.CONFIG.EN_CRC = 1;
  p_nRf->nRfConfigStruct.CONFIG.PWR_UP = 1;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfConfigReg);
// pause 5ms
  HAL_Delay(5);

// SETUP_AW 0x01 - RX/TX Address field width '01' - 3 bytes 
  p_nRf->nRfSetupAwStruct.byte = 0;
  p_nRf->nRfSetupAwStruct.SETUP_AW.AW = 3;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfSetupAwReg);
// SETUP_RETR 0x5F - 1011111 - ARD(Auto Retransmit Delay), ARC(Auto Retransmit Count)
  p_nRf->nRfSetupRetrStruct.byte = 0;
  p_nRf->nRfSetupRetrStruct.SETUP_RETR.ARD = 5;
  p_nRf->nRfSetupRetrStruct.SETUP_RETR.ARC = 15;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfSetupRetrReg);
// TX_ADDR адрес передатчика (0xb3,0xb4,0x01)
  p_nRf->nRfTxAddrStruct.buf[0] = 0xA5;
  p_nRf->nRfTxAddrStruct.buf[1] = 0xA5;
  p_nRf->nRfTxAddrStruct.buf[2] = 0xA5;
  p_nRf->nRfTxAddrStruct.buf[3] = 0xA5;
  p_nRf->nRfTxAddrStruct.buf[4] = 0xA5;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfTxAddrReg);
// ACTIVATE // эта настройка для модуля без плюса
//  tmp = 0x73;
//  nRf_WriteCMD(p_nRf, CMD_ACTIVATE, &tmp, 1);
// FEATURE 0
  p_nRf->nRfFeaturesStruct.byte = 0;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfFeaturesReg);
// DYNPD 0
  p_nRf->nRfDynpdStruct.byte = 0;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfDynpdReg);
// STATUS 0x70 - reset irq flags - 1110000 ?
  p_nRf->nRfStatusStruct.byte = 0;
  p_nRf->nRfStatusStruct.STATUS.TX_DS = 1;
  p_nRf->nRfStatusStruct.STATUS.RX_DR = 1;
  p_nRf->nRfStatusStruct.STATUS.MAX_RT = 1;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfStatusReg);
// RF_CH - 76 - частота 2476 МГц
  p_nRf->nRfRfChStruct.RF_CH.RF_CH = 76;//76;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfRfChReg);
// RF_SETUP 0x06 - 1Mbit, 0dBm - 110
  p_nRf->nRfRfSetupStruct.byte = 0;
  p_nRf->nRfRfSetupStruct.RF_SETUP.RF_DR_HIGH = 0;
  p_nRf->nRfRfSetupStruct.RF_SETUP.RF_PWR = 0;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfRfSetupReg);

  nRf_SwitchReceiveMode(p_nRf);
}
//-----------------------------------------------------------------------------
void nRF_Setup(t_nRF24L01 *p_nRf, bool master,
              void (*ceSetHi)(void),
              void (*ceSetLo)(void),
              void (*csnSetHi)(void),
              void (*csnSetLo)(void),
              void (*spiTransmit)(uint8_t *data, uint16_t size),
              void (*spiReceive)(uint8_t *data, uint16_t size),
              void (*ReceiveEventCallback)(uint8_t *data, uint16_t size))
{
  nRF_RegistersInit(p_nRf);

  p_nRf->isMaster = master;

  p_nRf->ceSetHi = ceSetHi;
  p_nRf->ceSetLo = ceSetLo;
  
  p_nRf->csnSetHi = csnSetHi;
  p_nRf->csnSetLo = csnSetLo;
  p_nRf->spiReceive = spiReceive;
  p_nRf->spiTransmit = spiTransmit;
  p_nRf->ReceiveEventCallback = ReceiveEventCallback;

  p_nRf->csnSetHi();
  p_nRf->ceSetLo();

  nRF_ModuleInit(p_nRf);
  
  if(master)
    nRf_AsMasterSetup(p_nRf);
  else
    nRf_AsSlaveSetup(p_nRf);
}
//-----------------------------------------------------------------------------
void nRf_ReadCMD(t_nRF24L01 *p_nRf, uint8_t cmd, uint8_t *p_data, uint8_t size)
{
  p_nRf->csnSetLo();
  p_nRf->spiTransmit(&cmd, 1);
  if(p_data != NULL)
  {
    p_nRf->spiReceive(p_data, size);
  }
  p_nRf->csnSetHi();
}
//-----------------------------------------------------------------------------
void nRf_WriteCMD(t_nRF24L01 *p_nRf, uint8_t cmd, uint8_t *p_data, uint8_t size)
{
  p_nRf->csnSetLo();
  p_nRf->spiTransmit(&cmd, 1);
//  HAL_Delay(1);
  if(p_data != NULL)
  {
    p_nRf->spiTransmit(p_data, size);
  }
  p_nRf->csnSetHi();
}
//-----------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------
void nRf_SwitchReceiveMode(t_nRF24L01 *p_nRf)
{
  nRfRegisterRead(p_nRf, &p_nRf->nRfConfigReg);

  p_nRf->nRfConfigStruct.CONFIG.PWR_UP = 1;
  p_nRf->nRfConfigStruct.CONFIG.PRIM_RX = 1;

  nRfRegisterWrite(p_nRf, &p_nRf->nRfConfigReg);

  p_nRf->ceSetHi();
  HAL_Delay(1);

// FLUSH_RX
  nRf_WriteCMD(p_nRf, CMD_FLUSH_RX, NULL, 0);
// FLUSH_TX
  nRf_WriteCMD(p_nRf, CMD_FLUSH_TX, NULL, 0);
}
//-----------------------------------------------------------------------------
void nRf_SwitchTransmitMode(t_nRF24L01 *p_nRf)
{
  nRfRegisterRead(p_nRf, &p_nRf->nRfConfigReg);

  p_nRf->nRfConfigStruct.CONFIG.PWR_UP = 1;
  p_nRf->nRfConfigStruct.CONFIG.PRIM_RX = 0;

  nRfRegisterWrite(p_nRf, &p_nRf->nRfConfigReg);

  p_nRf->ceSetHi();
  HAL_Delay(1);

// FLUSH_RX
  nRf_WriteCMD(p_nRf, CMD_FLUSH_RX, NULL, 0);
// FLUSH_TX
  nRf_WriteCMD(p_nRf, CMD_FLUSH_TX, NULL, 0);
}
//-----------------------------------------------------------------------------
void nRf_Send(t_nRF24L01 *p_nRf, uint8_t *p_buf, uint8_t size)
{
  nRfRegisterRead(p_nRf, &p_nRf->nRfStatusReg);
//  if(p_nRf->nRfStatusStruct.STATUS.TX_DS == 1)
//    return;
  p_nRf->ceSetLo();
// FLUSH_RX
  nRf_WriteCMD(p_nRf, CMD_FLUSH_RX, NULL, 0);
// FLUSH_TX
  nRf_WriteCMD(p_nRf, CMD_FLUSH_TX, NULL, 0);

// CONFIG set PWR_UP, reset PRIM_RX
  nRfRegisterRead(p_nRf, &p_nRf->nRfConfigReg);
  p_nRf->nRfConfigStruct.CONFIG.PWR_UP = 1;
  p_nRf->nRfConfigStruct.CONFIG.PRIM_RX = 0;
//  p_nRf->nRfConfigStruct.CONFIG.EN_CRC = 1;
  nRfRegisterWrite(p_nRf, &p_nRf->nRfConfigReg);
// delay 150 us
  HAL_Delay(1);
//  nRf_Transmit(p_nRf, CMD_W_TX_PAYLOAD, p_buf, size);
  nRf_WriteCMD(p_nRf, CMD_W_TX_PAYLOAD, p_buf, size);
// CE_SET
  p_nRf->ceSetHi();
// delay 10us
  HAL_Delay(1);
// CE_RESET
  p_nRf->ceSetLo();
}
//-----------------------------------------------------------------------------
void nRfRegisterRead(t_nRF24L01 *p_nRf, t_register *p_reg)
{
  p_nRf->csnSetLo();

  p_nRf->spiTransmit(&p_reg->addr, 1);
  p_nRf->spiReceive(p_reg->reg_union, p_reg->size);

  p_nRf->csnSetHi();
}
//-----------------------------------------------------------------------------
void nRfRegisterWrite(t_nRF24L01 *p_nRf, t_register *p_reg)
{
  uint8_t buf = 0;

  buf = CMD_W_REGISTER(p_reg->addr);

  p_nRf->csnSetLo();

  p_nRf->spiTransmit(&buf, 1);
  p_nRf->spiTransmit(p_reg->reg_union, p_reg->size);

  p_nRf->csnSetHi();
}
//-----------------------------------------------------------------------------
void nRf_RUN(t_nRF24L01 *p_nRf)
{
  nRfPollingRegisters(p_nRf);

  nRfRegisterRead(p_nRf, &p_nRf->nRfStatusReg);

  if(p_nRf->nRfConfigStruct.CONFIG.PRIM_RX == 0) // передатчик
  {
    if(p_nRf->nRfStatusStruct.STATUS.TX_DS == 1)
    {
      p_nRf->ceSetLo();
      p_nRf->nRfStatusStruct.STATUS.TX_DS = 1;
      nRfRegisterWrite(p_nRf, &p_nRf->nRfStatusReg);
      nRf_WriteCMD(p_nRf, CMD_FLUSH_TX, NULL, 0);
    }
    if(p_nRf->nRfStatusStruct.STATUS.MAX_RT == 1)
    {
      p_nRf->nRfStatusStruct.STATUS.MAX_RT = 1;
      nRfRegisterWrite(p_nRf, &p_nRf->nRfStatusReg);
    }
  }
  
  if(p_nRf->nRfConfigStruct.CONFIG.PRIM_RX == 1) // приёмник
  {
    if(p_nRf->nRfStatusStruct.STATUS.RX_P_NO == 1)
    {
      p_nRf->ceSetLo();
      if(p_nRf->nRfStatusStruct.STATUS.RX_DR == 1)
      {
        nRf_ReadCMD(p_nRf, CMD_R_RX_PAYLOAD, ReceiveBuffer, PAYLOAD_DATA_LEN);
        p_nRf->nRfStatusStruct.STATUS.RX_DR = 0;
        nRfRegisterWrite(p_nRf, &p_nRf->nRfStatusReg);
        p_nRf->ReceiveEventCallback(ReceiveBuffer, PAYLOAD_DATA_LEN);//
      }
      p_nRf->ceSetHi();
    }
  }
}
//-----------------------------------------------------------------------------
