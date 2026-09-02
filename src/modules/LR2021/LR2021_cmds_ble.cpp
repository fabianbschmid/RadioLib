#include "LR2021.h"

#include "../LR11x0/LR_common.h"
#include "LR2021_registers.h"

#include <string.h>
#include <math.h>

#if !RADIOLIB_EXCLUDE_LR2021

int16_t LR2021::setBleModulationParams(uint8_t phy, uint8_t rxBw) {
  uint8_t buff[] = { phy, rxBw };

  // when the Rx bandwidth is left on automatic, the parameter can be omitted entirely -
  // this is what the Semtech driver does, and it keeps the command byte-identical to it
  size_t len = (rxBw == RADIOLIB_LR2021_BLE_RX_BW_AUTO) ? 1 : sizeof(buff);
  int16_t state = this->SPIcommand(RADIOLIB_LR2021_CMD_SET_BLE_MODULATION_PARAMS, true, buff, len);
  RADIOLIB_ASSERT(state);

  // the default preamble length is wrong for LE 2M and has to be patched on every
  // call - the Semtech driver applies this from set_modulation_params as well
  if(phy == RADIOLIB_LR2021_BLE_PHY_2M) {
    state = bleFixPreamble2M();
  }
  return(state);
}

int16_t LR2021::setBleChannelParams(uint8_t channelType, uint8_t whiteningInit, uint32_t crcInit, uint32_t syncWord, bool crcInFifo) {
  uint8_t buff[] = {
    (uint8_t)((crcInFifo ? RADIOLIB_LR2021_BLE_CRC_IN_FIFO_ON : RADIOLIB_LR2021_BLE_CRC_IN_FIFO_OFF) | (channelType & 0x0F)),
    whiteningInit,
    (uint8_t)((crcInit >> 16) & 0xFF), (uint8_t)((crcInit >> 8) & 0xFF), (uint8_t)(crcInit & 0xFF),
    (uint8_t)((syncWord >> 24) & 0xFF), (uint8_t)((syncWord >> 16) & 0xFF),
    (uint8_t)((syncWord >> 8) & 0xFF), (uint8_t)(syncWord & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR2021_CMD_SET_BLE_CHANNEL_PARAMS, true, buff, sizeof(buff)));
}

int16_t LR2021::setBleTxPduLen(uint8_t len) {
  return(this->SPIcommand(RADIOLIB_LR2021_CMD_SET_BLE_PDU_LEN, true, &len, sizeof(len)));
}

int16_t LR2021::setBleTx(uint8_t len) {
  return(this->SPIcommand(RADIOLIB_LR2021_CMD_SET_BLE_TX, true, &len, sizeof(len)));
}

int16_t LR2021::getBleRxStats(uint16_t* packetRx, uint16_t* crcError, uint16_t* lenError) {
  uint8_t buff[6] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR2021_CMD_GET_BLE_RX_STATS, false, buff, sizeof(buff));
  if(packetRx) { *packetRx = ((uint16_t)(buff[0]) << 8) | (uint16_t)buff[1]; }
  if(crcError) { *crcError = ((uint16_t)(buff[2]) << 8) | (uint16_t)buff[3]; }
  if(lenError) { *lenError = ((uint16_t)(buff[4]) << 8) | (uint16_t)buff[5]; }
  return(state);
}

int16_t LR2021::getBlePacketStatus(uint16_t* payloadLen, float* rssiAvg, float* rssiSync, float* lqi) {
  uint8_t buff[6] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR2021_CMD_GET_BLE_PACKET_STATUS, false, buff, sizeof(buff));
  if(payloadLen) { *payloadLen = ((uint16_t)(buff[0]) << 8) | (uint16_t)buff[1]; }

  // both RSSI values are reported as 9-bit numbers in half-dB steps, with the
  // low bit split off into the flag byte
  uint16_t raw;
  if(rssiAvg) {
    raw = ((uint16_t)buff[2] << 1) | ((buff[4] >> 2) & 0x01);
    *rssiAvg = (float)raw / -2.0f;
  }
  if(rssiSync) {
    raw = ((uint16_t)buff[3] << 1) | (buff[4] & 0x01);
    *rssiSync = (float)raw / -2.0f;
  }

  // link quality indicator is the margin to the detection level in 0.25 dB steps
  if(lqi) { *lqi = (float)buff[5] * 0.25f; }
  return(state);
}

uint8_t LR2021::bleWhiteningInit(uint8_t channel) {
  uint8_t lfsr = 0x40 | (channel & 0x3F);
  uint8_t init = 0;
  for(uint8_t i = 0; i < 7; i++) {
    if(lfsr & (1 << i)) { init |= (uint8_t)(1 << (6 - i)); }
  }
  return(init);
}

int16_t LR2021::bleFixCodedSyncWord() {
  // replayed verbatim from lr20xx_workarounds_bluetooth_le_phy_coded_syncwords();
  // without it not every access address is detected in the LE Coded PHYs
  uint8_t buff[] = { 0x01, 0x20, 0x00, 0x09, 0x00 };
  return(this->SPIcommand(RADIOLIB_LR2021_CMD_INTERNAL_PARAM_WRITE, true, buff, sizeof(buff)));
}

int16_t LR2021::bleFixCodedFreqDrift() {
  // replayed from lr20xx_workarounds_bluetooth_le_phy_coded_frequency_drift();
  // the default drift setting costs sensitivity against high drift transmitters
  return(this->writeRegMemMask32(RADIOLIB_LR2021_REG_BLE_PHY_CODED_FREQ_DRIFT, (0x1FUL << 5), (30UL << 5)));
}

int16_t LR2021::bleFixPreamble2M() {
  // replayed verbatim from lr20xx_workarounds_bluetooth_le_2mbps_preamble_length();
  // the chip default preamble length is incorrect for LE 2M
  uint8_t buff[] = { 0x01, 0x21, 0x00, 0x07, 0x00 };
  return(this->SPIcommand(RADIOLIB_LR2021_CMD_INTERNAL_PARAM_WRITE, true, buff, sizeof(buff)));
}

#endif
