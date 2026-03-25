/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2024 Damien Maguire <info@evbmw.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Written purely by Claude Code

#include "BmwSmeBms.h"
#include "my_math.h"
#include "params.h"
#include "stm32_can.h"

/*
 * BMW OEM SME BMS implementation.
 *
 * Receives broadcast messages from the SME for battery monitoring.
 * Polls pack voltage via single-frame UDS requests on 0x6F1/0x607.
 * Sends 0x12F terminal status keepalive every 100ms.
 *
 * Contactor control (0x10B) is handled by BmwSmeContactor (ShuntType).
 *
 * CAN message map:
 *   RX 0x112 (20ms)  - Pack current, emergency flags
 *   RX 0x1FA (1s)    - ISO status, temperatures, system health
 *   RX 0x2F5 (100ms) - Charge/discharge limits
 *   RX 0x40D (1s)    - Available charge/discharge power
 *   RX 0x431 (200ms) - ISO measurement status, energy content
 *   RX 0x432 (200ms) - SOC, operating mode
 *   RX 0x607         - UDS responses (voltage, etc.)
 *   TX 0x12F (100ms) - Terminal status keepalive
 *   TX 0x6F1         - UDS requests (voltage polling)
 */

// SAE J1850 ZERO CRC8 lookup table
// Polynomial: 0x1D, used with init value 0x3F for BMW SME messages
static const uint8_t saeJ1850CrcTable[256] = {
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF,
    0x9C, 0x81, 0xA6, 0xBB, 0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E,
    0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76, 0x87, 0x9A, 0xBD, 0xA0,
    0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85,
    0xD6, 0xCB, 0xEC, 0xF1, 0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40,
    0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8, 0xDE, 0xC3, 0xE4, 0xF9,
    0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B,
    0x08, 0x15, 0x32, 0x2F, 0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A,
    0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2, 0x26, 0x3B, 0x1C, 0x01,
    0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24,
    0x77, 0x6A, 0x4D, 0x50, 0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2,
    0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A, 0x6C, 0x71, 0x56, 0x4B,
    0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA,
    0xA9, 0xB4, 0x93, 0x8E, 0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB,
    0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43, 0xB2, 0xAF, 0x88, 0x95,
    0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0,
    0xE3, 0xFE, 0xD9, 0xC4,
};

uint8_t BmwSmeBms::calcCrc(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x3F; // SAE J1850 ZERO init value for BMW SME
  for (uint8_t i = 0; i < len; i++) {
    crc = saeJ1850CrcTable[crc ^ data[i]];
  }
  return crc;
}

void BmwSmeBms::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x112); // Current, emergency flags (20ms)
  can->RegisterUserMessage(0x1FA); // ISO status, temps (1s)
  can->RegisterUserMessage(0x2F5); // Charge/discharge limits (100ms)
  can->RegisterUserMessage(0x40D); // Available power (1s)
  can->RegisterUserMessage(0x431); // ISO meas status (200ms)
  can->RegisterUserMessage(0x432); // SOC (200ms)
  can->RegisterUserMessage(0x607); // UDS responses
}

void BmwSmeBms::DecodeCAN(int id, uint8_t *data) {
  switch (id) {
  case 0x112:
    handle112(data);
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;
    break;
  case 0x1FA:
    handle1FA(data);
    break;
  case 0x2F5:
    handle2F5(data);
    break;
  case 0x40D:
    handle40D(data);
    break;
  case 0x431:
    handle431(data);
    break;
  case 0x432:
    handle432(data);
    break;
  case 0x607:
    handle607(data);
    break;
  }
}

// 0x112: Pack current and emergency flags (20ms cycle)
// Bytes 0-1: current = (byte[1] << 8 | byte[0]) - 8192, unit: deciAmps
// Byte 5: contactor open request bits [7:6]
// Byte 6: emergency flags
void BmwSmeBms::handle112(uint8_t *data) {
  int16_t raw = (int16_t)((data[1] << 8) | data[0]);
  packCurrent = raw - 8192; // deciAmps, positive = charge
  emergencyFlags = data[6];
}

// 0x1FA: ISO status, temperatures, system health (1s cycle)
// Byte 0 bits [1:0]: ISO error external, [3:2]: ISO error internal
// Byte 2 bits [7:6]: Isolation warning
// Byte 6: cell temp min = byte - 50 (deg C)
// Byte 7: cell temp max = byte - 50 (deg C)
void BmwSmeBms::handle1FA(uint8_t *data) {
  isoStatusByte0 = data[0];
  isoStatusByte2 = data[2];
  tempMin = (int8_t)(data[6] - 50);
  tempMax = (int8_t)(data[7] - 50);
}

// 0x2F5: Charge/discharge voltage and current limits (100ms cycle)
// Bytes 0-1: max charge voltage (LE 16-bit, 0.1V)
// Bytes 2-3: max charge current ((LE 16-bit) - 8192, deciAmps)
// Bytes 4-5: min discharge voltage (LE 16-bit, 0.1V)
// Bytes 6-7: max discharge current ((LE 16-bit) - 8192, deciAmps)
void BmwSmeBms::handle2F5(uint8_t *data) {
  maxChargeVoltage = (uint16_t)((data[1] << 8) | data[0]);
  maxChargeCurrent = (int16_t)((data[3] << 8) | data[2]) - 8192;
  minDischargeVoltage = (uint16_t)((data[5] << 8) | data[4]);
  maxDischargeCurrent = (int16_t)((data[7] << 8) | data[6]) - 8192;
}

// 0x40D: Available charge/discharge power (1s cycle)
// Bytes 0-1: short-term charge power (LE 16-bit, x3 W)
// Bytes 2-3: short-term discharge power (LE 16-bit, x3 W)
void BmwSmeBms::handle40D(uint8_t *data) {
  chargePowerShort = (uint16_t)((data[1] << 8) | data[0]);
  dischargePowerShort = (uint16_t)((data[3] << 8) | data[2]);
}

// 0x431: ISO measurement status, energy content (200ms cycle)
// Byte 0 bits [3:2]: isolation measurement status (2-bit enum)
void BmwSmeBms::handle431(uint8_t *data) {
  isoMeasStatus = (data[0] >> 2) & 0x03;
}

// 0x432: SOC and operating mode (200ms cycle)
// Byte 4: display SOC in %
void BmwSmeBms::handle432(uint8_t *data) { soc = data[4]; }

// 0x607: UDS response from SME
// Extended addressing: byte 0 = source address (0xF1 from SME)
// Byte 1: ISO-TP single frame PCI (0x05 = 5 data bytes)
// Bytes 2+: service response data
void BmwSmeBms::handle607(uint8_t *data) {
  // Verify extended address from SME
  if (data[0] != 0xF1)
    return;

  // Check for positive response to ReadDataByIdentifier (0x62)
  if (data[2] != 0x62)
    return;

  uint16_t did = (uint16_t)((data[3] << 8) | data[4]);
  uint16_t value = (uint16_t)((data[5] << 8) | data[6]);

  switch (did) {
  case 0xDDB4: // Pre-contactor voltage (0.1V resolution)
    batteryVoltage = (float)value * 0.1f;
    break;
  case 0xDD66: // Post-contactor voltage (0.1V resolution)
    postContactorVoltage = (float)value * 0.1f;
    break;
  }
}

// Send 0x12F terminal status keepalive (100ms cycle)
// Byte 0: CRC8 over bytes 1-7
// Byte 1: 0x20 + alive counter (0-14)
// Bytes 2-7: static terminal status values
void BmwSmeBms::sendKeepalive12F() {
  uint8_t frame[8];
  frame[1] = 0x20 + (aliveCounter12F & 0x0F);
  frame[2] = 0x86;
  frame[3] = 0x1B;
  frame[4] = 0xF1;
  frame[5] = 0x35;
  frame[6] = 0x30;
  frame[7] = 0x02;
  frame[0] = calcCrc(&frame[1], 7);

  can->Send(0x12F, (uint32_t *)frame, 8);

  aliveCounter12F++;
  if (aliveCounter12F > 14)
    aliveCounter12F = 0;
}

// Send single-frame UDS request for a 2-byte DID
// Uses extended addressing (byte 0 = 0x07 target address)
void BmwSmeBms::sendUdsRequest(uint16_t did) {
  uint8_t frame[8] = {0};
  frame[0] = 0x07;                  // Extended addressing: target SME
  frame[1] = 0x03;                  // Single frame, 3 data bytes
  frame[2] = 0x22;                  // ReadDataByIdentifier service
  frame[3] = (uint8_t)(did >> 8);   // DID high byte
  frame[4] = (uint8_t)(did & 0xFF); // DID low byte
  can->Send(0x6F1, (uint32_t *)frame, 8);
}

void BmwSmeBms::Task100Ms() {
  // Decrement timeout counter
  if (timeoutCounter > 0)
    timeoutCounter--;

  // Send terminal status keepalive
  sendKeepalive12F();

  // Poll voltage via UDS, alternating between pre and post contactor
  if (udsPollState == 0) {
    sendUdsRequest(0xDDB4); // Pre-contactor voltage
  } else {
    sendUdsRequest(0xDD66); // Post-contactor voltage
  }
  udsPollState = (udsPollState + 1) % 2;

  // Set voltage parameters
  // udc2 = pre-contactor (battery) voltage
  // udc = post-contactor (DC link) voltage — used by precharge check
  if (batteryVoltage < 450) {
    Param::SetFloat(Param::udc2, batteryVoltage);
  }
  if (batteryVoltage > 200) {
    Param::SetFloat(Param::udcsw, batteryVoltage - 20);
  }
  Param::SetFloat(Param::udc, postContactorVoltage);

  // Set current (deciAmps to Amps)
  float currentAmps = (float)packCurrent / 10.0f;

  // Set SOC
  Param::SetFloat(Param::SOC, (float)soc);

  // Set temperatures
  Param::SetFloat(Param::BMS_Tmin, (float)tempMin);
  Param::SetFloat(Param::BMS_Tmax, (float)tempMax);

  // Set charge/discharge limits
  Param::SetInt(Param::BMS_ChargeLim, MaxChargeCurrent());

  // Set power limits (x3 W to kW)
  Param::SetInt(Param::BMS_MaxInput,
                (int)((float)chargePowerShort * 3.0f / 1000.0f));
  Param::SetInt(Param::BMS_MaxOutput,
                (int)((float)dischargePowerShort * 3.0f / 1000.0f));

  // Isolation monitoring: check 0x1FA flags
  // Byte 0 bits [1:0] = ISO error external, bits [3:2] = ISO error internal
  // Byte 2 bits [7:6] = isolation warning
  // Value 0x02 (binary 10) in any 2-bit field = active fault
  uint8_t isoExternal = isoStatusByte0 & 0x03;
  uint8_t isoInternal = (isoStatusByte0 >> 2) & 0x03;
  uint8_t isoWarning = (isoStatusByte2 >> 6) & 0x03;

  bool isoFault = (isoExternal == 0x02) || (isoInternal == 0x02) ||
                  (isoWarning == 0x02);

  if (isoFault) {
    Param::SetInt(Param::BMS_Isolation, 0); // 0 = fault
  } else {
    Param::SetInt(Param::BMS_Isolation, 9999); // OK (no kOhm value without
                                               // multi-frame UDS)
  }

  // Set current and power only if BMS data is valid
  if (timeoutCounter > 0) {
    Param::SetFloat(Param::idc, currentAmps);
    float kw = batteryVoltage * currentAmps / 1000.0f;
    Param::SetFloat(Param::power, kw);
  } else {
    Param::SetFloat(Param::idc, 0);
    Param::SetFloat(Param::udcsw, 500); // Reset to high value on timeout
  }
}

float BmwSmeBms::MaxChargeCurrent() {
  // No charge if BMS timed out
  if (timeoutCounter < 1)
    return 0;

  // No charge if emergency flags set (bits [1:0] or [3:2] = open contactors)
  if ((emergencyFlags & 0x0F) != 0)
    return 0;

  // Return SME-reported max charge current (deciAmps to Amps)
  return (float)maxChargeCurrent / 10.0f;
}

void BmwSmeBms::DeInit() {
  timeoutCounter = 0;
  packCurrent = 0;
  tempMin = 0;
  tempMax = 0;
  soc = 0;
  maxChargeCurrent = 0;
  maxDischargeCurrent = 0;
  batteryVoltage = 500;
  postContactorVoltage = 0;
  isoStatusByte0 = 0;
  isoStatusByte2 = 0;
  emergencyFlags = 0;
  aliveCounter12F = 0;
  udsPollState = 0;
}
