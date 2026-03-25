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

#ifndef BMWSMEBMS_H
#define BMWSMEBMS_H

#include "bms.h"
#include <stdint.h>

/*
 * BMW OEM SME (Safety Management Electronics) BMS support.
 *
 * The SME is the factory battery management module used in BMW G30 530e,
 * G11/G12 740e, and other PHEV platforms. It manages the S-Box contactors,
 * cell supervision (CSC) modules, and isolation monitoring internally.
 *
 * This class handles the SME's external CAN protocol:
 *   - Receives broadcast messages for current, temps, SOC, limits, ISO status
 *   - Polls pack voltage via single-frame UDS requests (no ISO-TP needed)
 *   - Sends 0x12F terminal status keepalive (100ms)
 *
 * Contactor control (0x10B keepalive at 20ms) is handled separately by
 * BmwSmeContactor (ShuntType) for timing reasons.
 *
 * Reference: dalathegreat/Battery-Emulator BMW-PHEV-BATTERY implementation
 * Reference: damienmaguire/BMW_SBox
 */
class BmwSmeBms : public BMS {
public:
  void SetCanInterface(CanHardware *c) override;
  void DecodeCAN(int id, uint8_t *data) override;
  void Task100Ms() override;
  float MaxChargeCurrent() override;
  void DeInit() override;

private:
  void handle112(uint8_t *data);
  void handle1FA(uint8_t *data);
  void handle2F5(uint8_t *data);
  void handle40D(uint8_t *data);
  void handle431(uint8_t *data);
  void handle432(uint8_t *data);
  void handle607(uint8_t *data);
  void sendKeepalive12F();
  void sendUdsRequest(uint16_t did);

  static uint8_t calcCrc(const uint8_t *data, uint8_t len);

  int16_t packCurrent = 0;          // deciAmps from 0x112
  int8_t tempMin = 0;               // deg C from 0x1FA
  int8_t tempMax = 0;               // deg C from 0x1FA
  uint8_t soc = 0;                  // % from 0x432
  int16_t maxChargeCurrent = 0;     // deciAmps from 0x2F5
  int16_t maxDischargeCurrent = 0;  // deciAmps from 0x2F5
  uint16_t maxChargeVoltage = 0;    // 0.1V from 0x2F5
  uint16_t minDischargeVoltage = 0; // 0.1V from 0x2F5
  uint16_t chargePowerShort = 0;    // x3 W from 0x40D
  uint16_t dischargePowerShort = 0; // x3 W from 0x40D
  float batteryVoltage = 500;       // V from UDS, init high to prevent
                                    // precharge until voltage known
  float postContactorVoltage = 0;   // V from UDS
  uint8_t isoStatusByte0 = 0;       // raw 0x1FA byte 0
  uint8_t isoStatusByte2 = 0;       // raw 0x1FA byte 2
  uint8_t isoMeasStatus = 0;        // from 0x431 byte 0 bits [3:2]
  uint8_t emergencyFlags = 0;       // from 0x112 byte 6
  int timeoutCounter = 0;
  uint8_t aliveCounter12F = 0;      // 0-14
  uint8_t udsPollState = 0;         // alternates between voltage DIDs
};

#endif // BMWSMEBMS_H
