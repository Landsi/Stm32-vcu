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

#ifndef BMWSMECONTACTOR_H
#define BMWSMECONTACTOR_H

/*
 * BMW SME contactor control via CAN message 0x10B.
 *
 * Sends the 0x10B keepalive/contactor control message at 20ms intervals.
 * This is the ShuntType companion to BmwSmeBms — it handles the time-critical
 * contactor command that must be sent every 20ms (faster than BMS Task100Ms).
 *
 * The SME manages precharge internally. The VCU simply requests "open" or
 * "close" and the SME handles the full precharge sequence (precharge relay,
 * negative contactor, positive contactor, precharge off).
 *
 * Called from Ms10Task via ControlContactors dispatch.
 * Sends on BMSCan (not ShuntCan) since 0x10B goes to the SME.
 */

#include "canhardware.h"
#include <stdint.h>

class BmwSmeContactor {
  BmwSmeContactor();
  ~BmwSmeContactor();

public:
  static void ControlContactors(int opmode, CanHardware *can);

private:
  static uint8_t aliveCounter;
  static uint8_t sendDivider;
  static uint16_t startupCycles; // counts up to STARTUP_MIN_CYCLES before
                                 // allowing contactor close
  static uint8_t calcCrc(const uint8_t *data, uint8_t len);
};

#endif // BMWSMECONTACTOR_H
