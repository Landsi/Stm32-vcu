# Phase 2: BMW OEM SME — Implementation Plan (Revised)

**This plan requires user approval before proceeding to Phase 3 (code).**

---

## Architecture Overview

**Single CAN bus — all communication via SME external CAN only.**

The S-Box external bus is physically inside the sealed battery box and not
accessible without opening it. The only CAN interface at the battery connector
is the SME's EV-CAN bus.

```
                         CAN Bus (BMSCan)
                              │
               ┌──────────────┼──────────────┐
               │     BMW OEM SME Module      │
               │                             │
    TX:        │  0x10B (contactor cmd, 20ms) │
               │  0x12F (keepalive, 100ms)   │
               │  0x6F1 (UDS requests)       │
               │                             │
    RX:        │  0x112 (current, 20ms)      │
    (broadcast)│  0x1FA (ISO+temps, 1s)      │
               │  0x2F5 (limits, 100ms)      │
               │  0x432 (SOC, 200ms)         │
               │  0x431 (ISO meas, 200ms)    │
               │  0x40D (power, 1s)          │
               │  0x607 (UDS responses)      │
               └─────────────────────────────┘
```

### How Voltage is Obtained (UDS Single-Frame)

The SME does not broadcast actual pack voltage. Instead, the VCU polls it via
**simple UDS requests** — these are ordinary single CAN frame exchanges, NOT
multi-frame ISO-TP. No ISO-TP implementation needed.

**Voltage request:**
```
TX 0x6F1: [07] [03] [22 DD B4] [00 00 00]
           │    │    └── ReadDataByIdentifier, DID 0xDDB4
           │    └── Single frame, 3 data bytes
           └── Extended addressing: target SME (0x07)

RX 0x607: [F1] [05] [62 DD B4 XX XX] [00]
           │    │    └── Positive response, voltage bytes
           │    └── Single frame, 5 data bytes
           └── Extended addressing: from SME (0xF1)
```

Voltage = `(XX << 8 | XX) × 0.1` V. Polled every 100ms in Task100Ms.

Similarly for post-contactor voltage (DID 0xDD66), same format.

### How ISO Is Controlled During CCS

**UDS Routine 0xAD61** — also single-frame:

**Stop ISO measurement:**
```
TX 0x6F1: [07] [04] [31 02 AD 61] [00 00]
           └── RoutineControl, Stop, routine 0xAD61
```

**Start ISO measurement:**
```
TX 0x6F1: [07] [04] [31 01 AD 61] [00 00]
           └── RoutineControl, Start, routine 0xAD61
```

**Response:**
```
RX 0x607: [F1] [06] [71 03 AD 61 SS FF]
                                   └── Status byte
```

**CCS ISO strategy:**
1. When CCS session begins (i3LIM/Foccci, CCS_COND transitions to active):
   send UDS Stop ISO (0xAD61 sub-function 0x02)
2. When CCS session ends: send UDS Start ISO (0xAD61 sub-function 0x01)
3. Monitor 0x1FA isolation flags — report faults when ISO is active (non-CCS)

**Note:** Routine 0xAD61 stop behavior needs bench verification. If the SME
doesn't support stopping ISO via UDS, fall back to monitoring 0x1FA flags and
suppressing during CCS (less safe but functional). Also investigate whether
ISO measurement is automatically paused during current flow (driving/charging).

---

## New Components

### Two new classes, single CAN bus:

1. **`BmwSmeBms`** (BMS type, enum 6) — All SME data processing:
   - Broadcast decode (current, temps, SOC, limits, isolation, power)
   - UDS polling (voltage, ISO control)
   - Parameter updates (udc, idc, SOC, BMS_Tmin/Tmax, BMS_ChargeLim, etc.)
   - 0x12F keepalive (100ms, from Task100Ms)

2. **`BmwSmeContactor`** (ShuntType, enum 5) — 10ms keepalive only:
   - Send 0x10B contactor control message every 20ms (called from 10ms task)
   - Maps VCU opmode to SME contactor command
   - CRC calculation + alive counter
   - Sends on **BMSCan** (not ShuntCan)
   - No CAN message registration (TX only)
   - Does NOT set udc/idc (BMS class handles that)

**Why two classes?** The 0x10B keepalive must be sent every 20ms. The BMS base
class only has Task100Ms. The ShuntType's ControlContactors is called every
10ms from Ms10Task — perfect for the keepalive timing.

---

## New Files to Create

### 1. `include/BmwSmeBms.h`

```
Class: BmwSmeBms (inherits BMS)

Public methods (overrides):
  - void SetCanInterface(CanHardware *c)
  - void DecodeCAN(int id, uint8_t *data)
  - void Task100Ms()
  - float MaxChargeCurrent()
  - void DeInit()

Private members:
  - int16_t packCurrent         (deciAmps from 0x112)
  - int8_t tempMin, tempMax     (°C from 0x1FA)
  - uint8_t soc                 (% from 0x432)
  - int16_t maxChargeCurrent    (deciAmps from 0x2F5)
  - int16_t maxDischargeCurrent (deciAmps from 0x2F5)
  - uint16_t maxChargeVoltage   (0.1V from 0x2F5)
  - uint16_t minDischargeVoltage (0.1V from 0x2F5)
  - uint16_t chargePowerShort   (×3 W from 0x40D)
  - uint16_t dischargePowerShort (×3 W from 0x40D)
  - uint16_t preContactorVoltage (0.1V from UDS DID 0xDDB4)
  - uint16_t postContactorVoltage (0.1V from UDS DID 0xDD66)
  - uint8_t isoStatusByte0      (raw 0x1FA byte 0)
  - uint8_t isoStatusByte2      (raw 0x1FA byte 2)
  - uint8_t isoMeasStatus       (from 0x431 byte 0 bits [3:2])
  - uint8_t emergencyFlags      (from 0x112 byte 6)
  - uint8_t timeoutCounter      (for BMS timeout detection)
  - uint8_t aliveCounter12F     (0-14)
  - uint8_t udsPollState        (alternates between voltage DIDs)
  - bool isoActive              (tracks whether ISO measurement is running)

Private methods:
  - void handle112(uint8_t *data)
  - void handle1FA(uint8_t *data)
  - void handle2F5(uint8_t *data)
  - void handle40D(uint8_t *data)
  - void handle431(uint8_t *data)
  - void handle432(uint8_t *data)
  - void handle607(uint8_t *data)    (UDS response parser)
  - void sendKeepalive12F()
  - void sendUdsRequest(uint16_t did)
  - void sendIsoControl(bool enable)
  - bool isCcsCharging()

Static:
  - SAE J1850 ZERO CRC8 lookup table (256 bytes)
  - uint8_t calcCrc(const uint8_t *data, uint8_t len)
```

### 2. `src/BmwSmeBms.cpp`

**SetCanInterface():**
- Register: 0x112, 0x1FA, 0x2F5, 0x40D, 0x431, 0x432, 0x607

**DecodeCAN():**
- Switch on CAN ID, dispatch to handlers
- 0x607: parse UDS responses (voltage, ISO control acknowledgment)
- Reset timeout counter on valid broadcast message

**Task100Ms():**
- Decrement timeout, check BMS comm loss
- Send 0x12F keepalive (100ms cycle)
- UDS polling cycle:
  - Alternate: DID 0xDDB4 (pre-contactor V) → DID 0xDD66 (post-contactor V)
  - Every 200ms each DID gets polled
- Set params: udc, udc2, idc, SOC, BMS_Tmin, BMS_Tmax, BMS_ChargeLim,
  BMS_MaxInput, BMS_MaxOutput, BMS_Isolation
- CCS ISO management:
  - If CCS session just started → sendIsoControl(false)
  - If CCS session just ended → sendIsoControl(true)

**MaxChargeCurrent():**
- Return 0 if: timed out, ISO fault active (non-CCS), emergency flags set
- Otherwise: return maxChargeCurrent from 0x2F5 (converted to Amps)

**handle607() — UDS response parser:**
```
Check byte 0 == 0xF1 (extended address from SME)
Check byte 1 for single-frame PCI
Parse based on service ID:
  0x62 (ReadDataByIdentifier response):
    DID 0xDDB4 → preContactorVoltage = (byte[4] << 8 | byte[5]) (0.1V)
    DID 0xDD66 → postContactorVoltage = same format
  0x71 (RoutineControl response):
    Routine 0xAD61 → log status, update isoActive flag
```

**sendUdsRequest():**
```
uint8_t frame[8] = {0x07, 0x03, 0x22, (did >> 8), (did & 0xFF), 0, 0, 0};
can->Send(0x6F1, (uint32_t *)frame, 8);
```

**sendIsoControl(enable):**
```
uint8_t subFunc = enable ? 0x01 : 0x02;  // Start or Stop
uint8_t frame[8] = {0x07, 0x04, 0x31, subFunc, 0xAD, 0x61, 0, 0};
can->Send(0x6F1, (uint32_t *)frame, 8);
isoActive = enable;
```

**isCcsCharging():**
```cpp
int chgInterface = Param::GetInt(Param::interface);
int ccsState = Param::GetInt(Param::CCS_COND);
return (chgInterface == ChargeInterfaces::i3LIM ||
        chgInterface == ChargeInterfaces::Foccci) &&
       (ccsState >= CCS_READY && ccsState <= CCS_INSULATION);
```

### 3. `include/BmwSmeContactor.h`

```
Class: BmwSmeContactor (static class)

Public static methods:
  - void ControlContactors(int opmode, CanHardware *can)

Private:
  - static uint8_t aliveCounter (0-14)
  - static uint8_t sendDivider  (for 20ms from 10ms calls)
  - static uint8_t calcCrc(const uint8_t *data, uint8_t len)
  - static SAE J1850 CRC table (shared with BmwSmeBms, or duplicated)
```

### 4. `src/BmwSmeContactor.cpp`

**ControlContactors(opmode, can):**
```
Called every 10ms. Use sendDivider to send every other call (20ms).

uint8_t contactorCmd;
switch (opmode) {
  case MOD_OFF:
  case MOD_PCHFAIL:
    contactorCmd = 0x00;  // Keep contactors open
    break;
  case MOD_PRECHARGE:
  case MOD_RUN:
  case MOD_CHARGE:
  case MOD_PREHEAT:
    contactorCmd = 0x10;  // Request contactor close
    break;
  default:
    contactorCmd = 0x00;
}

uint8_t frame[8] = {0};
frame[1] = (contactorCmd | (aliveCounter & 0x0F));
frame[2] = 0xFC;
frame[0] = calcCrc(&frame[1], 2);  // CRC over bytes 1-2, init 0x3F

can->Send(0x10B, (uint32_t *)frame, 3);

aliveCounter++;
if (aliveCounter > 14) aliveCounter = 0;
```

**Note:** The SME handles the full precharge sequence internally. When the VCU
sends contactorCmd = 0x10 (close request), the SME:
1. Closes precharge relay
2. Closes negative contactor
3. Closes positive contactor
4. Opens precharge relay
The VCU just needs to request "close" and wait for voltage to appear.

---

## Files to Modify

### 5. `Makefile`

Add to OBJSL:
```
BmwSmeBms.o BmwSmeContactor.o
```

### 6. `include/param_prj.h`

**Changes:**

```cpp
// Line 39: Update BMS_Mode max from 5 to 6
PARAM_ENTRY(CAT_SETUP, BMS_Mode, BMSMODES, 0, 6, 0, 90)

// Line 40: Update ShuntType max from 4 to 5
PARAM_ENTRY(CAT_SETUP, ShuntType, SHNTYPE, 0, 5, 0, 88)

// Line 305: Fix typo and add BMW_SME
// OLD: "0=None, 1=ISA, 2=SBOX, 3=VAG. 4=ISA_udcsw"
// NEW:
#define SHNTYPE "0=None, 1=ISA, 2=SBOX, 3=VAG, 4=ISA_udcsw, 5=BMW_SME"

// Line 319-321: Add BMW_SME
// OLD: "0=Off, 1=SimpBMS, 2=TiDaisychainSingle, 3=TiDaisychainDual, 4=LeafBms, "
//      "5=RenaultKangoo33"
// NEW:
#define BMSMODES                                                               \
  "0=Off, 1=SimpBMS, 2=TiDaisychainSingle, 3=TiDaisychainDual, 4=LeafBms, "   \
  "5=RenaultKangoo33, 6=BMW_SME"

// Line 450-457: Add enum value
enum BMSModes {
  BMSModeNoBMS = 0,
  BMSModeSimpBMS = 1,
  BMSModeDaisychainSingleBMS = 2,
  BMSModeDaisychainDualBMS = 3,
  BMSModeLeafBMS = 4,
  BMSRenaultKangoo33BMS = 5,
  BMSModeBmwSme = 6
};
```

**No new parameters needed.** Uses existing BMSCan, BMS_Timeout, limits, etc.
ShuntCan parameter is unused for BMW_SME (all comms on BMSCan).

### 7. `src/stm32_vcu.cpp`

**Includes:**
```cpp
#include "BmwSmeBms.h"
#include "BmwSmeContactor.h"
```

**Static instance (near other BMS instances ~line 200):**
```cpp
static BmwSmeBms BMSbmwSme;
```

**UpdateBMS() (~line 1068) — add case:**
```cpp
case BMSModes::BMSModeBmwSme:
  selectedBMS = &BMSbmwSme;
  break;
```

**Contactor control dispatch (~line 884) — add:**
```cpp
if (Param::GetInt(Param::ShuntType) == 5)
  BmwSmeContactor::ControlContactors(
      opmode,
      canInterface[Param::GetInt(Param::BMSCan)]);
```

Note: Uses **BMSCan** (not ShuntCan) since 0x10B goes to SME.

**Shunt value reading:**
Need to check all places where SBOX/ISA voltage/current values are read and
used. For ShuntType == 5, voltage and current come from the BMS class (set via
Param::SetFloat on udc/idc), so no additional shunt value reading paths are
needed. The BMS class sets these directly.

Specifically, in `utils::ProcessUdc()` and `utils::CalcSOC()`, the udc/idc
params are already used generically — they don't depend on ShuntType.

---

## Implementation Checklist

- [ ] `include/BmwSmeBms.h` — BMS class header
- [ ] `src/BmwSmeBms.cpp` — BMS implementation (broadcasts + UDS + ISO control)
- [ ] `include/BmwSmeContactor.h` — Contactor control header
- [ ] `src/BmwSmeContactor.cpp` — 0x10B keepalive + contactor control
- [ ] `Makefile` — Add BmwSmeBms.o and BmwSmeContactor.o to OBJSL
- [ ] `include/param_prj.h`:
  - [ ] Add BMSModeBmwSme = 6 to enum
  - [ ] Update BMSMODES string (add "6=BMW_SME")
  - [ ] Update BMS_Mode max from 5 to 6
  - [ ] Update ShuntType max from 4 to 5
  - [ ] Update SHNTYPE string (add "5=BMW_SME")
  - [ ] Fix SHNTYPE typo (period → comma at "3=VAG")
- [ ] `src/stm32_vcu.cpp`:
  - [ ] Add #includes
  - [ ] Add static BmwSmeBms instance
  - [ ] Add case in UpdateBMS()
  - [ ] Add ShuntType == 5 contactor control dispatch

---

## Verification Plan

1. **Build:** `make` — no errors or warnings
2. **Tests:** `make Test && ./test/test_vcu` — all pass
3. **Format:** `pre-commit run --all-files` — clean
4. **CAN ID conflicts:** Verify 0x10B, 0x112, 0x12F, 0x1FA, 0x2F5, 0x40D,
   0x431, 0x432, 0x607, 0x6F1 don't overlap with other active components
5. **Enum uniqueness:** BMSModeBmwSme=6, ShuntType=5 are unused

---

## Known Limitations (Initial Implementation)

1. **Per-cell voltage (BMS_Vmin/Vmax) not available** — Requires multi-frame
   UDS (DID 0xDFA0 or 0xDFA5), which needs ISO-TP. The SME enforces cell limits
   internally and reflects them in 0x2F5 charge/discharge voltage limits.
2. **No SME wakeup** — Assumes SME powered directly from 12V supply
3. **ISO control via UDS routine 0xAD61 needs bench testing** — If stop
   sub-function doesn't work, fall back to flag suppression
4. **UDS DID byte ordering needs verification** — Response endianness TBD
   from log analysis

---

## Future Enhancements

1. **ISO-TP implementation** — Unlock per-cell voltages (DID 0xDFA5, 96 cells),
   cell voltage summary (DID 0xDFA0), ISO resistance in kOhm (DID 0xDD6A),
   SOH (DID 0xDD7B), balancing status
2. **SME wakeup sequence** — 100 kbit/s CAN wake for sleep mode
3. **Contactor feedback** — Parse 0x112 bytes 5-6 for contactor state reporting
4. **Temperature averaging** — BMS_Tavg calculation
5. **Energy content** — From 0x431 for kWh remaining display
