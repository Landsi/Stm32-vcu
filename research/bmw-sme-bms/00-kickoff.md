# Phase 0: BMW OEM SME BMS — Kickoff

## Component Summary

| Field | Value |
|-------|-------|
| Component type | BMS + Contactor Control + Shunt + Isolation Monitoring |
| Component name | BMW OEM SME (Safety Management Electronics) |
| Protocol | CAN 500 kbit/s (EV-CAN bus) |
| CAN bus | Configurable (BMSCan parameter) |
| Feature branch | `feature/bmw-phev-bms` |

## What is the SME?

The SME (Safety Management Electronics) is BMW's factory battery management module
used in G30 530e, G11/G12 740e, and other PHEV platforms. It is the **master
controller** for the entire HV battery system:

- Supervises 6x CSC (Cell Supervision Circuit) modules via internal CAN
- Controls the S-Box (Safety Box) contactors and precharge via internal CAN
- Evaluates isolation monitoring results from S-Box hardware
- Reports pack-level data (SOC, temps, current, limits) on external CAN
- Can run standalone outside a BMW with only 2 keepalive CAN messages

**Key advantage over SimpBMS + direct S-Box control:** The SME is a single unit
that replaces separate BMS, contactor controller, shunt, and isolation monitor.
It uses a different CAN protocol than the S-Box external bus (0x100-0x600).

## Scope — What We're Adding to ZombieVerter

This is a **community-facing feature** for the ZombieVerter project:

1. **New BMS type** — OEM SME battery monitoring (SOC, cell temps, charge limits)
2. **Contactor control** — SME manages S-Box contactors internally (precharge,
   neg, pos) via keepalive message 0x10B
3. **Shunt** — Pack current from SME broadcast (0x112)
4. **Isolation monitoring** — Decode SME isolation status flags, report faults,
   suppress ISO during CCS charging sessions (i3 LIM / FOCCCI)

All via the SME's external CAN protocol, which is **different** from the existing
S-Box direct control (ShuntType=SBOX using 0x100/0x200/0x210/0x220/0x300).

## Clio Repo Research Findings

### RN-001: BMS Isolation Monitoring Research
**Path:** `project-clio-leaf/02_Research/Battery_REESS/RN-001_BMS_Isolation_Monitoring_Research.md`

Key findings:
- SME can run standalone with only 0x10B + 0x12F keepalives
- Isolation monitoring evaluated by SME, measured by S-Box hardware
- S-Box ISO relay controlled via 0x300 byte 0 (0xFF=OFF, 0x51=ON)
- No aftermarket project has tested S-Box ISO with relay enabled
- OEM SME reports ISO status in broadcast 0x1FA and detailed data via UDS

### RN-003: S-Box CAN Reverse Engineering
**Path:** `project-clio-leaf/03_CAN_RE/BMW_SBox/RN-003_SBox_CAN_Reverse_Engineering.md`

Key findings:
- Complete external bus message map (0x100-0x600)
- SME keepalive protocol: 0x10B (20ms, contactor control), 0x12F (100ms, terminal status)
- CRC8 SAE J1850 ZERO polynomial (init 0x3F)
- SME broadcast messages: 0x112, 0x1FA, 0x239, 0x2F5, 0x40D, 0x430, 0x431, 0x432
- UDS diagnostic protocol on 0x6F1/0x607 for detailed cell data and ISO resistance

### DBC File
**Path:** `project-clio-leaf/03_CAN_RE/BMW_SBox/DBC/Sbox.dbc`
- Covers S-Box external bus messages (not SME protocol)

### CAN Logs
**Path:** `project-clio-leaf/03_CAN_RE/BMW_SBox/Logs/`
- 13 CSV files across SBox, 530e, and 740e captures
- Include both S-Box external and SME bus traffic

## ZombieVerter Codebase Findings

### Existing BMW S-Box Implementation
- `include/bmw_sbox.h` / `src/bmw_sbox.cpp` — Static class, ShuntType=SBOX (enum 2)
- Reads 0x200 (current), 0x210 (pre-contactor V), 0x220 (post-contactor V)
- Sends 0x100 (contactor commands) and 0x300 (ISO relay OFF)
- Uses BMW CRC8 (Maxim poly 0x31) — **different from SME's SAE J1850 ZERO**
- Does NOT interact with SME at all

### BMS Framework
- Base class: `include/bms.h` — `DecodeCAN(int, uint8_t*)`, `Task100Ms()`, `SetCanInterface()`
- 4 existing BMS types: SimpBMS, LeafBMS, DaisychainBMS, KangooBMS
- BMS enum max = 5 (RenaultKangoo33), next value = 6
- Next param ID = 157, next value ID = 2124

### Contactor Control Flow
- Contactor control dispatched in Ms10Task based on ShuntType
- `SBOX::ControlContactors(opmode, can)` sends 0x100 + 0x300 every 10ms
- VCU also controls GPIO contactors (dcsw_out, prec_out, neg contactor)
- VCU manages precharge timing in MOD_PRECHARGE state

### CCS Charging Detection
- ChargeInterfaces enum: i3LIM=1, Chademo=2, CPC=3, Foccci=4
- CCS state tracked via `CCS_COND` parameter (CCS_STATUS enum)
- `CCS_Contactor` parameter indicates CCS contactor state
- `chgtyp` parameter: OFF=0, AC=1, DCFC=2

### No ISO-TP Support
- Neither ZombieVerter nor libopeninv has ISO-TP implementation
- UDS-based voltage/cell data polling would require new infrastructure

## Architecture Decision: Voltage Data Source

**Problem:** The SME does not broadcast actual pack voltage in any periodic CAN
message. Voltage is only available via UDS (ISO-TP, DIDs 0xDDB4/0xDD66), which
requires ISO-TP — not currently available in the codebase.

**Options:**

### Option A: Dual CAN bus (SME + S-Box external)
- BMSCan → SME: keepalives, BMS data, contactor commands
- ShuntCan → S-Box external bus: voltage (0x210/0x220), current (0x200)
- Need a "read-only S-Box" ShuntType (existing SBOX sends contactor commands which
  would conflict with SME internal control)
- **Pro:** Accurate voltage, uses existing hardware decoding
- **Con:** Requires 2 CAN buses, new ShuntType variant

### Option B: Single CAN bus (SME only) + implement ISO-TP for UDS
- BMSCan → SME: everything
- Poll voltage via UDS DID 0xDDB4 (pre-contactor) and 0xDD66 (post-contactor)
- **Pro:** Single CAN bus, full data access
- **Con:** Significant new code (ISO-TP multi-frame handling), adds complexity

### Option C: Single CAN bus (SME only), no direct voltage
- BMSCan → SME: everything
- Use 0x2F5 voltage limits as approximate voltage bounds
- Or derive voltage from SOC + known cell chemistry curve
- **Pro:** Simplest implementation
- **Con:** No accurate real-time voltage, precharge validation unreliable

**Recommendation:** Start with **Option A** (dual CAN). It's the most practical
and leverages existing S-Box hardware that users with BMW PHEV batteries already
have. The S-Box external bus is always active regardless of SME control. Add a
read-only S-Box ShuntType that doesn't send contactor commands.

**Update:** Implementation proceeded with a single CAN bus approach (effectively Option B simplified). The S-Box external bus was confirmed to be physically inside the battery box and inaccessible. Voltage is obtained via single-frame UDS polling (DIDs 0xDDB4 and 0xDD66), which was simpler than expected — no ISO-TP needed. See 02-implementation-plan.md for the final architecture.

## Data Gaps and Open Questions

1. **SME CRC polynomial:** RN-003 says SAE J1850 ZERO (init 0x3F). Existing S-Box
   code uses Maxim poly 0x31. Need to verify which CRC the SME actually uses for
   0x10B and 0x12F.

2. **SME wakeup:** Requires 100 kbit/s CAN frame (0x554) before switching to
   500 kbit/s. STM32 CAN peripheral speed change at runtime needs investigation.
   May not be needed if SME is powered directly (no sleep mode).

3. **0x112 unknown bytes:** Bytes 2-7 partially decoded. May contain additional
   useful data (voltage?). Needs log analysis.

4. **Isolation monitoring during CCS:** Does the SME handle ISO suppression
   internally, or does it enter a fault state? The VCU may need to actively manage
   this via the contactor command or a UDS routine (0xAD61).

5. **Contactor feedback:** How does the VCU know contactors are actually closed?
   SME broadcast 0x112 bytes 5-6 have contactor-related flags, but exact decode
   needs verification.

6. **S-Box external bus behavior when SME controls internally:** Are 0x200/0x210/
   0x220 still broadcast? Likely yes (hardware-level), but needs confirmation.

## Reference Implementations

- **Existing ZombieVerter:** `bmw_sbox.cpp` (S-Box direct control pattern)
- **Battery-Emulator:** `dalathegreat/Battery-Emulator` BMW-PHEV-BATTERY.cpp
  (complete SME standalone implementation — primary reference)
- **Upstream:** `damienmaguire/BMW_SBox` (Arduino test code)
- **SimpBMS firmware:** `Tom-evnut/BMWPhevBMS` (CSC cell monitoring)
