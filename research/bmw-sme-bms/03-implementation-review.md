# Phase 3-4: BMW OEM SME — Implementation & Review

## Implementation Summary

### New Files

| File | Purpose | Size |
|------|---------|------|
| `include/BmwSmeBms.h` | BMS class header, inherits `BMS` base | ~70 lines |
| `src/BmwSmeBms.cpp` | SME broadcast decode, UDS voltage polling, 0x12F keepalive, ISO monitoring | ~335 lines |
| `include/BmwSmeContactor.h` | Contactor control header, static class | ~55 lines |
| `src/BmwSmeContactor.cpp` | 0x10B keepalive at 20ms, contactor commands, SAE J1850 CRC | ~113 lines |

### Modified Files

| File | Change |
|------|--------|
| `Makefile` | Added `BmwSmeBms.o BmwSmeContactor.o` to OBJSL |
| `include/param_prj.h` | Added `BMSModeBmwSme = 6`, ShuntType max 4→5, SHNTYPE `5=BMW_SME`, BMSMODES `6=BMW_SME`, fixed SHNTYPE typo (period→comma at `3=VAG`) |
| `src/stm32_vcu.cpp` | Includes, static `BMSbmwSme` instance, `UpdateBMS()` case, ShuntType==5 contactor dispatch on BMSCan |
| `src/utils.cpp` | Added ShuntType==5 case in `ProcessUdc()` — resets `udc` to 0 during MOD_OFF |

### Binary Size

- Before: baseline (not measured)
- After: 79,600 bytes text, 10,028 data, 3,612 bss = 93,240 total

---

## Architecture

Single CAN bus — all communication via SME external CAN only. The S-Box
external bus is physically inside the sealed battery box and not accessible.

**Two classes, one CAN bus (BMSCan):**

1. **BmwSmeBms** (BMS_Mode = 6) — receive + UDS polling:
   - Decodes SME broadcasts: 0x112, 0x1FA, 0x2F5, 0x40D, 0x431, 0x432
   - Polls pack voltage via single-frame UDS on 0x6F1/0x607 (no ISO-TP needed)
   - Sends 0x12F terminal status keepalive every 100ms
   - Sets: udc, udc2, udcsw, idc, SOC, BMS_Tmin/Tmax, BMS_ChargeLim,
     BMS_MaxInput/MaxOutput, BMS_Isolation, power
   - Works standalone (ShuntType=None + GPIO contactors) or with ShuntType=5

2. **BmwSmeContactor** (ShuntType = 5) — 20ms contactor control:
   - Sends 0x10B every 20ms (called from 10ms task, sends every other call)
   - Maps VCU opmode → SME contactor command (0x00=open, 0x10=close)
   - SME manages full precharge sequence internally
   - Sends on BMSCan (not ShuntCan) since 0x10B goes to SME
   - Required when using sealed BMW PHEV pack (no GPIO access to contactors)

---

## CAN Message Map

### TX (VCU → SME)

| ID | DLC | Cycle | Sent By | Content |
|----|-----|-------|---------|---------|
| 0x10B | 3 | 20ms | BmwSmeContactor | Contactor command + CRC + alive counter |
| 0x12F | 8 | 100ms | BmwSmeBms | Terminal status keepalive + CRC |
| 0x6F1 | 8 | 200ms* | BmwSmeBms | UDS voltage request (alternating DDB4/DD66) |

*Each DID polled every 200ms (alternating at 100ms)

### RX (SME → VCU)

| ID | DLC | Cycle | Handler | Provides |
|----|-----|-------|---------|----------|
| 0x112 | 8 | 20ms | BmwSmeBms | Pack current (deciAmps), emergency flags |
| 0x1FA | 8 | 1s | BmwSmeBms | ISO status flags, cell temp min/max |
| 0x2F5 | 8 | 100ms | BmwSmeBms | Charge/discharge voltage + current limits |
| 0x40D | 8 | 1s | BmwSmeBms | Available charge/discharge power |
| 0x431 | 8 | 200ms | BmwSmeBms | ISO measurement status |
| 0x432 | 8 | 200ms | BmwSmeBms | Display SOC % |
| 0x607 | 8 | — | BmwSmeBms | UDS responses (voltage) |

---

## Parameter Mapping

| VCU Parameter | Source | Conversion |
|---------------|--------|------------|
| udc | UDS DID 0xDD66 (post-contactor) | value × 0.1 V |
| udc2 | UDS DID 0xDDB4 (pre-contactor) | value × 0.1 V |
| udcsw | udc2 - 20 V | Dynamic precharge threshold |
| idc | 0x112 bytes 0-1 | (LE16 - 8192) ÷ 10 A |
| power | udc2 × idc ÷ 1000 | kW |
| SOC | 0x432 byte 4 | Direct % |
| BMS_Tmin | 0x1FA byte 6 | byte - 50 °C |
| BMS_Tmax | 0x1FA byte 7 | byte - 50 °C |
| BMS_ChargeLim | 0x2F5 bytes 2-3 | (LE16 - 8192) ÷ 10 A |
| BMS_MaxInput | 0x40D bytes 0-1 | LE16 × 3 ÷ 1000 kW |
| BMS_MaxOutput | 0x40D bytes 2-3 | LE16 × 3 ÷ 1000 kW |
| BMS_Isolation | 0x1FA byte 0 + byte 2 | Flag: 0=fault, 9999=OK |
| BMS_Vmin | — | Not available (needs ISO-TP) |
| BMS_Vmax | — | Not available (needs ISO-TP) |

---

## Review Findings

### Issue Found: `udc` Not Set for Precharge Validation

**Problem:** The initial implementation set `udc3` for post-contactor voltage,
but the VCU's precharge check in `Ms10Task` compares `udc` against `udcsw`.
Without `udc` being set, precharge would never complete.

**Root cause:** Misunderstanding of the VCU voltage parameter semantics:
- `udc` = DC link / post-contactor voltage (used by mode state machine)
- `udc2` = battery / pre-contactor voltage
- `udc3` = tertiary voltage (not used for precharge)

**Fix:** Changed BmwSmeBms to set `udc` = postContactorVoltage (from UDS DID
0xDD66), matching how the existing SBOX shunt sets `udc` from `Voltage2`
(post-contactor).

### Issue Found: `ProcessUdc()` Missing ShuntType 5

**Problem:** `utils.cpp ProcessUdc()` handles ShuntTypes 0-4 but had no case
for ShuntType 5 (BMW_SME). For ShuntType 0, it resets `udc` to 0 during
MOD_OFF — critical for precharge to work (udc must start at 0 and rise).
Without this reset, `udc` would retain its last value after shutdown, and the
next precharge check could pass immediately without actual precharge.

**Fix:** Added ShuntType == 5 case that resets `udc` to 0 during MOD_OFF.
All other voltage/current updates are handled by BmwSmeBms in Task100Ms.

### Protocol Verification

| Check | Result |
|-------|--------|
| 0x10B message format (CRC byte 0, cmd+counter byte 1, 0xFC byte 2) | Correct |
| 0x10B CRC: SAE J1850 ZERO, init 0x3F, over bytes 1-2 | Correct |
| 0x10B alive counter 0-14, wraps to 0 | Correct |
| 0x10B timing: 20ms (via 10ms task with divider) | Correct |
| 0x10B contactor commands: 0x00=open, 0x10=close | Correct |
| 0x12F message format (CRC byte 0, counter in byte 1, static bytes 2-7) | Correct |
| 0x12F static bytes: 0x86, 0x1B, 0xF1, 0x35, 0x30, 0x02 | Correct |
| 0x12F alive counter: 0x20 + (0-14) | Correct |
| 0x112 current: (LE16 - 8192) deciAmps | Correct |
| 0x1FA temp: byte - 50 °C | Correct |
| 0x1FA ISO flags: byte 0 bits [1:0],[3:2], byte 2 bits [7:6] | Correct |
| 0x2F5 voltage limits: LE16, 0.1V | Correct |
| 0x2F5 current limits: (LE16 - 8192) deciAmps | Correct |
| 0x40D power: LE16 × 3 W | Correct |
| 0x432 SOC: byte 4, direct % | Correct |
| UDS extended addressing: byte 0 = target/source addr | Correct |
| UDS single-frame PCI: byte 1 = length | Correct |
| No CAN ID conflicts with other components | Verified |

### Pattern Compliance (vs Existing BMS Implementations)

| Pattern | Kangoo | Leaf | BMW SME | Match |
|---------|--------|------|---------|-------|
| Sets udc2 (battery voltage) | Yes | Yes | Yes | Yes |
| Sets udcsw dynamically | battV-30 | battV-20 | battV-20 | Yes |
| Sets idc | Yes | Yes (ShuntType==0) | Yes | Yes |
| Sets SOC | Yes | Yes (ShuntType==0) | Yes | Yes |
| Reports BMS_Isolation | Yes (Ohm) | No | Yes (flag) | Yes |
| Timeout with fallback | Yes (udcsw=500) | No | Yes (udcsw=500) | Yes |
| Sends CAN in Task100Ms | Yes (0x423) | No | Yes (0x12F, UDS) | Yes |
| MaxChargeCurrent with safety | Yes | Base default | Yes (timeout+emergency) | Yes |
| DeInit resets all state | N/A | N/A | Yes | Yes |
| Works without ShuntType | N/A | Yes (ShuntType=0) | Yes (ShuntType=0) | Yes |

---

## Known Limitations

1. **Per-cell voltage (BMS_Vmin/Vmax)** — Not available. Requires multi-frame
   UDS (ISO-TP) for DID 0xDFA0 or 0xDFA5. The SME enforces cell limits
   internally and reflects them in 0x2F5.

2. **No SME wakeup** — Assumes SME powered directly from 12V supply. The
   100 kbit/s CAN wake sequence is not implemented.

3. **ISO monitoring is flag-based** — Reports fault/OK from 0x1FA status
   flags, not actual resistance in kOhm (which requires multi-frame UDS
   DID 0xDD6A).

4. **No active ISO disable during CCS** — UDS routine 0xAD61 stop
   sub-function is documented but not implemented (needs bench testing).
   CCS ISO interaction behavior is unknown.

5. **UDS DID byte ordering unverified** — Response endianness assumed
   big-endian based on Battery-Emulator reference. Needs verification
   against CAN logs.

6. **CRC table unverified against logs** — SAE J1850 ZERO CRC with init
   0x3F is from Battery-Emulator. Should be verified against captured
   0x10B/0x12F frames from a working SME.

7. **Test suite pre-existing failure** — `make Test` fails on master due to
   unrelated vtable linker errors in test infrastructure. Not caused by
   these changes.

---

## Future Enhancements

1. **ISO-TP for multi-frame UDS** — Per-cell voltages (DID 0xDFA5),
   cell summary (DID 0xDFA0), ISO resistance in kOhm (DID 0xDD6A),
   SOH (DID 0xDD7B), balancing status
2. **Active ISO control during CCS** — UDS routine 0xAD61
3. **SME wakeup** — 100 kbit/s CAN wake for sleep mode setups
4. **Contactor feedback** — Parse 0x112 bytes 5-6 for contactor state
5. **BMS_Tavg** — Average temperature calculation
6. **Energy content** — From 0x431 for kWh remaining
7. **Additional single-frame UDS DIDs** — SOC at 0.01% (0xDDC4),
   SOH (0xDD7B) — no ISO-TP needed
