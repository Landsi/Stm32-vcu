# Phase 1: BMW OEM SME — Protocol Analysis

## SME External CAN Bus Protocol

The SME communicates on the vehicle's EV-CAN bus at **500 kbit/s**. The VCU sends
two keepalive messages; the SME responds with periodic broadcast messages.

---

## TX Messages (VCU → SME)

### 0x10B — Contactor Control Keepalive (20 ms cycle)

**DLC:** 3 bytes

| Byte | Field | Encoding |
|------|-------|----------|
| 0 | CRC8 | SAE J1850 ZERO, init 0x3F, over bytes 1-2 |
| 1 | [7:4] Contactor command | `0x0` = open (startup/off), `0x1` = close request |
| 1 | [3:0] Alive counter | 0-14, wraps to 0 |
| 2 | Static | Always `0xFC` |

**Contactor command sequence:**
1. Startup: Send command `0x0` (open) for ~160 cycles (~3.2 s)
2. Close request: Switch to `0x1` — SME handles full precharge internally
3. Open request: Switch back to `0x0`

**CRC calculation:** SAE J1850 ZERO polynomial with init value 0x3F:
```
crc = 0x3F
for each byte in [byte1, byte2]:
    crc = SAE_J1850_TABLE[crc ^ byte]
byte0 = crc
```

**Note:** This CRC is different from the S-Box external bus CRC (Maxim poly 0x31
with init 0x00).

### 0x12F — Terminal Status Keepalive (100 ms cycle)

**DLC:** 8 bytes

| Byte | Content |
|------|---------|
| 0 | CRC8 (SAE J1850 ZERO, init 0x3F, over bytes 1-7) |
| 1 | `0x20` + alive counter (cycles 0x20-0x2E, counter 0-14) |
| 2 | `0x86` (static) |
| 3 | `0x1B` (static) |
| 4 | `0xF1` (static) |
| 5 | `0x35` (static) |
| 6 | `0x30` (static) |
| 7 | `0x02` (static) |

Bytes 2-7 are static terminal status values. Only byte 1 changes (alive counter).

---

## RX Messages (SME → VCU, Broadcast)

### 0x112 — HV Battery Status 2 (20 ms cycle)

Primary source for **pack current**.

| Bytes | Field | Decoding | Unit |
|-------|-------|----------|------|
| 0-1 | Battery current | `(byte[1] << 8 \| byte[0]) - 8192` | deciAmps (÷10 for A) |
| 2-3 | Diagnostics | Partially decoded | — |
| 4 | Status | Various states | — |
| 5 | Contactor open request | bits [7:6]: `00`=no info, `01`=not active, `10`=active | — |
| 6 | Emergency flags | bits [1:0]: open instantly, [3:2]: open fast | — |
| 7 | Unknown | | — |

**Example:** `00 20 4D 0D FF 7F F5 00`
- Current: `0x2000 - 8192 = 0` → 0.0 A (idle)

### 0x1FA — HV Battery Status 1 (1000 ms cycle)

Primary source for **isolation status**, **temperatures**, and **system health**.

| Byte | Bits | Field | Values |
|------|------|-------|--------|
| 0 | [1:0] | ISO error external (chassis) | 2-bit: 00=OK, 01=not active, 10=active fault, 11=invalid |
| 0 | [3:2] | ISO error internal | Same encoding |
| 0 | [5:4] | Cooling request | 2-bit status |
| 0 | [7:6] | Valve cooling status | 2-bit status |
| 1 | [1:0] | Interlock (HVIL) error | 2-bit status |
| 1 | [3:2] | Precharge locked | 2-bit status |
| 1 | [5:4] | Disconnecting switch status | 2-bit status |
| 1 | [7:6] | Emergency mode status | 2-bit status |
| 2 | [1:0] | Service request | 2-bit status |
| 2 | [3:2] | Emergency mode error | 2-bit status |
| 2 | [5:4] | Disconnecting switch error (weld) | 2-bit status |
| 2 | [7:6] | **Isolation warning** | 2-bit: 10=fault present |
| 3 | [3:0] | Cold shutoff valve status | 4-bit status |
| 6 | — | Cell temperature min | `byte[6] - 50` (°C) |
| 7 | — | Cell temperature max | `byte[7] - 50` (°C) |

**Example:** `40 4A 05 70 3B 3B 3A 3D`
- Byte 0 = 0x40 → cooling request active, ISO errors OK
- Byte 6 = 0x3A → Tmin = 58-50 = 8°C
- Byte 7 = 0x3D → Tmax = 61-50 = 11°C

### 0x2F5 — Charge/Discharge Limits (100 ms cycle)

| Bytes | Field | Decoding | Unit |
|-------|-------|----------|------|
| 0-1 | Max charge voltage | `byte[1] << 8 \| byte[0]` (LE 16-bit) | 0.1 V |
| 2-3 | Max charge current | `(byte[3] << 8 \| byte[2]) - 8192` | deciAmps (÷10) |
| 4-5 | Min discharge voltage | `byte[5] << 8 \| byte[4]` (LE 16-bit) | 0.1 V |
| 6-7 | Max discharge current | `(byte[7] << 8 \| byte[6]) - 8192` | deciAmps (÷10) |

**Example:** `BA 0F 00 20 85 0A 00 20`
- Max charge V: 0x0FBA = 4026 → 402.6 V (96 cells × 4.19 V)
- Max charge I: 0x2000 - 8192 = 0 → 0 A (idle, no charge request)
- Min discharge V: 0x0A85 = 2693 → 269.3 V (96 cells × 2.81 V)

**Note on current scaling:** The offset of 8192 and deciAmps unit needs
verification against logged data. The Battery-Emulator reference uses this
decoding.

### 0x40D — Available Power (1000 ms cycle)

| Bytes | Field | Decoding | Unit |
|-------|-------|----------|------|
| 0-1 | Short-term charge power | `(byte[1] << 8 \| byte[0]) × 3` | W |
| 2-3 | Short-term discharge power | `(byte[3] << 8 \| byte[2]) × 3` | W |
| 4-5 | Long-term charge power | `(byte[5] << 8 \| byte[4]) × 3` | W |
| 6-7 | Long-term discharge power | `(byte[7] << 8 \| byte[6]) × 3` | W |

### 0x432 — SOC Info (200 ms cycle)

| Byte(s) | Bits | Field | Decoding |
|---------|------|-------|----------|
| 0 | [1:0] | Request operating mode | 2-bit enum |
| 0[7:4]+1 | — | Target CV voltage | `(byte[1] << 4 \| byte[0] >> 4) / 10` V |
| 2 | — | Request charge condition min | `byte[2] / 2` % |
| 3 | — | Request charge condition max | `byte[3] / 2` % |
| 4 | — | **Display SOC** | Direct **%** value |

### 0x431 — Battery Unit Data (200 ms cycle)

| Byte | Bits | Field |
|------|------|-------|
| 0 | [1:0] | Service disconnection plug status |
| 0 | [3:2] | **Isolation measurement status** (2-bit enum) |
| 0 | [5:4] | Request abort charging |
| 2-3 | — | Prediction duration charging (minutes) |
| 5 + 6[3:0] | — | Energy content max: `((byte[6] & 0x0F) << 8 \| byte[5]) / 50` kWh |

### 0x239 — Predicted Charge Condition (200 ms cycle)

| Bytes | Field | Decoding | Unit |
|-------|-------|----------|------|
| 1-2 | Predicted energy charge condition | `byte[2] << 8 \| byte[1]` | Wh |
| 3-4 | Predicted energy charging target | `(byte[4] << 8 \| byte[3]) × 0.02` | kWh |

### 0x430 — Prediction Voltages (1000 ms cycle)

| Bytes | Field |
|-------|-------|
| 0-1 | Prediction voltage short-term charge |
| 2-3 | Prediction voltage short-term discharge |
| 4-5 | Prediction voltage long-term charge |
| 6-7 | Prediction voltage long-term discharge |

---

## UDS Single-Frame Requests (VCU → SME on 0x6F1, response on 0x607)

These are simple single-CAN-frame request/response pairs. No ISO-TP multi-frame
handling needed. Uses extended addressing (byte 0 = target/source address).

### Voltage Polling (every 100ms, alternating DIDs)

**DID 0xDDB4 — Pre-contactor voltage:**
```
TX 0x6F1: 07 03 22 DD B4 00 00 00
RX 0x607: F1 05 62 DD B4 [Vhi] [Vlo] 00
```
Voltage = `(Vhi << 8 | Vlo) × 0.1` V

**DID 0xDD66 — Post-contactor voltage:**
```
TX 0x6F1: 07 03 22 DD 66 00 00 00
RX 0x607: F1 05 62 DD 66 [Vhi] [Vlo] 00
```
Same decoding.

### ISO Measurement Control

**Routine 0xAD61 — Start ISO test:**
```
TX 0x6F1: 07 04 31 01 AD 61 00 00
RX 0x607: F1 06 71 03 AD 61 [status] [fault]
```

**Routine 0xAD61 — Stop ISO test:**
```
TX 0x6F1: 07 04 31 02 AD 61 00 00
RX 0x607: F1 06 71 03 AD 61 [status] [fault]
```

**Note:** Stop sub-function (0x02) needs bench verification. Used to actively
disable ISO measurement during CCS charging sessions.

### Multi-Frame DIDs (Future — Requires ISO-TP)

These contain valuable data but need full ISO-TP implementation:

| DID | Content | Response Size |
|-----|---------|---------------|
| 0xDFA0 | Cell min/max/avg voltages + temps | Multi-frame |
| 0xDFA5 | All 96 cell voltages in mV | ~195 bytes |
| 0xDD6A | ISO resistance in kOhm + plausibility | Multi-frame |
| 0xDDC4 | SOC in 0.01% resolution | Single frame |
| 0xDD7B | SOH % | Single frame |

Single-frame DIDs (0xDDC4, 0xDD7B) could be added without ISO-TP.

---

## S-Box External Bus Messages (NOT accessible — inside battery box)

These messages exist on the S-Box's external CAN bus but are physically inside
the sealed battery enclosure. They are NOT available at the battery connector.
Listed here for reference only:

- 0x200: Battery current (mA) — 10ms
- 0x210: Pre-contactor voltage (mV) — 10ms
- 0x220: Post-contactor voltage (mV) — 10ms
- 0x310: Contactor status feedback — 20ms
- 0x510: Status / KL30C — varies

---

## Wakeup Procedure (Optional)

Required only if SME enters sleep mode (BMW power management). Not needed if SME
is powered directly from 12V ignition supply.

1. Switch CAN to **100 kbit/s**
2. Send 0x554, DLC 4, data `5A A5 5A A5` — **twice**
3. Wait **50 ms**
4. Switch CAN back to **500 kbit/s**
5. Begin sending keepalives (0x10B, 0x12F)

**Confirmation:** SME is awake when broadcasts begin (0x112, 0x432, etc.)

**Implementation note:** Runtime CAN speed switching on STM32F103 requires
reinitializing the CAN peripheral. This is disruptive and should be optional.
Most conversion setups power the SME directly, avoiding the need for wakeup.

---

## CRC Implementation

### SAE J1850 ZERO (for SME messages 0x10B, 0x12F)

Polynomial: 0x1D, init: 0x3F, no final XOR

Standard SAE J1850 CRC8 lookup table. **Different** from the Maxim/Dallas CRC8
(poly 0x31) used by the S-Box external bus (0x100 message in existing bmw_sbox.cpp).

Both CRC tables will coexist in the codebase since the S-Box external bus and SME
protocol use different polynomials.

---

## Isolation Monitoring Protocol

### Normal Operation
- SME controls S-Box ISO relay internally
- SME evaluates isolation measurement results
- Status reported in 0x1FA byte 0 bits [1:0] (external) and [3:2] (internal)
- Isolation warning in 0x1FA byte 2 bits [7:6]
- Detailed status in 0x431 byte 0 bits [3:2]

### During CCS DC Fast Charging
- External charger PE connection invalidates S-Box ISO measurement
- SME may report false ISO fault → VCU must suppress/ignore
- **Strategy:** When `interface` param = i3LIM or Foccci AND `CCS_COND` indicates
  active charging session (CCS_READY, CCS_PRECHARGE, CCS_INSULATION), the VCU
  ignores ISO fault flags from 0x1FA
- **Future:** If UDS routine 0xAD61 can actively stop ISO measurement, implement
  that as an enhancement

### VCU Reporting
- Map ISO status to existing `BMS_Isolation` parameter (Ohm)
- Values: 0 = fault/unknown, >0 = OK (use 9999 or actual kOhm if available)
- Post error via `ErrorMessage::Post()` on ISO fault (when not suppressed)

---

## Parameters to Add

### New Configuration Parameters

| Category | Name | Unit | Min | Max | Default | Notes |
|----------|------|------|-----|-----|---------|-------|
| — | — | — | — | — | — | No new config params needed initially — uses existing BMSCan, ShuntCan, BMS_Timeout, BMS_VminLimit, etc. |

### Existing Parameters Used

| Parameter | Source | Usage |
|-----------|--------|-------|
| udc | UDS DID 0xDD66 | Post-contactor voltage (DC link, 0.1V resolution) |
| udc2 | UDS DID 0xDDB4 | Pre-contactor pack voltage (0.1V resolution) |
| idc | 0x112 bytes 0-1 | Pack current (deciAmps → A) |
| SOC | 0x432 byte 4 | Display SOC % |
| BMS_Tmin | 0x1FA byte 6 | Cell temp min: `byte - 50` °C |
| BMS_Tmax | 0x1FA byte 7 | Cell temp max: `byte - 50` °C |
| BMS_ChargeLim | 0x2F5 bytes 2-3 | Max charge current from SME |
| BMS_MaxInput | 0x40D bytes 0-1 | Short-term charge power (×3 W, ÷1000 → kW) |
| BMS_MaxOutput | 0x40D bytes 2-3 | Short-term discharge power (×3 W, ÷1000 → kW) |
| BMS_Isolation | 0x1FA byte 0/2 | Mapped from ISO status flags (0=fault, 9999=OK) |
| BMS_Vmin | — | Not available without multi-frame UDS (DID 0xDFA0) |
| BMS_Vmax | — | Not available without multi-frame UDS (DID 0xDFA0) |

**Note on BMS_Vmin/Vmax:** Per-cell min/max voltages require multi-frame UDS
(ISO-TP) which is not implemented. The SME enforces cell limits internally and
reflects them in 0x2F5 charge/discharge voltage limits. BMS_Vmin/Vmax will
remain at 0 (base class default) in the initial implementation.

---

## Message Summary Table

| CAN ID | Dir | DLC | Cycle | Content | Priority |
|--------|-----|-----|-------|---------|----------|
| **0x10B** | TX | 3 | 20 ms | Contactor control + CRC + counter | Critical |
| **0x12F** | TX | 8 | 100 ms | Terminal status keepalive + CRC | Critical |
| **0x112** | RX | 8 | 20 ms | Pack current, emergency flags | High |
| **0x1FA** | RX | 8 | 1000 ms | ISO status, temps, HVIL, system health | High |
| **0x2F5** | RX | 8 | 100 ms | Charge/discharge voltage + current limits | High |
| **0x432** | RX | 8 | 200 ms | SOC, operating mode | High |
| **0x431** | RX | 8 | 200 ms | ISO measurement status, energy content | Medium |
| **0x40D** | RX | 8 | 1000 ms | Available charge/discharge power | Medium |
| 0x239 | RX | 8 | 200 ms | Predicted charge condition | Low |
| 0x430 | RX | 8 | 1000 ms | Prediction voltages | Low |
| **0x200** | RX* | 8 | 10 ms | S-Box: pack current (mA) | High* |
| **0x210** | RX* | 8 | 10 ms | S-Box: pre-contactor voltage (mV) | High* |
| **0x220** | RX* | 8 | 10 ms | S-Box: post-contactor voltage (mV) | High* |

*S-Box messages on separate CAN bus (dual-CAN architecture)

---

## Open Questions

1. **CRC verification:** The SAE J1850 ZERO CRC with init 0x3F is from the
   Battery-Emulator reference. Should be verified against CAN logs before
   implementation.

2. **Precharge validation:** With SME managing precharge internally, the VCU's
   precharge timeout check (udc vs udcsw) still works since we get voltage from
   UDS polling. But there may be a delay (up to 200ms) before the first UDS
   voltage response arrives. The SME contactor close is "fire and forget" — the
   SME decides when precharge is complete. The VCU's timeout acts as a safety
   backup.

3. **ISO stop during current flow:** Does the SME automatically pause ISO
   measurement during current flow (driving/charging)? If yes, the UDS stop
   command may be redundant. Needs bench testing.

4. **UDS response timing:** How quickly does the SME respond to UDS requests?
   If response time > 50ms, voltage updates will lag. Likely fine for the VCU's
   100ms control loop.

5. **0x10B startup delay:** Battery-Emulator sends contactorCmd = 0x00 for
   ~160 cycles (3.2s) before allowing close. Is this a hard SME requirement or
   a safety margin? Need to map this to VCU's MOD_PRECHARGE state.
