# AEB Decision Logic – Software Requirements

| Item            | Value                                                        |
|-----------------|--------------------------------------------------------------|
| Component       | `aeb::AebController` (Automatic Emergency Braking decision)  |
| Cycle time      | 10 ms                                                        |
| Inputs          | Ego speed, accelerator pedal, AEB switch, radar object (distance, closing speed, object acceleration, frame timestamp) |
| Outputs         | State, forward collision warning, fault lamp, brake deceleration request |
| Safety concept  | Fail-silent: on any sensor fault the function stops braking and reports a fault |

Each requirement has a unique ID. The **Verification** column names the unit
test (`tests/`) or simulation scenario (`sim/src/scenarios.cpp`) that proves it.

## 1. Definitions

- **TTC (time to collision)** = distance / closing speed, where closing speed =
  ego speed − object speed. If closing speed < 0.1 m/s the TTC is infinite.
  When the object is braking, the enhanced TTC of REQ-AEB-025 is used instead.
- **Threat level** is derived from TTC: *None* > *Warning* > *Partial* > *Full*.
- **Braking state** = `PartialBrake` or `FullBrake`.

## 2. Functional requirements

| ID          | Requirement | Verification |
|-------------|-------------|--------------|
| REQ-AEB-001 | When the AEB switch is off, the function shall be in state `Off` and shall not warn or request braking. | `REQ_AEB_001_*` |
| REQ-AEB-002 | When no object is present, the function shall stay in `Standby`. | `REQ_AEB_002_*` |
| REQ-AEB-010 | A new intervention shall only start when ego speed is in [5, 130] km/h. | `REQ_AEB_010_*` |
| REQ-AEB-020 | The function shall issue a forward collision warning when TTC < 2.6 s. | `REQ_AEB_020_*` |
| REQ-AEB-021 | The function shall request partial braking (4 m/s²) when TTC < 1.6 s. | `REQ_AEB_021_*` |
| REQ-AEB-022 | The function shall request full braking (9 m/s²) when TTC < 1.0 s. | `REQ_AEB_022_*` |
| REQ-AEB-023 | The function shall be able to escalate directly from `Standby` to any higher level (e.g. a sudden cut-in). | `REQ_AEB_023_*` |
| REQ-AEB-024 | An escalation shall only happen after the higher threat is confirmed in 2 consecutive cycles (protection against single-cycle false positives). | `REQ_AEB_024_*` |
| REQ-AEB-025 | When the radar reports the object braking harder than 1 m/s², TTC shall assume the object keeps braking until it stops (enhanced TTC). *(Added in v1.1: constant-speed TTC reacted too late in scenario CCRb-12m-6mps2.)* | `REQ_AEB_025_*`, `EnhancedTtc.*`, scenarios `CCRb-*` |
| REQ-AEB-030 | Leaving a level shall use a TTC hysteresis of 0.3 s to avoid toggling. | `REQ_AEB_030_*` |
| REQ-AEB-031 | During braking, the function shall not step down from `FullBrake` to `PartialBrake`, and shall keep braking until the closing speed is < 0.1 m/s (speeds matched) while ego speed is still above 5 km/h, or otherwise until the vehicle stops (→ REQ-AEB-040). *(Changed in v1.1: the original "release when TTC > warning threshold" made the car coast into stopped targets — found by scenarios CCRs-10…30. Changed in v1.2: near standstill, sensor noise could release the brake just before `Hold` — found in the CCRb-12m-6mps2 plot.)* | `REQ_AEB_031_*` |
| REQ-AEB-032 | An ongoing braking intervention shall continue even if ego speed drops below the activation window. | `REQ_AEB_032_*` |
| REQ-AEB-033 | The brake request shall rise by at most 50 m/s³ and fall by at most 25 m/s³ (comfort and stability). | `REQ_AEB_033_*` |
| REQ-AEB-040 | When the vehicle comes to a standstill (≤ 0.1 m/s) during braking, the function shall enter `Hold` and keep a 4 m/s² brake request. | `REQ_AEB_040_*` |
| REQ-AEB-041 | `Hold` shall end after 2 s and return to `Standby`. | `REQ_AEB_041_*` |
| REQ-AEB-050 | Accelerator pedal ≥ 90 % (kick-down) shall override the function: no warning and no braking. | `REQ_AEB_050_*`, scenario `Kickdown-40` |

## 3. Safety requirements

| ID          | Requirement | Verification |
|-------------|-------------|--------------|
| REQ-AEB-060 | If no radar frame is received for more than 200 ms, the function shall enter `Fault`, set the fault lamp and not request braking. | `REQ_AEB_060_*`, scenario `RadarLoss-40` |
| REQ-AEB-061 | Implausible radar data (NaN/Inf, distance < 0 m or > 250 m, \|closing speed\| > 70 m/s, \|object acceleration\| > 15 m/s²) shall be treated as a fault. | `REQ_AEB_061_*` |
| REQ-AEB-062 | On a fault during braking, the brake request shall be released with the limited ramp of REQ-AEB-033 (no abrupt release). | `REQ_AEB_062_*` |
| REQ-AEB-063 | The function shall leave `Fault` once valid radar data is received again. | `REQ_AEB_063_*` |

## 4. System-level performance targets (closed-loop simulation)

Simulated vehicle: 150 ms brake actuator lag, max 9.3 m/s² (dry asphalt), radar
noise σ = 5 cm / 5 cm/s. Scenario names follow the Euro NCAP AEB Car-to-Car protocol.

| ID          | Target | Verification |
|-------------|--------|--------------|
| REQ-AEB-070 | CCRs (stationary target): avoid the collision for ego speeds 10–50 km/h. | scenarios `CCRs-10` … `CCRs-50` |
| REQ-AEB-071 | CCRs: reduce impact speed by ≥ 50 % for 60–80 km/h. | scenarios `CCRs-60` … `CCRs-80` |
| REQ-AEB-072 | CCRm (target at 20 km/h): avoid the collision for ego speeds up to 70 km/h. | scenarios `CCRm-*` |
| REQ-AEB-073 | CCRb (target braking, both at 50 km/h): avoid the collision for 12 m / 40 m gaps and 2 / 6 m/s² target deceleration. | scenarios `CCRb-*` |
| REQ-AEB-074 | No warning or braking when following at a constant gap or when the target is pulling away (false-positive check). | scenarios `Follow-50`, `PullAway-50` |

## 5. Non-functional requirements

| ID          | Requirement | Verification |
|-------------|-------------|--------------|
| REQ-AEB-090 | The controller shall use no dynamic memory, exceptions, RTTI or C++ standard library (bare-metal compatible). | Firmware build with `-ffreestanding -nostdlib` |
| REQ-AEB-091 | The controller shall produce identical results on the host (SIL) and on the target CPU (PIL). | `tools/compare_sil_pil.sh` |
| REQ-AEB-092 | The firmware shall fit into 256 KB flash and 64 KB RAM. | Linker memory regions in `target/linker.ld` |
