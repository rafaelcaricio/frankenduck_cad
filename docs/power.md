# Power study for the PARKSIDE PAPK 12 A4

This note estimates how much current the robot draws and shows how the 12 V
PAPK 12 A4 stick battery can feed the existing electrical layout. It is a
study: nothing here is wired, ordered or built. All current figures are
**estimates**; the robot has never had its current measured.

The PARKSIDE stick battery is no longer the default. The assembly now carries a
Zeee 2S 2200 mAh LiPo (see [lipo-battery.md](lipo-battery.md)), and the KCD1
switch opening was removed from the right body shell. The current estimates
below still describe the robot's load; the wiring, switch and regulator
proposals were made for the 12 V stick and have not been redone for the 2S pack.

## Present electrical layout

The last documented layout runs everything from a 2S Li-ion pack (SmallRig
NP-F550, 6.6 V empty to 8.2 V full under load):

```
NP-F550 2S -> KCD1 rocker switch -> Waveshare Bus Servo Adapter (A) -> 15 STS3215 servos
                                 -> HENGE 4 A UBEC (5.0 V) -> Radxa Cubie A7Z header pins 4 (5 V) and 6 (GND)
```

- Servos: Feetech STS3215 7.4 V (ST-3215-C001). Register 14 (maximum voltage)
  is set to 9.5 V. The unloading condition leaves the voltage bit clear, so a
  servo does **not** switch off on overvoltage.
- The Waveshare adapter's power path is rated 5 A.
- The HENGE UBEC takes 7–25.5 V in and gives 4 A (6 A maximum) at 5.0, 6.0 or
  7.4 V.
- `robotd` reads the battery through the servos' voltage register.
  `duck_control::model` maps 6.6 V (`BATTERY_EMPTY_V`) to 0 % and 8.2 V to
  100 %, and the robot sits down and powers off at 0 %.

Sources: `microduck_rl-cad_blender-20261008/ELECTRICAL_ARCHITECTURE.md`,
`frankenduck/frankenduck-sts/docs/COMMISSIONING.md`,
`frankenduck/duck-control/src/model.rs` and
`microduck_rl/franken_duck/records/walking-failure-205342/REPORT.md`.

## How the current was estimated

No recording has the servo current register (69). The robot logs at 50 Hz
include, for each servo, the PWM duty (register 60, "load"), the supply voltage,
the goal and the measured position. The current comes from the identified
motor model that the simulator uses (BAM fit `feetech_sts3215_7_4V/m1`:
kt = 1.21 N·m/A, R = 2.68 Ω at the servo output):

- motor current = (duty × supply voltage − kt × joint speed) / R, capped at the stall current;
- supply current = duty × motor current (PWM bridge);
- the drive direction is the sign of (goal − measured position);
- each servo's electronics are counted as 10 mA (assumed).

A second figure ignores back-EMF (joint speed set to zero). It is an upper
limit.

Recordings used (`microduck-training-sts3215/`):

| file | content |
|---|---|
| `sim_vs_hw_phase_20260917/control_184246.jsonl` | 10 leg servos; 83 s walking at 0.3 m/s, 29 s standing, home, off |
| `stand_review_20260927/control_20260926T233233Z.jsonl`, `…085314Z`, `…105914Z` | 14 servos; home, then failed stands that end in falls |

## Measured-model results (servo rail)

| state | rail current, model (mean / p95 / max) | upper limit, no back-EMF (mean / p95 / max) | rail power, model (mean / p95 / max) |
|---|---|---|---|
| off, torque off | 0.1–0.15 A | — | about 1 W |
| home pose held | 0.1–0.4 A | 0.1–0.4 A | 0.8–3.2 W |
| standing (Sept 17) | 0.28 / 1.4 / 4.9 A | 0.25 / 1.1 / 6.6 A | 1.9 / 9.4 / 34 W |
| **walking (Sept 17)** | **1.8 / 3.5 / 8.2 A** | 4.1 / 8.5 / 13.2 A | **12 / 23 / 54 W** |
| failed stand, servos fighting before a fall | 5–8 / 8–15 / 11–19 A | 9–12 / 15–22 / 17–23 A | 30–50 / 51–91 / 70–134 W |

The rail sat at 6.8 V on average and dropped to 6.0 V while walking. The single
servo maximum was about 2.5 A, close to the 2.76 A stall current at 7.4 V. Short
peaks between the 20 ms samples, such as foot strikes, are not seen.

## Estimate for the stick-battery robot

Adjustments to the walking numbers:

- **Mass.** The stick variant weighs 1.795 kg, against about 1.63 kg for the
  recorded robot. Leg current is scaled by 1.2 for this.
- **Head.** The head servos (not in the walking log) add about 1.5 W (from the
  14-servo home logs).
- **Electronics.** The board, IMUs, camera and speaker are assumed to take 5 W
  on 5 V, plus a regulator loss. This is **not measured**.
- **Regulators.** Efficiency is 90 % for the servo regulator. The pack sits
  near 10.8 V under load.

| state | power from the pack | pack current at 10.8 V | runtime on 20 Wh usable (85 % of 24 Wh) |
|---|---|---|---|
| on, torque off | about 6 W | 0.6 A | about 3 h |
| standing quietly | about 10 W | 0.9 A | about 2 h |
| **walking, model mean** | **about 24 W** | **2.2 A** | **about 50 min** |
| walking, upper-limit mean | about 44 W | 4.1 A | about 27 min |
| walking, model p95 | about 39 W | 3.6 A | — |
| walking, highest sample | 82–125 W | 7.6–11.6 A | — |
| fall or servo fight, highest sample | about 150 W | about 14 A, brief | — |

Planning figures:
- **Typical walking draw:** 2–4 A from the pack (about 3–6 A on the 7.4 V servo rail).
- **Peaks to survive:** about 12–15 A from the pack for a fraction of a second.

A 3S drill pack carries these currents without strain, but its cells and their
rating are unconfirmed.

## Connecting the PAPK 12 A4

The pack is 3S Li-ion: 12.6 V full, about 10.8 V nominal, and about 9 V empty.
**Do not connect it straight to the servos.** The C001 servos are 7.4 V parts,
and 12.6 V is above the 9.5 V limit in register 14. Because the voltage bit is
cleared, they would keep running at 12.6 V.

Proposed layout:

```
PAPK 12 A4 + / − contacts
  -> short 18 AWG leads
  -> fuse 15 A, next to the pack
  -> XT60 (pod bay)
  -> KCD1 switch
  -> current/voltage monitor (INA226 or similar, on the existing I2C bus)
  -> splits into:
       a) 7.4 V buck regulator, 10 A continuous / 20 A burst -> servo chains
       b) HENGE 4 A UBEC at 5.0 V -> Radxa pins 4 and 6 (as now)
All grounds common; the T contact stays unconnected.
```

### Servo rail (a)

The servo rail needs a new step-down regulator. It should have:
- 7.4 V output (7.5 V is acceptable);
- 9–12.6 V input;
- 10 A continuous and 20 A burst;
- current limiting.

The servos then see the voltage the policy and the motor model assume
(`nominal_voltage = 7.4`) for the whole discharge. With the 2S pack, the rail
fell to 6.0–6.4 V while walking. The HENGE 4 A is too small for this rail.

### Servo power path

Walking peaks (8–12 A) exceed the Waveshare adapter's 5 A power path. Waveshare
states the board has no voltage regulation and its output voltage equals its
input, so the servo headers carry the input voltage, and the board limits the
current to 5 A.

Power should reach each servo chain (left leg, right leg, head) directly from
the regulator, through a short injection lead between the adapter and the first
servo of that chain:
- data and ground continue from the adapter;
- the adapter's V+ is cut;
- V+ and ground come from the regulator.

The adapter's own power input then stays empty, and the adapter carries only
data and a common ground. Check on the bench, with the servos powered this way,
that the adapter talks to the servos with nothing on its power input.

### Board rail (b)

The existing HENGE UBEC accepts 12 V input and stays at 5.0 V for the Radxa.
Because the board has its own regulator straight from the pack, servo surges
during a fall cannot pull the board's supply down.

### Switch

The KCD1 switch now carries 2–4 A at 12 V instead of 3–6 A at 7.4 V. Its DC
rating must still be checked against the 15 A fuse. If it is too low, use the
switch for the regulators' enable lines, or choose a larger switch.

### Battery reading (firmware change)

Battery reading changes and needs a separate firmware task. With a regulated
rail, the servo voltage register reads a constant 7.4 V. `battery_percent` and
the empty shutdown would stop working. The pack voltage must come from the
monitor instead. Suggested new constants under load:
- empty about 9.9 V (3.3 V per cell, the same per-cell floor as today's 6.6 V);
- full about 12.3 V.

The monitor would also give the first real current measurement, which the
roadmap lists as missing.

### T contact

Its function is unconfirmed for the X12V pack. No source found any digital
telemetry: no charge level and no cell voltages. On the 20 V packs, community
teardowns report how the third pin behaves:
- In normal use it is a 10 kΩ thermistor to ground, switched by a transistor.
- The pack lets the pin float when it detects a fault, such as overdischarge at
  about 2.6 V per cell.
- The pack also opens the pin briefly a few times a second.

If the X12V pack works the same way, the pin can give a temperature reading and
an "empty" signal. Read it through a 10 kΩ pull-up to 5 V and a filter of about
1 s. The pack voltage must still come from the monitor. Measure the pin on the
real pack before using it.

### Pack protection

The pack balances its cells while charging. Whether it cuts off when it is
nearly empty is unconfirmed, so the robot's own low-voltage shutdown is
required.

## Open items

- Real current: log register 69 (6.5 mA per count, unconfirmed) or read the
  monitor during walking. This replaces the model estimate.
- The board and peripherals' power draw is not measured.
- The pack's cell model, current rating, protection behaviour and T-contact
  function are unknown.
- The KCD1 switch's DC rating is not checked.
- Space for the 7.4 V regulator, the fuse and the monitor is not in the CAD
  model. The XT60 bay and contact slots are only allowances.

## Sources

- PAPK 12 A4 manual: https://www.manualslib.com/manual/3488864/Parkside-Papk-12-A4.html
- Pack listing (charging current, cell balancing): https://www.park-price.com/en/product/papk-12-a4-plgk-12-a3-battery-2-ah-and-charger-508296/1288/
- X12V pack discussion (three cells, third contact): https://www.elektroda.com/rtvforum/topic3585920.html
- STS3215 data: https://core-electronics.com.au/attachments/uploads/sts3215-smart-servo-datasheet-translated.pdf
