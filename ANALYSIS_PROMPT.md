# Prompt for analysing the PID vs MPC test logs

Copy everything below the line into the other AI, and attach the log files
listed in section 2.

---

You are analysing bench-test logs from an embedded temperature-control
experiment comparing a PID controller against an MPC controller on
identical hardware. Produce a results table. Follow the metric definitions
exactly — do not substitute your own. **Where data is missing or invalid,
write "no data" or "invalid" and say why. Do not estimate, interpolate, or
invent any number.**

## 1. The apparatus

- Arduino Uno drives an insulated enclosure (24×19×24 cm, ~11 L).
- Heating: PTC heater on a relay (pin 9). Cooling: fan on a relay (pin 10)
  that pulls in **ambient room air** — so ambient is a hard cooling floor.
- Both actuators are driven by **time-proportioning** (software PWM for
  mechanical relays): a 2000 ms window, with ON-time = |u|/255 of the
  window. Commands with |u| ≤ 64 (the relay deadband) produce no actuation
  at all.
- Control sensor: DHT22 inside the enclosure (0.1 °C resolution, and it
  caches readings for 2 s, so a 1 Hz log genuinely updates every other row).
- Reference sensor: a second DHT22 outside, near the fan intake. Logged as
  `Tamb_C`. Never used by either controller; it defines the setpoints.
- Measured open-loop ceiling: at 100 % heater duty the box plateaus at
  **ambient + 3.14 °C**. Targets above that are physically unreachable.
- Identified plant (fitted from the logged open-loop step response):
  first-order, `K = 0.01232 °C per duty unit above ambient`, `tau = 454 s`.

## 2. The log files

Each file is plain text. Three kinds of line:

- Lines starting with `#` — commentary, phase banners, warnings, summary.
- Numeric CSV sample rows, one per second (see section 3).
- `RESULT,` and `DUTY,` rows — metrics computed on-board at the end of a
  run (see section 4). Present only if the run reached its summary.

| File | Controller | Tests it contains | Computed RESULT rows? |
|---|---|---|---|
| `device-monitor-260812-064112-test1-3.log` | PID | Tests 1, 2, 3 (and a T4 stabilisation that failed) | **No** — you must compute these yourself |
| `device-monitor-260812-094332-test4 only.log` | PID | Test 4 only | Yes (1 row) |
| `mpc-test-260812-101350-test1-2.csv` | MPC | Tests 1 and 2 | **No** — you must compute these yourself |
| `mpc-test-260812-122724 test3-4.csv` | MPC | Tests 3 and 4 | Yes (2 rows) |

Ignore these — aborted fragments or duplicates, containing no completed
test: `mpc-test-260812-114000.csv`, `device-monitor-260812-063347.log`,
`device-monitor-260812-122510.log`, `device-monitor-260812-122653.log`,
and `mpc-test-260812-122724.csv` (a byte-identical copy of the
`... test3-4.csv` file — use only one of the two).

**Two of the four files stop before their run printed a summary**, so they
contain raw samples but no computed metrics:

- `device-monitor-260812-064112-test1-3.log` — the control sensor failed
  partway through, and the session never reached its summary.
- `mpc-test-260812-101350-test1-2.csv` — the MATLAB host stopped
  responding during the phase after test 2, so logging ended there. Its
  own progress banners record the outcomes: `# [2522s] test 1 TIMED OUT`
  and `# [3192s] test 2 settled`. Use those to sanity-check what you
  compute; if your settling analysis disagrees with a banner, say so
  rather than silently preferring one.

For both of those files you must derive every metric from the raw sample
rows using section 6.

## 3. CSV sample columns

PID logs (14 columns):

```
t_s,phase,test,Tin_C,RH_pct,Tamb_C,SP_C,err_C,u,duty_pct,heat,fan,in_band,event
```

MPC logs (15 columns — note the extra `link_ok` before `event`):

```
t_s,phase,test,Tin_C,RH_pct,Tamb_C,SP_C,err_C,u,duty_pct,heat,fan,in_band,link_ok,event
```

- `t_s` seconds since the run started; `phase` see section 5.
- `Tin_C` controlled temperature; `Tamb_C` ambient reference; `SP_C` setpoint.
- `err_C` = `SP_C − Tin_C`. `u` = controller output, −255…+255, + is heat.
- `heat`, `fan` = relay state (1 = energised). `in_band` = 1 when |err| ≤ 0.5.
- `link_ok` (MPC only) = 1 when a fresh command from the host was available.
  **Any stretch with `link_ok = 0` is not under control — exclude it and
  report how many samples were affected.**
- Sensor dropouts appear as the literal string `NaN` in `Tin_C`, `RH_pct`,
  and `err_C`. Skip those rows in every statistic and count them.

**Formatting warnings — parse numerically, never by string comparison:**

- `mpc-test-260812-101350-test1-2.csv` has been opened and re-saved by
  Excel. Trailing zeros are stripped (`25` where other files write `25.00`,
  `0` where others write `0.0`) and every `#` comment line has trailing
  commas appended to pad it to the column count. The values themselves are
  intact. The other files are unmodified.
- Decimal places therefore vary between and within files. `24.7` and
  `24.70` are the same reading.
- Split on commas and ignore trailing empty fields; do not assume a fixed
  field count on `#` lines.

## 4. RESULT / DUTY row schema

```
RESULT,test,target_C,start_C,settled,settle_s,first_in_band_s,overshoot_C,undershoot_C,rms_C,max_dev_C,extreme_C,dur_s,heat_pct,fan_pct
DUTY,scope,heat_on_s,fan_on_s,total_s,heat_pct,fan_pct,combined_pct
```

`settled` is 1/0. `NA` means the quantity is undefined, not zero.

## 5. Phases and the tests they belong to

| `phase` | Meaning |
|---|---|
| `BASELINE` | 120 s, system off, measures ambient. Not a test. |
| `T1_HEAT` | **Test 1** — heating step to ambient+5.0 °C |
| `T2_COOL` | **Test 2** — cooling step to ambient+0.0 °C |
| `T3_REHEAT` | Re-stabilise at ambient+5.0 before test 3. Not a test. |
| `T3_HOLD` | 120 s dwell before the step. Not a test. |
| `T3_STEP` | **Test 3** — setpoint step down to ambient−2.0 °C |
| `T4_STAB` | Stabilise at ambient+2.0 before test 4. Not a test. |
| `T4_PREP` | 30 s countdown. Not a test. |
| `T4_OPEN` | 5 s lid-open disturbance window |
| `T4_RECOV` | **Test 4** — recovery after the lid is resealed |
| `SUMMARY`, `DONE` | End of run. Not a test. |

A test's step begins at the `# --- step:` line preceding its phase banner.

## 6. Metric definitions — use these exactly

All are computed over one step, from step start to step end.

- **Settling time (s)** — time from step start to the *beginning* of the
  first continuous window of **≥ 300 s** during which |err| ≤ **0.5 °C**.
  If no such window exists, the test did not settle: report "did not settle".
  Leaving the band restarts the window from scratch.
- **First in band (s)** — time from step start to the first sample with
  |err| ≤ 0.5 °C.
- **Crossing** — a heating step has "crossed" once `Tin ≥ target`; a cooling
  step once `Tin ≤ target`. Overshoot/undershoot/max-deviation are
  **undefined before crossing** — report `NA`, never 0.
- **Overshoot (°C)** — heating: `max(0, max(Tin after crossing) − target)`.
  Cooling: `max(0, target − min(Tin after crossing))`.
- **Undershoot (°C)** — the same quantity measured on the opposite side.
- **RMS error (°C)** — `sqrt(mean(err²))` over every valid sample in the step.
- **Max deviation (°C)** — `max(|err|)` after crossing.
- **Extreme (°C)** — hottest point of a heating step, coldest of a cooling step.
- **Duty (%)** — `sum(heat)/N` and `sum(fan)/N` over the step's samples,
  which equals relay ON-time as a fraction of step duration.

## 7. Controller parameters to report

**PID** (on-board, `Ts` = 1 s): `Kp = 150.0`, `Ki = 0.2`, `Kd = 15.0`;
output ±255; integral contribution clamped to ±38.25 (15 % of full scale);
derivative taken on the measurement, not the error; integral reset on
error sign change; relay deadband 64; time-proportioning window 2000 ms.

**MPC** (MATLAB Model Predictive Control Toolbox 26.1, running on a PC,
sending commands over serial at 1 Hz): first-order plant `K = 0.01232`,
`tau = 454 s`, `Ts = 1 s`; prediction horizon `p = 300`, control horizon
`m = 3`; manipulated variable limits ±255, rate limits ±60 per step;
weights — output 1, MV 0, MV-rate 0.1. Controlled in **deviation
variables**: `y = Tin − Tamb`, `ref = SP − Tamb`. Same relay deadband (64)
and same 2000 ms window as the PID, so duty figures are comparable.

**Protocol constants** (identical for both controllers): settling band
±0.5 °C held 300 s; baseline 120 s; test-3 dwell 120 s; lid open 5 s;
test-4 recovery dwell 60 s; per-step timeout 2400 s; targets ambient+5.0
(T1), ambient+0.0 (T2), ambient−2.0 (T3), ambient+2.0 (T4).

## 8. What to produce

**Table A — tests performed.** One row per test per controller: test
number, description, controller, source log, whether it produced valid
data, and if not, why.

**Table B — results.** Columns: Test | Controller | Target (°C) | Start
(°C) | Settled? | Settling time (s) | Overshoot (°C) | Undershoot (°C) |
RMS error (°C) | Max deviation (°C) | Extreme reached (°C) | Duration (s) |
Heat duty (%) | Fan duty (%).

Sources for each row:

- **PID tests 1, 2, 3** — compute from the raw samples in
  `device-monitor-260812-064112-test1-3.log` using section 6.
- **PID test 4** — read from the `RESULT,` line in
  `device-monitor-260812-094332-test4 only.log`.
- **MPC tests 1, 2** — compute from the raw samples in
  `mpc-test-260812-101350-test1-2.csv` using section 6.
- **MPC tests 3, 4** — read from the `RESULT,` lines in
  `mpc-test-260812-122724 test3-4.csv`.

Mark clearly in the table which rows were computed and which were read from
the firmware's own output, since the two came from different code paths.

**Table C — parameters**, as given in section 7.

**Table D — duty-cycle / energy comparison.** Per controller: total heat-relay
ON time, total fan-relay ON time, and each as a percentage of total test
duration.

**Notes section.** Flag every one of these that you confirm in the data:

- Any test that did not settle, with the actual plateau reached and the gap
  to its target.
- Any test whose *preceding stabilisation phase* failed, since its starting
  condition then violates the protocol.
- Any test-4 run where the peak deviation is < 0.3 °C — that means the lid
  was never actually opened and the recovery time is meaningless.
- Sensor dropout (`NaN`) counts per run.
- For MPC, samples with `link_ok = 0`.
- **Ambient drift between sessions.** Every target is defined relative to
  the ambient measured during that session's own baseline, so the four
  files do not share absolute setpoints. Report each session's baseline
  ambient and note that absolute temperatures are not directly comparable
  across files even though the relative targets are.
- The results come from four separate sessions run over one day, not one
  continuous experiment. Say so.
- Any cell you could not fill, and the reason.

Finally, state plainly which controller comparisons are actually supported
by this data and which are not.
