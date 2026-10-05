# EHD Thruster in RCAIDE: Project Notes (start to end)

Session date: 4 Oct 2026. Repo: `C:\RCAIDE\My Work\RCAIDE_LEADS` (local clone of RCAIDE_LEADS 1.5.0),
branch `feature-ehd_thruster`.

---

## 1. Goal

Add an electrohydrodynamic (EHD, "ionic wind") thruster to RCAIDE so a battery-electric UAV can fly a
mission on EHD thrust, with power and battery state of charge (SOC) adding up. Built exactly from the
spec PDF *RCAIDE and Electrohydrodynamic UAV Propulsion: Current State, Usable Physics, and an
Implementation Plan* (Revision 2.1, in `My Work`), using the wire-to-NACA 0010 design: one thin
positive emitter wire in front of one NACA 0010 collector, repeated N times.

---

## 2. Timeline

| Step | What happened |
|---|---|
| 1 | Read the RCAIDE_LEADS folder and the first spec PDF; said the project was feasible. |
| 2 | You gave the full build brief (3 phases, scope limits, stop conditions). Spec PDF had been updated to Revision 2.1 (adds sections 2.4, 3.3.1, 3.3.2, 3.7), so it was re-read in full, including formula pages as images. |
| 3 | Hand-checked the spec's reference numbers (MIT-like unit: 120.3 kV/cm, 7.70 kV, 1.287 N, 172.9 W). They are self-consistent. |
| 4 | **Phase 1, reading RCAIDE.** Mapped the Propulsor interface, the electric-rotor chain, the network evaluate loop, segment field names, and tests. Key finding: the network calls any propulsor generically, so **no Framework edits were needed**. Wrote `ehd_spec/IMPLEMENTATION_NOTES.md`. Commit `0f0bd41e1`. |
| 5 | **Phase 2, implementation.** 3 component classes, 3 method packages, 6 one-line `__init__.py` registrations. Commit `73e7a1051`. |
| 6 | **Phase 3, tests.** 22 physics/unit/trend tests + a cruise mission test, all passing. Extended the mission test to two identical thrusters to exercise the "reuse stored data" path. |
| 7 | Started before/after runs of existing RCAIDE tests. First attempt was contaminated (ran while code changed); redone from a clean copy of the pre-change code. |
| 8 | VS Code couldn't import the EHD code (it used the PyPI RCAIDE). Tried a workspace-wide `.env` fix, **reverted** at your request (you wanted it EHD-only). |
| 9 | Chose the upstream-friendly setup: new conda env **`rcaide1.5.0EHDproj`** with the clone installed via `pip install -e .`. Norton Antivirus blocked conda's SSL; worked around per command. |
| 10 | Built the **MIT-like aircraft test** (placeholders for unknown geometry) and **OpenVSP export** with electrodes drawn in. Installed OpenVSP 3.47 into the new env. Found a bug in RCAIDE's VSP exporter. |
| 11 | Made a project folder, reverted it (you just couldn't find the file), then made it properly: `MIT EHD RCAIDE\` with a runnable script and the `.vsp3`. |
| 12 | This note sheet. |
| 13 | **Phase 3 commit** `56928eba2` (tests + notes) and **pushed** branch `feature-ehd_thruster` to your fork `aeroavi09/RCAIDE_LEADS`. |
| 14 | Rewrote `MIT_EHD_Aircraft.py` as one self-contained script in the C-17 style (own `Airfoils/`, plots saved to `MIT EHD Plots/`, VSP export). |

---

## 3. What was built (all inside `RCAIDE_LEADS`)

**Components** (`RCAIDE/Library/Components/Powertrain/`)
- `Propulsors/EHD_Thruster.py`: the propulsor; owns the electrode array and the HV converter. No solver unknowns (throttle maps straight to voltage).
- `Converters/EHD_Electrode_Array.py`: all electrode inputs, sorted into the spec's six roles. Fixed values (NACA 0010, positive polarity, 1 emitter per collector) raise an error if changed.
- `Modulators/High_Voltage_Converter.py`: fixed-efficiency HV converter (85%, 1.15 kW/kg, 160-225 V input). The existing ESC couldn't be reused.

**Methods** (`RCAIDE/Library/Methods/Powertrain/`)
- `Converters/EHD_Electrode_Array/`: Peek inception (R19, R20), space-charge current/thrust/power (R21, R22), electrode drag (R24), mass, input checks/warnings.
- `Modulators/High_Voltage_Converter/`: conditions, performance, mass.
- `Propulsors/EHD_Thruster/`: the 10-step per-time-step calculation, reuse path, total mass, design/sizing (`design_ehd_thruster`), optimizer-input check.

**Tests** (`VnV/Verification/`)
- `propulsion/ehd_thruster_physics_test.py`: 22 tests.
- `network_electric/ehd_thruster_mission_test.py`: cruise mission with 2 thrusters.
- `network_electric/ehd_mit_aircraft_test.py`: MIT-like aircraft (sweep, level flight, VSP export).

**Docs**
- `ehd_spec/IMPLEMENTATION_NOTES.md`: verified interfaces with line numbers, file map, spec-to-code mapping, 17 decisions/conflicts.

**Outside the repo**
- `My Work\MIT EHD RCAIDE\MIT_EHD_Aircraft.py`: all-in-one script (vehicle, analyses, mission, thrust sweep, plots, VSP export), same layout as `C17_Globemaster_III.py`. The same aircraft is also in the repo test; keep the two in step.
- `My Work\MIT EHD RCAIDE\Airfoils\Clark_y.txt` and `MIT EHD Plots\` (7 PNGs).
- `My Work\MIT EHD RCAIDE\MIT_like_EHD_aircraft.vsp3`: OpenVSP model.
- This file.

---

## 4. The physics in one page

Per unit (one wire + one collector), each time step:
1. Relative air density δ = (p/p₀)(T₀/T), reference 25 °C / 1 atm.
2. Corona inception field (Peek, R19) and inception voltage V_i = E_i·a·ln(d/a) (R20), or a measured V_i.
3. Voltage V_a = V_i + throttle·(V_max − V_i).
4. Dimensionless current ĵ(V_a/V_i) (R21): 0 below inception, 9/8 at very high voltage.
5. Thrust T_unit = ε₀·(V_a/d)²·ĵ·S·b.
6. Power P_unit = ε₀·μ·V_a³/d³·ĵ·S·b (so T/P = d/(μV_a) exactly).
7. Drag of collector (R24 or user c_d) and wire.
8. Net thrust = N·(T_unit − drags).
9. Battery-side power = N·P_unit / converter efficiency.

Caveat from the spec: this 1-D model **over-predicts** thrust and current until calibrated with
bench data (k_T, k_P, k_Vi factors, Stage 2).

---

## 5. Results

**New tests:** all pass (22 physics + 1 mission + 3 MIT).
- Spec reference vectors reproduced to 3 significant figures (MIT-like and Kahol-like rows).
- ĵ values 0.8080 / 1.0695 / 1.1250 exact; T/P identity to machine precision.
- Cruise mission (5 kg UAV, 9 m/s, 2 km): converged, throttle 0.788, 33.4 kV, 4.20 N, 680 W, SOC 1.000 → 0.946; battery energy matches integrated power to 1e-6.
- MIT-like level flight (2.45 kg, 5 m/s, 60 m): converged, 3.52 N, throttle 0.563, 26.1 kV, 415 W.

**Existing RCAIDE tests, before vs after the EHD code:**

| Test | Before | After |
|---|---|---|
| battery_electric_aircraft_test | pass | pass |
| electric_ducted_fan_aircraft_test | pass | pass |
| motor_test | pass | pass |
| rotor_performance_test | pass | pass |
| battery_cell | pass | pass (rerun; first run was cut off during process cleanup) |
| vtol_aircraft_test | still running (40+ min test) | still running |

---

## 6. Environments

| Env | Use | Notes |
|---|---|---|
| `rcaide1.5.0` | F-15, C-17, other projects | PyPI RCAIDE 1.5.0. **Does not have the EHD code.** Has OpenVSP. |
| `rcaide1.5.0EHDproj` | All EHD work | Clone installed editable (`pip install -e .`), pytest, OpenVSP 3.47 (from `C:\VSP313\python`). Newer numpy/scipy. |
| `rcaide1.2.1` | (gone) | Your `.vscode/settings.json` still points to it. |

- **VS Code:** for EHD files, run "Python: Select Interpreter" → `rcaide1.5.0EHDproj`.
- **Norton:** it intercepts HTTPS, so conda fails with `CERTIFICATE_VERIFY_FAILED`. Fix per command in PowerShell: `$env:CONDA_SSL_VERIFY='truststore'` before the conda command. pip worked without it.

---

## 7. Decisions and deviations from the spec

- **ε₀** isn't given as a number in the spec; used `scipy.constants.epsilon_0` (scipy is already an RCAIDE dependency).
- **Collector drag**: spec gives laminar and turbulent friction laws with no rule; default is laminar (spec says Re ≈ 10⁴), selectable.
- **"Voltage near inception" warning**: spec gives no threshold, so not implemented (would need an invented number). Gap 10-300 mm warning is implemented.
- **Spacing and chord locked from optimization**: RCAIDE has no built-in lock, so it's a permissions table plus `check_ehd_optimizer_inputs(problem)` that you call after setting up a Nexus problem.
- **`spacing_correction` hook**: exists but raises if used (spec says keep it disabled; its formula is garbled in the PDF).
- **Weight buildups** don't know the EHD thruster; aircraft mass is set by hand in the tests.
- **Converter rated power** sets mass only; no overload check (the MIT-like sweep shows 1.6 kW at full throttle vs a 600 W converter, so treat high throttle as unrealistic).
- **Thrust direction**: body +x, same convention as RCAIDE's turbofan/turbojet.
- New tests are not added to RCAIDE's regression list (that would edit an existing test file).

---

## 8. Problems found in RCAIDE itself

1. **OpenVSP exporter bug**: `RCAIDE/Framework/External_Interfaces/OpenVSP/export_vsp_vehicle.py` lines 133-136 only write wings when `verbose=True`. Good small first upstream PR.
2. Bus-level converters in the network are type-checked to motors/generators only, so an HV converter has to live inside the propulsor.
3. Running the existing `rotor_performance_test` rewrites `VnV/Verification/propulsion/test_rotor.res` in the repo (shows as modified in git; needs restoring before committing).

---

## 9. Still to do

- [x] Phase 3 commit and push to fork (`origin/feature-ehd_thruster`).
- [ ] Finish VTOL before/after comparison.
- [ ] Check whether `My Work\1292944621-MIT.pdf` is the Xu et al. Nature paper and replace the MIT placeholders (chord, spacing, unit layout, wing, battery).
- [ ] Stage 2 (spec): calibrate k_T, k_P, k_Vi, c_d from bench data; add speed, humidity, pressure corrections.
- [ ] For upstream: rebase onto `upstream/develop`, register the tests in `VnV/test_automatic_regression.py`, open a PR to `develop`. Separate small PR for the VSP exporter bug.

---

## 10. How to run

```
:: MIT-like aircraft: sweep, level flight, plots, writes the .vsp3 next to the script
C:\Users\avite\anaconda3\envs\rcaide1.5.0EHDproj\python.exe "C:\RCAIDE\My Work\MIT EHD RCAIDE\MIT_EHD_Aircraft.py"

:: All EHD tests
cd "C:\RCAIDE\My Work\RCAIDE_LEADS"
C:\Users\avite\anaconda3\envs\rcaide1.5.0EHDproj\python.exe -m pytest VnV/Verification/propulsion/ehd_thruster_physics_test.py VnV/Verification/network_electric/ehd_thruster_mission_test.py VnV/Verification/network_electric/ehd_mit_aircraft_test.py -q
```

Placeholder values for the MIT aircraft live in the `GUESS` block of
`RCAIDE_LEADS/VnV/Verification/network_electric/ehd_mit_aircraft_test.py`.
