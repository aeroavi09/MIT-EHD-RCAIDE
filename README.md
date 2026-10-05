# MIT-EHD-RCAIDE
A model of the MIT solid-state (ionic wind) aircraft in RCAIDE, flown on an electrohydrodynamic (EHD) thruster. The aircraft is based on Xu et al., *Nature* 563, 2018: 5 m span, 2.45 kg, eight wire-to-NACA 0010 electrode units at about 40 kV. It uses the EHD thruster I added to RCAIDE, which lives on my fork: <a href="https://github.com/aeroavi09/RCAIDE_LEADS/tree/feature/ehd-propulsor">aeroavi09/RCAIDE_LEADS, branch feature/ehd-propulsor</a>.

<img src="MIT EHD Plots/EHD_Thrust_Sweep.png" width="90%">

What the script does
<ul>
  <li>Builds the aircraft: Clark Y main wing with dihedral tips, horizontal and vertical tail, battery, high-voltage converter and EHD thruster</li>
  <li>Runs a static thrust sweep (throttle 0 to 1, sea level, 0 m/s) and plots thrust, battery power and thrust-per-power against electrode voltage</li>
  <li>Flies 60 m of level flight at 5 m/s and 2 m altitude, solving throttle and pitch angle so thrust equals drag and lift equals weight</li>
  <li>Saves the mission plots (flight conditions, forces, coefficients, drag components, throttle, battery) to <a href="MIT EHD Plots">MIT EHD Plots</a></li>
  <li>Exports <code>MIT_EHD_aircraft.vsp3</code> with every emitter wire and NACA 0010 collector drawn in. The electrodes are display only; their drag is already inside the EHD propulsor model</li>
</ul>

Running it

The EHD thruster is not in the PyPI release of RCAIDE, so install RCAIDE from the fork (see `requirements.txt`), then run from this folder so `./Airfoils/` resolves:

```
python MIT_EHD_Aircraft.py
```

Results at the current settings: level flight at AoA 6.4 deg, C_L 0.78, L/D 10.2, 2.3 N of thrust and about 255 W from the battery over the 12 s run.

Caveats
<ul>
  <li>Chord, electrode layout, tail, battery and wing break location are placeholders (the <code>GUESS</code> block in the script). Only the values in the <code>MIT</code> block come from sources, and the 3.2 N thrust is from a secondary source</li>
  <li>The thruster is an uncalibrated 1-D model, so treat the numbers as a working model rather than a match to the real aircraft</li>
  <li>The same aircraft also lives in the fork as a test, <code>VnV/Verification/network_electric/ehd_mit_aircraft_test.py</code>. Keep the two in step</li>
</ul>

`EHD_PROJECT_NOTES.md` is the build log for the thruster work.
