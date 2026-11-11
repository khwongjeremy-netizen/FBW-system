# Maintaining and debugging TMPE

## Primary takeoff path

`main()` creates `Aircraft`, reads the five environmental inputs and calls
`simulate_takeoff()`. Each rotor is checked separately by `calculate_proprotor()`.
With the current XML data, each returns `UNAVAILABLE` because commanded mast power
and a supported thrust/torque model are missing. Rated SHP is only inventory.

`vertical_force_balance()` computes weight and, when supplied, projected rotor
forces and other vertical loads. `integrate_vertical()` advances a known force
balance with a constant-acceleration step and unilateral ground constraint.
`simulate_takeoff()` accepts an explicit external force provider for future models
and isolated mechanics tests. Missing forces preserve state and elapsed time; they
never become zero thrust. No actual XV-15 performance has been validated.

`--diagnostics` prints the XML ledger and primary/secondary blockers; `--verbose`
adds force equations and the separately identified wing AoA trace. The AoA path
is no longer used by the takeoff calculation.


## How the Python fits together

- `Aircraft.__init__()` lists the ordinary fields as tuples: Python attribute, XML file, element/attribute path, expected unit, meaning and whether live physics uses it.
- `load_parameter()` reads one field, validates numeric values and units, and stores a dictionary in `Aircraft.diagnostics`. It returns the value, or `None` after recording a specific failure. Metadata such as source and verification flags can be inherited from parent XML elements.
- `load_recovery_items()` retains NotebookLM citations and validates each recovery item separately.
- `Aircraft._recovered_group()` builds converted geometry only if every input is available. Its diagnostic record names the individual input records so a derived failure can be traced to the actual XML item.
- `calculate_aoa()` and `wing_aerodynamics()` retain the existing physics. `physics_diagnostics()` classifies their returned results without modifying them.
- `report_diagnostics()` prints the load ledger, runtime calculation statuses and blockers. `report_aoa()` handles the detailed verbose trace.
- The original `fetchvalue()`, `fetchpara()` and `try_load()` helpers remain callable for compatibility; aircraft initialization now uses the tracked loaders.

## Load record example

```python
row = aircraft.diagnostics['wing_incidence_deg']
print(row['status'], row['value'])
print(row['file'], row['path'], row['reason'])
```

If `geometry/wing/incidence` is absent, the value becomes `None` and the record says `MISSING`, with the exact file and element path. Airspeed still calculates; wing AoA becomes `UNAVAILABLE` because incidence is missing. The diagnostic blockers point back to that input.

If the element contains `abc`, `nan` or an incompatible unit, the record says `INVALID` and includes the exception reason. Empty values are invalid; absent elements or attributes are missing. Zero is a successful numerical value, never a substitute for `None`.

Lengths accept inches, feet or metres and convert to the requested unit. Dimensionless legacy fields may omit `unit`; dimensional fields must declare it. The report displays both source and output units when they differ.

## What statuses mean

- Aircraft `LOADED`: parsing and conversion succeeded; this does not certify source accuracy.
- Aircraft `UNVERIFIED`: a value or containing XML section explicitly flags unverified data. The value can still be inspected and used where the existing model already uses it.
- Aircraft `MISSING`: absent data, including deliberately unavailable scalar width and aerodynamic polars. The reason distinguishes intentional `None` from failed loading.
- Aircraft `INVALID`: malformed XML, invalid numeric content or incompatible units.
- Physics `CALCULATED`: an existing function produced a numerical result, including exactly zero freestream force.
- Physics `UNAVAILABLE`: required data or a supported model is missing, or AoA is undefined at insufficient chord-plane speed.
- Physics `UNSUPPORTED`: a nonzero operating condition is outside the current forward-flow aerodynamic assumptions. A kinematic AoA can still be calculated in reverse flow.
- Physics `NOT IMPLEMENTED`: a required model does not exist, such as the current actual XV-15 rotor performance model. Vertical mechanics is implemented, but aircraft takeoff remains unavailable until forces are established.

The ledger covers fields the program attempts to load, not every research element in the XML inventory. Fields marked “not used in live physics” are data inventory or future model inputs. Approximate aerodynamic descriptors do not provide supported coefficient arrays.
