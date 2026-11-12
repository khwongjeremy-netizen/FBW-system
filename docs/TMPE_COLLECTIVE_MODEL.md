# Configurable collective-pitch model

The local approximation is `CT(theta) = CT_reference + k_theta_per_deg * (theta - theta_reference_deg)`.
CT is dimensionless, theta and theta_reference are in degrees, and k_theta is CT per degree.
The equation contains no aircraft coefficients. Parameters and limits load from
`MY_physics_engine/data(v.2)/collective_model.json` using `LinearCollectiveModel.load()`.
An alternate JSON path can be supplied for isolated calibration or mathematical tests.

The shipped model is **UNCALIBRATED / UNAVAILABLE**. All three calibration parameters
are null. It never associates the existing fixed CT of 0.0102 with 10 degrees or any
other pitch. No fallback to the fixed coefficient occurs when collective is requested.
The existing fixed-CT implementation remains available explicitly; the CLI now defaults to collective mode.

## Parameter evidence and missing data

| Parameter | Repository state | Needed evidence |
| --- | --- | --- |
| CT_reference | Missing for collective model | Measured per-rotor thrust at a documented pitch and operating condition, with density, radius and RPM to calculate CT |
| theta_reference_deg | Missing | Actual collective at the same thrust measurement; pitch datum, blade radial station and control calibration |
| k_theta_per_deg | Missing | At least two paired CT/pitch points at the same condition; preferably a curve with uncertainties to fit and assess a local slope |
| Pitch operating interval | Aircraft limits missing | Documented mechanical limits and aerodynamic validity interval for the fitted curve |
| Reference condition | Missing | RPM, density, inflow/advance ratio, climb rate, height/ground effect, nacelle angle and rotor/airframe configuration |
| Fixed-model CT | 0.0102 illustrative equilibrium assumption | No known pitch pairing; cannot serve as collective calibration |
| Diameter / hover RPM | Existing XML reports 25 ft / 565 design RPM | These permit SI thrust conversion, but do not establish pitch sensitivity or measured rotor performance |

Configuration requires a source, reference condition, all three finite model parameters,
a parameter_status of `calibrated` or `illustrative`, and sourced pitch bounds. Bounds
must identify their basis as `documented` or `provisional-software`. The shipped
[-30, 30] degree interval is solely a provisional software input guard chosen for
this implementation. It is neither a known XV-15 limit nor a calibrated validity
range and does not make the unavailable model usable. Replace it with the documented
intersection of the aircraft operating limits and local fit interval when available.
The ±12 degree flapping limits in rotor.xml describe flapping, not collective travel.

`calibrated` records the configuration author's calibration assertion; it does not
assert independent experimental validation. A dataset should identify source/table,
units, CT convention, fit method and uncertainty in its source/reference-condition
metadata. A measured curve should be used instead of a linear fit outside a supported
local interval. Negative/nonfinite calculated CT is unavailable rather than clamped.
Zero CT and zero RPM give zero thrust; negative-thrust operation is outside this model.

## API and simulation connection

```python
model = LinearCollectiveModel.load()
left = calculate_proprotor(
    'left', aircraft, environment, rotor_model='linear-collective',
    collective_deg=left_pitch_deg, ct_model=model)
right = calculate_proprotor(
    'right', aircraft, environment, rotor_model='linear-collective',
    collective_deg=right_pitch_deg, ct_model=model)
```

Each call evaluates its own pilot input; each rotor may use a different model instance
or configuration. Inputs must be finite numeric degrees, including on an uncalibrated
model, and must satisfy the configured bounds. Missing/malformed data yield None with
an explicit reason, never a thrust prediction. The CLI now supports `--rotor-model prototype-collective` and prompts separately for left and right pitch. The default is `prototype-collective`.

Supported CT feeds the existing independent per-rotor SI equation:
`T = CT * rho * A * (Omega * R)**2`, where `A = pi*R**2`,
`R = diameter_ft*0.3048/2` and `Omega = RPM*2*pi/60`.
It does not infer engine, shaft or induced power.

For simulation, the existing `simulate_takeoff(..., force_provider=provider)` extension
accepts a function `(time, height, upward_velocity)` returning
`left_proprotor_thrust_n`, `right_proprotor_thrust_n`, and `other_vertical_force_n`.
That function can call the two rotor evaluations above and pass their `value` fields;
unavailable values stop integration. Other forces must be explicitly established or
explicitly excluded by the caller; the new collective API does not silently zero them.
This keeps force balance, integration and the working fixed-CT takeoff path unchanged. The new built-in `prototype-collective` simulation mode also connects directly to these same force and integration functions, explicitly excluding other loads for a software mechanics exercise.

A future curve or BEMT adapter implements `evaluate(theta, *, allow_illustrative=False)`
and returns `value` (CT or None), `status`, `reason`, `calibration_status`, and
`experimental_validation_status`. `calculate_collective_proprotor` accepts that adapter
through `model` (or dispatch's `ct_model`) without changing the simulation. A richer
adapter can capture RPM, inflow and other operating inputs in its own context.

## Verification and validation

Illustrative configurations require `parameter_status: illustrative` and explicit
`allow_illustrative=True`; results carry **ILLUSTRATIVE — MATHEMATICAL TEST ONLY**.
Arbitrary coefficients exist only in automated test fixtures, never in shipped aircraft
configuration. These tests establish algebra, reference identity, slope, thrust coupling,
independent inputs, finite-input rejection, bounds, zero thrust, negative CT rejection,
and density/RPM scaling. Existing fixed-CT regression tests continue to pass.
Run `python3 -m unittest discover -s tests -q`.

Experimental validation is separately reported as **NOT VALIDATED**, even when a
configuration asserts calibration. Calibration requires paired rotor measurements;
validation requires independent holdout rotor/flight measurements, uncertainty and
residual analysis over the intended envelope. Neither mathematical test success nor
an equilibrium constructed using 0.0102 validates XV-15 performance. The local model
omits inflow response, stall, power/torque limits, ground effect, interference and wind;
it cannot establish realistic takeoff or climb performance.


## CLI debugging result

The initial implementation exposed only the `linear-collective` Python API. Argparse
and `simulate_takeoff` accepted only `unsupported` and `prototype-fixed-ct`; there was
no pitch prompt. The CLI default therefore continued to show the fixed coefficient.
The minimal wiring now accepts `prototype-collective`, loads `--collective-config`
(or the uncalibrated default), asks each pilot pitch and passes it to the collective
calculation on every force evaluation. `--prototype-ct` remains exclusive to fixed CT.

Run the aircraft configuration (expected UNCALIBRATED / UNAVAILABLE):

```sh
python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --rotor-model prototype-collective
```

Run explicitly illustrative mathematical testing:

```sh
python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --rotor-model prototype-collective --collective-config tests/fixtures/collective_illustrative.json --allow-illustrative-collective
```

The synthetic fixture uses CT_reference=0.009, theta_reference=4 degrees and
k_theta=0.001/degree. These are arbitrary test values, not XV-15 estimates. Input
4 degrees on both sides produces CT=0.009 and grounded motion; 6 degrees produces
CT=0.011 and about 9.81 m height after five seconds under the excluded-load exercise.
Both mathematical success and changing motion are software checks, not experimental
validation. See [actual commands and terminal output](TMPE_COLLECTIVE_CLI_RUNS.md).


## Normal execution and interactive testing

Running the Python file without arguments (including VS Code Run Python File) now
selects `prototype-collective`. The five environmental questions are unchanged.
With the shipped uncalibrated configuration, the program lists missing calibration
fields and asks `Enable illustrative collective testing? [y/N]:`. Only `y` or `yes`
enables `tests/fixtures/collective_illustrative.json`; Enter, `n` or `no` retains the
unavailable aircraft model. Other responses are rejected and prompted again.
The fixture path resolves from the source file, independent of the working directory.

After that choice, left and right pitch are prompted separately with the selected
configuration's sourced bounds displayed. Illustrative results remain labeled as
mathematical testing and NOT VALIDATED. Selecting 6 degrees on both sides after
opting in produces approximately 9.81 m after five simulated seconds.

`--allow-illustrative-collective` explicitly selects the existing fixture when no
`--collective-config` is supplied. An explicit configuration path is honored as
supplied and illustrative data there still require the explicit opt-in flag.
The low-level physics API defaults remain unchanged.

Fixed-CT regression execution:

```sh
python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --rotor-model prototype-fixed-ct
```

See [default-mode execution evidence](TMPE_DEFAULT_COLLECTIVE_RUNS.md) for actual
commands and outputs. The automated suite passed 57 tests after this default change.
Earlier CLI execution logs and the ChatGPT handoff describe the prior default and
are historical records; this section describes current startup behavior.
