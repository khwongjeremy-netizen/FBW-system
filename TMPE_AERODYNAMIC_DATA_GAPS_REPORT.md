# TMPE aerodynamic data gaps: research handoff for ChatGPT

Prepared 7 November 2026 from the current repository. This report is self-contained so it can be uploaded to another ChatGPT conversation. File paths are relative to the repository root; line numbers identify the inspected snapshot and may move after edits.

**Research objective:** identify the exact data, definitions and implementation needed to calculate supported XV-15 wing lift. Trace each missing item to an original source, distinguish missing extraction from unavailable source data, and recommend the smallest defensible next implementation.

**Current finding:** TMPE calculates freestream wing angle of attack and airspeed, but has no implemented lift-coefficient evaluator. Its XML contains approximate curve descriptors and references to figures, with explicit notices that numerical lookup arrays were not extracted. The code also lacks the configuration state and source-definition mapping needed to use those curves consistently. These are several distinct gaps, rather than one missing constant.

This report does not supply a replacement coefficient curve or change the simulation. Original-source locations copied from XML are research leads unless explicitly identified below as independently checked.

## 1. Project scope and the question to answer

TMPE means Tiltrotor Modeling Physics Engine. It is an educational Python prototype using the NASA/Bell XV-15 as a reference. It reads selected XML aircraft parameters, collects environmental inputs and prescribed initial conditions, and computes freestream airspeed and wing AoA. It does not integrate aircraft motion or implement rotor thrust, rotor wash, flight-by-wire, or pilot-command response.

The immediate question is:

> What exactly must be recovered from the original aerodynamic sources, and what must be implemented in Python, before TMPE can report a supported nonzero wing lift for a clearly defined operating condition?

Answer this first for a limited forward-flight, freestream model. Treat hover wash, transition, post-stall, reverse flow, full aircraft dynamics and rotor-blade aerodynamics as separate extensions. Identify explicitly when an extension is necessary for a particular requested case.

## 2. Evidence map in the repository

| File and location | What it establishes |
|---|---|
| `MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py:81` | `Aircraft` loads selected mass, geometry and rotor fields. It does not load `aerodynamics.xml`. |
| Same Python file, line 155 | `AircraftState` contains prescribed NED ground velocity and roll/pitch/yaw. It contains no actual nacelle/flap/flaperon state. |
| Same Python file, line 188 | `calculate_aoa()` subtracts wind, transforms world to body to reference-wing coordinates, and calculates AoA. |
| Same Python file, line 231 | `wing_aerodynamics()` returns zero freestream force at exactly zero speed; otherwise it returns unavailable. It evaluates no coefficients. |
| Same Python file, line 288 | `report_results()` displays wing lift as unavailable unless the model returned exactly zero force. The message is a presentation summary. |
| Same Python file, line 414 | `calculate_lift()` contains the scalar lift equation, but is not called by the interactive calculation path. |
| Same Python file, line 495 | `main()` obtains the prescribed state, calculates AoA, calls the unavailable-force stub, then reports results. |
| `MY_physics_engine/data(v.2)/geometry.xml:13` | Wing dimensions, area, incidence and airfoil designation. |
| `MY_physics_engine/data(v.2)/aerodynamics.xml:3` | Wing/pylon figure metadata, approximate descriptors, configuration labels and missing-array notice. |
| `MY_physics_engine/data(v.2)/aerodynamics.xml:21` | Repeated scalar descriptors under `lifting_surfaces/wing`, without a full coefficient dataset. |
| `MY_physics_engine/data(v.2)/review/re-interation/data_gaps.xml:6` | G3 records missing aerodynamic arrays; G4 records missing curve digitization. |
| `MY_physics_engine/data(v.2)/review/re-interation/review_flags.xml:27` | R6 warns that approximate slopes and stall descriptors do not replace full nonlinear tables. |
| `MY_physics_engine/data(v.2)/sources.xml:20` | S4 identifies NASA CR-166536 as the aerodynamic math-model source. |
| `MY_physics_engine/data(v.2)/sources.xml:32` | S8 identifies NASA CR-166537 as a companion implementation/data-deck research lead. |
| `MY_physics_engine/data(v.2)/controls.xml` | Mechanical limits and conversion ranges, rather than the actual current aerodynamic configuration. |
| `MY_physics_engine/data(v.2)/rotor.xml:63` | Separate missing rotor-model inputs. |
| `MY_physics_engine/data(v.2)/wake_interference.xml` | Reported wake/download observations with applicability cautions; no implemented local-flow model. |
| `tests/test_tmpe.py` | Eight tests of current AoA/frame/undefined-flow behavior. They do not validate an aircraft coefficient model. |

The repository also contains `MY_physics_engine/data(v.2).zip`. Its archived aerodynamic XML was inspected: it also contains figure metadata and the missing-array notice. It is not a hidden complete coefficient database. The loose XML and archive differ, so their versions should not be silently mixed. Uploading only that archive would supply an older dataset snapshot.

## 3. Inputs already available

The scalar lift relationship already present in Python is:

```text
q = 0.5 * rho * V_air**2
L = q * S_ref * CL
```

| Quantity | Current availability | Remaining qualification |
|---|---|---|
| Air density `rho` | User input; default 1.225 kg/m³ | Prescribed, not derived from an atmospheric model. |
| Aircraft ground velocity | Prescribed NED vector in m/s | Not a simulated response to a command. |
| Wind velocity | Calculated from wind speed and FROM direction | Horizontal, spatially uniform wind; no vertical wind. |
| Airspeed `V_air` | Magnitude of aircraft velocity relative to air | Freestream only. Verify the source model's dynamic-pressure definition before reuse. |
| Aircraft attitude | Prescribed roll/pitch/yaw | Not integrated or solved from trim. |
| Wing incidence | XML value 3.0° | Present in the dataset; original-source applicability still needs checking. |
| Wing reference-frame AoA | Calculated internally | Must be mapped to the source curve's AoA datum. |
| Geometric wing area | XML value 169.0 ft² = 15.70061376 m² | Verify that the coefficient source uses this same reference area and component scope. |
| `CL` | No evaluator or numerical table loaded | Immediate coefficient gap for nonzero wing lift. |

Other recorded wing geometry is span 32.17 ft, chord 5.25 ft, aspect ratio 6.12, dihedral 2.0°, quarter-chord sweep −6.5°, and airfoil `NACA 64A223 (Mod)`. These are dataset entries, not independent validation in this report. The current reference-wing AoA transformation uses incidence only; sweep and dihedral do not enter it.

`CD` is needed to calculate drag, and `Cm` is needed to calculate a pitching moment. Neither is inherently required to report a scalar lift under a supported lift-only model. A complete force vector, total aircraft force, pitching response and trajectory require additional work.

## 4. Current coordinate and AoA definitions

The implemented conventions are:

- World axes: north, east, down, abbreviated NED.
- Body axes: x forward, y right, z down.
- Positive roll lowers the right wing; positive pitch raises the nose; positive yaw turns north toward east.
- Attitude is a yaw-pitch-roll, 3-2-1 rotation. The world-to-body matrix is the transpose of the body-to-NED matrix.
- Internal angles use radians. Terminal angles use degrees.
- Wind direction is meteorological FROM, clockwise from true north.
- Wing-frame x points forward along the chord, y right, and z completes the right-handed frame, downward for an unrotated wing.

The calculation is:

```text
d = wind FROM direction, converted to radians
V_wind_NED = (-wind_speed*cos(d), -wind_speed*sin(d), 0)
V_air_NED = V_ground_NED - V_wind_NED
V_air_body = R_world_to_body @ V_air_NED

i = wing incidence, converted to radians
u_w = cos(i)*u_b - sin(i)*w_b
v_w = v_b
w_w = sin(i)*u_b + cos(i)*w_b

alpha_body = atan2(w_b, u_b)
alpha_wing = atan2(w_w, u_w)
```

`V_air` describes aircraft velocity relative to air; incoming airflow points in the opposite direction. AoA is unavailable when chord-plane speed is below `1e-6 m/s`. Purely spanwise flow also leaves AoA undefined. Exactly zero freestream speed gives zero modeled freestream force without evaluating a coefficient. Rotor wash can produce wing loading in hover, but that local flow is excluded.

**Definition gap to investigate:** do the source tables use wing-chord AoA, fuselage/body AoA, a wing/pylon subsystem angle, or a corrected local-flow angle? The current 3° incidence must not be counted twice. Establish this from source equations and symbol definitions before feeding `alpha_wing` into an extracted curve.

## 5. What the aerodynamic XML actually contains

Under the root `xv15_dataset`, the relevant path is `aerodynamics/wing_pylon_coefficients`. It is marked `status="partially_verified"` and `source="S4"`.

| XML field | Stored content | What is still absent |
|---|---|---|
| `lift_curve` | Figure 8; AoA range −40° to +40°; plotted coefficient range −1.2 to +2.0; nacelle cases 0° and 90°; flap/flaperon cases 0/0 and 40/25° | Actual paired alpha/CL values and the exact association of each curve with configuration. Plot-axis limits are not data points or valid-domain guarantees. |
| `approximate_lift_slope` | 0.085 per degree and approximately 4.87 per radian | Verified linear-fit interval, configuration and angle datum. |
| `approximate_zero_lift_angle` | −2.5° | Verified configuration and angle reference. |
| `approximate_stall_angle` | 14° | Curve/configuration-specific stall behavior and fit limits. |
| `approximate_max_lift` | 1.35 for 0/0; 1.62 for 40/25 | Full approach to stall and post-stall shape; exact operating conditions. |
| `lift_compressibility_corrections` | Figure 9; Mach cases listed as 0–0.2, 0.4, 0.5, 0.6 | Numerical correction values and the equation defining how to apply them. |
| `large_negative_angle_lift` | Figure 10; −100° to 0°; flap/flaperon 40/25° | Curve coordinates, component scope and extension rules. |
| `drag_curve` | Figure 11; −40° to +40°; configuration labels | Numerical CD values and coefficient definitions. |
| `drag_compressibility_corrections` | Figure 12 | Numerical corrections and combination rules. |
| `large_angle_drag` | Figure 13; −100° to +100° | Numerical coordinates and applicability. |
| `pitching_moment_coefficient` | `Cm_WP = f(beta_m, delta_F)` and candidate source locations | Numerical values, symbol definitions, moment reference, and the complete source equation. |
| `numerical_lookup_arrays` | Explicitly says actual arrays were not included in the supplied extraction | A usable source-derived aerodynamic dataset. |

The `lifting_surfaces/wing` block repeats the slope, zero-lift angle, stall angle and maximum-lift descriptors. This repetition does not provide an independent source or complete curve.

The XML's general missing-data note mentions `Cm(alpha)`, while its more specific moment metadata says `Cm_WP = f(beta_m, delta_F)`. Research the original model before assuming that a one-dimensional `Cm(alpha)` table is what must be recovered. In particular, do not assume what the symbol `beta_m` means from the letter alone.

## 6. Exact missing data and definitions

| ID | Missing item | Why it matters | Minimum resolution for a limited wing-lift model |
|---|---|---|---|
| A1 | Numerical CL curve or a supported analytic approximation | AoA alone cannot determine CL | Recover one source-supported configuration with a documented valid interval. |
| A2 | Source AoA datum and positive direction | Prevents sign mistakes and counting incidence twice | Document the conversion from TMPE's angle to the source's angle. |
| A3 | Coefficient component scope and reference area | Wing, wing+pylon, airfoil section and whole-aircraft coefficients are different quantities | Establish which components the curve includes and its exact `S_ref`. |
| A4 | Dynamic-pressure/velocity definition | Determines what speed belongs in `q` | Establish freestream versus local effective flow and any source correction. |
| A5 | Exact curve configuration | Flaps, flaperons and nacelle position can distinguish curves | Recover curve labels and select a real configuration explicitly. |
| A6 | Validity bounds and out-of-range policy | A known angle can still lie outside a supported coefficient model | Establish alpha bounds and flow restrictions; return unavailable outside them. |
| A7 | Interpolation and correction rules | Sparse data require a documented evaluation method | Identify alpha interpolation and any configuration/Mach interpolation or correction required by the source. |
| A8 | Extraction provenance and uncertainty | Scanned figures and OCR can introduce substantial errors | Record report, page, figure/table, extraction method and uncertainty. |
| A9 | Mach/Reynolds and sideslip applicability | May limit transfer of source data to the requested case | Establish stated source conditions; restrict the initial model if these dependencies are unsupported. |
| A10 | Rotor-flow inclusion/exclusion in source coefficients | Prevents omitting or counting wake effects twice | Establish whether a selected curve is usable as a freestream-only model. |

For the first implementation, these do not all require new multidimensional models. Some can be resolved by restricting the valid operating domain to a single documented configuration and suitable flow conditions. State those restrictions explicitly.

A section airfoil polar for unmodified NACA 64A223 is not automatically an interchangeable XV-15 wing/pylon curve. The local designation includes “Mod”, and the repository's cited curve describes a finite wing/pylon subsystem. Any substitute needs a stated rationale, corrections and applicability limits.

## 7. Exact missing implementation

These code gaps remain even if the missing source values are found:

1. **Aerodynamic data loading.** `Aircraft.__init__()` reads `mass.xml`, `geometry.xml` and `rotor.xml`; there is no aerodynamic dataset loader or coefficient-model object.
2. **Configuration state.** Mode and pilot command are labels. There is no numerical current nacelle angle, flap angle or flaperon angle used to choose a curve. `controls.xml` limits do not supply those actual state values.
3. **Coefficient evaluation.** There is no CL interpolation, analytic evaluator, source-angle conversion, model-domain check, configuration selection, or correction application.
4. **Lift calculation wiring.** `calculate_lift()` already implements `0.5*rho*V**2*S*CL`, but the live report never calls it.
5. **Aerodynamic input plumbing.** `wing_aerodynamics(flow)` receives only the flow dictionary. Density, area, coefficient data and configuration would need to be passed or otherwise supplied through a defined interface.
6. **Separate result statuses.** AoA validity, lift-model validity, drag-model validity and moment-model validity need separate results. Missing CD/Cm should not automatically suppress supported scalar lift.
7. **Force-direction calculation.** No nonzero lift or drag vector is implemented. A future force calculation needs directions consistent with the coefficient axes and airflow. Wing lift must not be treated automatically as a vertical body/world force.
8. **Validation.** Existing tests establish some kinematic behavior and missing-model handling, rather than agreement with published aerodynamic coefficient values.

The essential existing force-model behavior is:

```python
if flow['speed'] == 0.0:
    return dict(status='zero freestream force; rotor wash excluded',
                force_body_n=(0.0, 0.0, 0.0))
# For every nonzero speed the current function returns unavailable.
```

Consequently, the terminal message does not mean a coefficient lookup was attempted and failed. It summarizes a deliberately unimplemented force model. Current flow-regime descriptions are not a substitute for aerodynamic table bounds checks.

## 8. Why the default case needs special attention

All-default inputs prescribe stationary, level, north-facing aircraft and wind of 5 m/s FROM south (180°). Therefore:

```text
V_ground_NED = (0, 0, 0) m/s
V_wind_NED   ≈ (5, 0, 0) m/s
V_air_NED    ≈ (-5, 0, 0) m/s
V_air_wing   ≈ (-4.993147674, 0, -0.261679781) m/s
alpha_wing  = -177.0°
```

This is reverse flow relative to the reference wing. The tiny east component produced by floating-point trigonometry is display noise, not a meaningful crosswind.

The XML's ordinary lift-curve range is −40° to +40°; its large-negative-angle lead is −100° to 0°. Neither recorded range covers −177°. Recovering Figure 8 alone would therefore not establish a supported lift for the default case.

Do not change the AoA sign, wrap it into a small angle, clamp it, or extrapolate a pre-stall curve to make this case produce lift. Research either a genuinely applicable reverse-flow model or retain an unavailable result for this case. A separate prescribed forward-flight case can exercise an initially limited model.

For example, prescribing NED ground velocity `(50, 0, 0) m/s`, zero wind and level north-facing attitude yields airspeed 50 m/s and reference-wing AoA 3°. This is a useful kinematic setup for later coefficient testing; it does not itself validate any CL value.

## 9. What the scalar descriptors can and cannot support

The stored approximate slope and zero-lift angle suggest this candidate linear relationship, with alpha in degrees:

```text
CL_candidate = 0.085 * (alpha_deg + 2.5)
```

This is an interpretation of stored descriptors, not an implemented or validated XV-15 model. Research must establish whether the descriptors share a configuration, whether alpha is body- or wing-referenced, and what interval is appropriate before recommending its use.

The descriptors should not be treated as an exact nonlinear curve: substituting the recorded approximate stall angle of 14° gives `CL_candidate = 1.4025`, while recorded clean maximum CL is approximately 1.35. These approximate values do not define a unique consistent stall curve. Do not invent a stall blend, clip to CLmax, or infer a lower alpha bound from this arithmetic alone.

There are two reasonable research outcomes:

- Recover a supported tabulated model, with its source-defined configuration and domain.
- Establish a clearly labeled educational linear approximation with independently justified bounds and assumptions.

A limited model need not wait for every aircraft aerodynamic table. Conversely, a limited model must continue to reject operating conditions it cannot support.

## 10. Source-recovery map

An independent check of NASA's indexed PDF text found these concrete leads:

- CR-166536 directs readers to **Appendix B, B-31–B-55**. Appendix A, **A-47**, maps CL to **Tables 4-I/4-II**, CD to **4-III/4-IV**, and moment to **4-VIII**. At **A-50**, mast conversion is 0° helicopter/90° airplane; do not equate it numerically with the project's nacelle-incidence convention. At **A-70**, freestream pressure uses `0.5*rho*(U²+W²)`, with wake-excluded wing area and a separate aileron contribution. **B-51, Table 4-IX**, contains configuration-specific slopes that merit comparison with the XML's generic slope. This identifies definition checks, not a ready-to-import model. [NASA CR-166536](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166536_882.pdf).
- CR-166537, **p. 51 and Appendix A**, provides calculated trim/performance/stability reference outputs; its appendix was not verified as the coefficient-input dataset. [NASA CR-166537](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166537_883.pdf).

These checks used indexed text. Page images, PDF-viewer offsets, table units and complete numerical arrays remain unverified; confirm them before extraction.

The following figure and page references are copied from the repository. **They require checking against the actual document**, including printed versus PDF page numbering and exact figure labels. Their presence in XML is not proof that the extraction is accurate.

| Priority | Source location recorded in XML | Data or definition to recover |
|---|---|---|
| First | NASA CR-166536, Figure 8, printed p. 28 / PDF p. 32 | Wing/pylon CL curves; alpha definition; configuration labels; numerical curve values. |
| First | NASA CR-166536, Appendix B; local XML omits exact pages | Use the independent source check above to locate the actual arrays, recover their names and breakpoints, and map them to Figure 8. |
| First | NASA CR-166536, Appendix A, wing/pylon subsystem equations and definitions | Angle datum, local-flow definition, coefficient normalization, component scope and force transformations. |
| Next | NASA CR-166536, Figure 9, printed p. 29 / PDF p. 33 | Lift compressibility correction and how it modifies baseline lift. |
| Next | NASA CR-166536, Figure 10, printed p. 30 / PDF p. 34 | Large-negative-angle lift and supported configuration/domain. |
| Later | NASA CR-166536, Figure 11, printed p. 31 / PDF p. 35 | CD curves and drag definitions. |
| Later | NASA CR-166536, Figure 12, printed p. 32 / PDF p. 36 | Drag compressibility corrections. |
| Later | NASA CR-166536, Figure 13, printed p. 33 / PDF p. 37 | Large-angle drag. |
| Later | NASA CR-166536, Table 4-VIII; printed A-47, A-53, A-71 / PDF pp. 197, 203, 221 | Verify the XML's moment-model citation, symbols, values and reference point; several candidate pages are recorded, rather than a resolved citation. |
| Supporting | NASA TM-X-62407, section 3.3, printed p. 12; XML elsewhere records PDF p. 15 | Wing geometry, mounting incidence and area definitions. |
| Supporting | NASA CR-166537, exact coefficient/data-deck pages not recorded | Companion program documentation, table names, implemented interpolation, data decks and normalization. |

Source links recorded in `sources.xml`:

- S4: [NASA CR-166536, Generic Tilt-Rotor Simulation Math Model](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166536_882.pdf).
- S2: [NASA TM-X-62407, NASA/Army XV-15 Tilt Rotor Research Aircraft Familiarization Document](https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf).
- S8: NASA CR-166537, recorded locally as *Development and Validation of a Simulation for a Generic Tilt-Rotor Aircraft (GTRSIM) Program*. The XML supplies only the [NASA NTRS home page](https://ntrs.nasa.gov/); the independent check above supplies a direct NASA PDF. Reconcile registry metadata against its title page.
- S6, if researching local wake effects: [NTRS record 19750005744](https://ntrs.nasa.gov/citations/19750005744), recorded as work on rotor wake–airframe ground aerodynamic interference.

Useful document searches include `wing-pylon`, `CLWP`, `C_LWP`, `ALFWP`, `angle of attack`, `reference area`, `Appendix B`, `data deck`, and `Table 4-VIII`. These are search leads, not assertions that every token occurs in the source. OCR variants may require inspecting the scanned equations and figures directly.

The repository contains source-ID inconsistencies. For example, the structured aerodynamic block uses S4 for CR-166536, while an inline comment associates “S2” with CR-166536 even though `sources.xml` defines S2 as TM-X-62407. Other hover-download notes also vary in source attribution. Resolve by report title/number and original page evidence, rather than trusting an isolated source ID or comment.

## 11. Separate rotor, hover and dynamics gaps

The current wing-lift question should not become blocked by every missing tiltrotor subsystem. Keep the dependencies explicit:

| Capability | Additional missing data or implementation | Relation to limited freestream wing lift |
|---|---|---|
| Rotor-blade AoA and rotor thrust | Blade pitch distribution, section aerodynamics, local inflow/induced velocity and a rotor performance or blade-element model; suitable thrust/torque data | Separate model; wing AoA cannot replace blade AoA. |
| Wing loading in hover | Local rotor-wake velocity/distribution or a documented source-specific interference model | Required for rotor-wash loading, not for a declared freestream-only model. |
| Transition | Actual nacelle/control state and supported configuration/wake dependencies | Can initially be outside the valid domain. |
| Drag and pitching moment | Supported CD/moment models and their definitions | Can remain unavailable while supported scalar lift is calculated. |
| Time-varying motion | Force/moment totals, mass/inertia, equations of motion and integration | Prescribed-state lift can be calculated before dynamics exist. |
| Flight-by-wire | Control mappings, actuator/aircraft response and control laws | Separate development stage. |

`rotor.xml` records nonlinear blade twist of −45°, precone 2.5°, pitch/flap coupling −15°, flapping limits and RPM. These quantities do not supply a current blade angle of attack or a wing lift curve.

The reported 7% hover wing-download loss, 60-knot wake-interference observation and 30 ft/s ground-effect washout are separate observations with different contexts. They cannot be combined into a universal CL correction without recovering the original equations and applicability.

## 12. Requested analysis from ChatGPT

Use the following instructions with this report:

> I am developing TMPE, an educational XV-15 modeling prototype. Analyze the evidence in this report and locate exactly what data and definitions are missing for a supported wing-lift calculation. Inspect original primary sources where accessible. Do not assume you can access my local repository or other chats; this report is the supplied project context.
>
> First distinguish (a) values absent from my XML extraction, (b) data present but not loaded or implemented, (c) unknown source definitions/applicability, and (d) unsupported operating regimes. Do not assume absence from my extraction proves the original NASA source lacks the data.
>
> Inspect NASA CR-166536 Figure 8, its associated equations and definitions, and Appendix B. Follow the independently checked source leads above, verify them against page images, locate the complete numerical arrays, and establish how the tables map to the plotted configurations. Verify the identity and relevance of companion NASA CR-166537. Give direct document links, printed and PDF page numbers, figure/table/equation identifiers and exact variable names.
>
> Resolve the AoA datum, incidence handling, coefficient component scope, reference area, dynamic-pressure definition, nacelle/flap/flaperon settings and interpolation rules. Explain whether TMPE's calculated wing-chord AoA can be used directly or must be converted. Check for double counting of incidence or rotor-wake effects.
>
> Explain the minimum required inputs for a limited forward-flight lift-only model, and distinguish those from optional drag, pitching moment, rotor, transition and aircraft-dynamics extensions. If a restricted model is justified, specify its source-supported configuration and operating domain. Do not require a complete simulator merely to calculate supported scalar wing lift.
>
> Assess whether the stored approximate slope of 0.085/degree and zero-lift angle −2.5° can support a bounded educational approximation. Do not invent bounds or treat the approximate stall angle as an exact linear limit. Explain the status of the recorded CLmax values and whether they share the same configuration and angle convention.
>
> Explicitly address the default −177° reverse-flow case. Do not clamp, remap or extrapolate the physical AoA to force a result. Distinguish it from a supported forward-flight test case, and distinguish undefined AoA from unsupported coefficients and exactly zero freestream force.
>
> If extraction requires reading plots, label values as digitized estimates, state uncertainty, and keep source nodes separate from interpolated samples. If a PDF or table cannot be inspected, say what remains unverified and request that specific document/page; do not claim successful verification or invent coordinates.
>
> Finish with an evidence table, a prioritized recovery plan, and a concrete implementation specification for the next small TMPE change. Propose code edits only after the source and validity questions are resolved. Preserve existing coordinate conventions, full internal precision, honest unavailable statuses and the concise terminal interface.

## 13. Expected research deliverables

Request these outputs so the next conversation produces actionable evidence:

1. **Gap register:** for A1–A10, give current status, exact missing field/definition, original-source location, confidence, and the next action. Include newly discovered gaps separately.
2. **Source map:** list each verified report with a direct URL and distinguish original tables, source equations, digitized figures, and unverified repository notes.
3. **Coefficient specification:** identify supported component/configuration, source alpha convention, units, reference area, velocity definition, domain and interpolation method. Include only coefficients actually recovered or an explicitly justified analytic model.
4. **Extraction artifact:** if numerical tables are recovered, provide machine-readable source nodes with configuration columns and provenance. Mark digitized estimates and uncertainty. Leave unavailable values unavailable.
5. **Minimal implementation plan:** name the existing functions to extend, the new model/configuration inputs, error/status behavior and how scalar lift will be calculated without demanding CD/Cm unnecessarily.
6. **Verification cases:** establish source-derived coefficient checks and kinematic/domain cases. Do not invent “expected lift” numbers from an unsupported curve.

A proposed extraction schema, whose entries still need to be recovered, is:

```text
source_report, printed_page, pdf_page, figure_or_table, source_variable,
component_scope, alpha_definition, alpha_unit, alpha,
nacelle_deg, flap_deg, flaperon_deg, mach_condition, reynolds_condition,
coefficient_name, coefficient_value, reference_area, reference_area_unit,
velocity_definition, extraction_method, uncertainty, verification_status
```

Fields that are constant for a dataset can be stored as dataset metadata. Separate alpha nodes, configuration nodes and interpolation rules; do not imply a complete Cartesian grid when only particular curves exist.

## 14. Reproduction and acceptance checks

Run the current program from the repository root:

```bash
python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py'
python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --verbose
python3 -m unittest discover -s tests -v
```

The initial-state question accepts Enter for stationary, level, north-facing conditions. Answer `n` to prescribe the velocity and attitude individually. The normal report rounds display values; internal calculations retain floating-point precision.

The following current outcomes were reproduced while preparing this report:

| Prescribed case | Calculated airspeed | Calculated wing AoA | Current aerodynamic result |
|---|---:|---:|---|
| Default stationary state, 5 m/s wind FROM south | 5.00 m/s | −177.0° | Unavailable; reverse flow and no coefficient model. |
| Ground velocity north 50 m/s, level north-facing, calm | 50.00 m/s | 3.0° | Unavailable; no coefficient model. |
| Stationary, level, calm | 0.00 m/s | Unavailable | Zero modeled freestream force; rotor wash excluded. |
| Ground velocity east 10 m/s, level north-facing, calm | 10.00 m/s | Unavailable | Spanwise flow; aerodynamic model unavailable. |

For a future coefficient implementation, require agreement with verified source values within extraction tolerance, correct source-angle conversion, correct unit/reference-area handling, and clear rejection outside its supported domain. Retain undefined-AoA handling and the exact-zero freestream result. Preserve the distinction between tiny nonzero speed that rounds to `0.00 m/s` and true zero speed.

To let another ChatGPT conversation inspect code beyond these excerpts, also attach the current Python file, `aerodynamics.xml`, `geometry.xml`, `sources.xml`, and the two `review/re-interation/` XML files. Add `controls.xml`, `rotor.xml` and `wake_interference.xml` when investigating those extensions. This report alone supplies the main context and research questions; local file paths do not grant that conversation file access.
