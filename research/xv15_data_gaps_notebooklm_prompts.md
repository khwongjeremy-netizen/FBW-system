# XV-15 Data Gaps NotebookLM Prompts

Use the selected source register and the source comments to ground each prompt in the original report. Keep the task source-specific and do not assume a value is missing when the source may simply describe it qualitatively.

## Source register context
- S1: XV-15 Tiltrotor (author: NASA, date: 2003-09-11)
- S2: NASA/Army XV-15 Tilt Rotor Research Aircraft Familiarization Document (author: NASA, date: 1975-01, report: NASA TM-X-62407)

## 1) Intermediate nacelle-angle inertia data
Research objective: Determine whether the original XV-15 source contains three-axis mass moments of inertia at intermediate nacelle angles, or whether the report only provides airplane and helicopter mode values.

Required output: exact page numbers, section names, equations or tables, units, operating condition, and the statement of whether the report gives those values or not.
What to extract: observed engine/rotor mode, nacelle angle, axis names (roll/pitch/yaw), inertia values, and any notes about interpolation or absence.

## 2) Nonlinear aerodynamic coefficient curves
Research objective: Identify whether the source provides nonlinear aerodynamic coefficient curves for CL, CD, and Cm over angle of attack and relevant flight configuration.

Required output: exact reference to any figure, table, or section; values or relationships; operating conditions; units; and any limitations or assumptions.
What to extract: coefficient definitions, reference area/chord conventions, and whether the source provides complete curves or only a few points.

## 3) Rotor-wake flowfield data during conversion
Research objective: Clarify whether the source contains time-dependent or numerically tabulated rotor-wake flowfield information during nacelle conversion.

Required output: page numbers, figure/table numbers, section names, and a statement explaining whether the source contains detailed wake-flow data or not.
What to extract: wake model type, flowfield data format, conversion condition, and any missing context or notes.

If the source does not contain the information, mark the result as absent and explain why that limits the model.
