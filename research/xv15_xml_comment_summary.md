# XV-15 XML Comment Audit Summary

This summary records the original XML comments, their likely target, and the recommended follow-up. The source text is preserved verbatim where it was written inline, and the interpretation is kept separate from the original comment wording.

## Catalog overview
- Total comments catalogued: 18
- Files represented: data/aircraft.xml, data/controls.xml, data/control_response.xml, data/engine.xml, data/geometry.xml, data/review/re-interation/data_gaps.xml, data/review/re-interation/review_flags.xml, data/systems.xml
- Research-priority categories: data gap, research request, unresolved question

## By file

### data/aircraft.xml
- C01 | line 53 | category: unresolved question | status: research required
  - Original: What is 'comparison's' role in this variable? and what is the valuable fo this variable specifically?
  - Target: Confirm the exact meaning and reference condition of the comparison value.
  - Action: Trace the source text for the comparison and define the operating condition.
- C02 | line 58 | category: unresolved question | status: research required
  - Original: What is fuel efficiency threshold?
  - Target: Define what the threshold represents and whether it is a distance, mission metric, or fuel-burn criterion.
  - Action: Locate the source sentence or figure that explains the threshold.

### data/controls.xml
- C03 | line 42 | category: research request | status: research required
  - Original: - NotebookLM describes these as limits during conversion; confirm the exact sign convention in the figure. Not sure on that one I. beleive all are correct at the moment
  - Target: Verify whether the listed limits use positive/negative deflection convention or upward/downward control-surface direction.
  - Action: Check the original figure before making any sign-convention change.

### data/control_response.xml
- C04 | line 12 | category: research request | status: research required
  - Original: what is SCAS? and explain this variable more thoroughly, including other tags with commented #
  - Target: Define SCAS and the meaning of control sensitivity and damping values.
  - Action: Map SCAS to the source terminology and explain the metrics in the XML comments.

### data/engine.xml
- C05 | line 47 | category: research request | status: unresolved
  - Original: - Rating names, durations, and values are transcribed from the NotebookLM extraction; check the report table before relying on them.
  - Target: Validate the exact engine power rating labels, durations, and operating conditions.
  - Action: Check the original report table before using these values in a model.
- C06 | line 50 | category: unresolved question | status: research required
  - Original: Why is power_rating the name for these variables? what does that mean and what do these units mean? are they operating parameters of the engines?
  - Target: Define the power-rating terminology and the meaning of SHP in the source context.
  - Action: Trace the report's engine section and document this terminology explicitly.

### data/geometry.xml
- C07 | line 98 | category: research request | status: research required
  - Original: where does this show up? where is the source of this?
  - Target: Locate the original source for the horizontal-tail incidence adjustment range.
  - Action: Match the value to the source page and note whether it is a flight or ground range.
- C08 | line 103 | category: research request | status: research required
  - Original: where does this show up? where is the source of this?
  - Target: Confirm the original source for the NACA 64A015 profile value.
  - Action: Verify the exact source page and whether the airfoil is a section or design profile.
- C09 | line 108 | category: reflective observation | status: research required
  - Original: this is actually under the 'tail length'
  - Target: Confirm that the value is a tail-length lever-arm measurement rather than an independent aerodynamic dimension.
  - Action: Check the source figure or section naming.
- C10 | line 132 | category: unresolved question | status: research required
  - Original: unclear as of what this is specifically refering to.
  - Target: Define tip chord for the vertical tail and whether it is the outboard edge chord or another planform chord.
  - Action: Cross-check the geometry section for the exact meaning.
- C11 | line 136 | category: unresolved question | status: research required
  - Original: unclear as of what this is specifically refering to.
  - Target: Define the mean aerodynamic chord used for the vertical tail and whether it is planform-based.
  - Action: Check the source formula or planform description for the fin.

### data/review/re-interation/data_gaps.xml
- C16 | line 9 | category: data gap | status: research priority
  - Original: Three-axis mass moments of inertia at intermediate nacelle angles were not identified; extracted values are for airplane and helicopter modes only.
  - Target: Research whether the source contains intermediate-angle inertia data or confirms it is absent.
  - Action: Prompt NotebookLM for exact pages/figures or a statement that the data is absent.
- C17 | line 12 | category: data gap | status: research priority
  - Original: Full nonlinear aerodynamic coefficient curves, including CL, CD, and Cm across angle of attack, were not identified.
  - Target: Determine whether nonlinear aerodynamic data exists in the source or must remain unknown.
  - Action: Ask NotebookLM to identify whether the source provides these curves or only qualitative descriptions.
- C18 | line 15 | category: data gap | status: research priority
  - Original: Numerical unsteady rotor-wake flowfield tables during conversion were not identified.
  - Target: Identify whether the source contains conversion-phase wake-flow tables or only qualitative descriptions.
  - Action: Use a NotebookLM prompt to search for conversion-intermediate wake-flow data.

### data/review/re-interation/review_flags.xml
- C12 | line 20 | category: proposed correction | status: ready for implementation
  - Original: Use this value for the simulation unless the source page defines useful load differently.
  - Target: Keep the S1/S2 useful-load discrepancy visible while using S2 as the simulation default pending source validation.
  - Action: Do not collapse the discrepancy until the source is checked.
- C13 | line 38 | category: proposed correction | status: ready for implementation
  - Original: This is the value to use in the model because it matches the report's sweep definition.
  - Target: Use S2 quarter-chord sweep value pending sign-convention confirmation.
  - Action: Keep the sign convention explicit in the model documentation.
- C14 | line 24 | category: implementation instruction | status: ready for implementation
  - Original: I like actions like these so the AI who created this should re-iterate more of the above
  - Target: Keep review actions explicit, evidence-linked, and source-traceable.
  - Action: Maintain action language that explains what to verify and why.

### data/systems.xml
- C15 | line 33 | category: reflective observation | status: unresolved
  - Original: when the rotors are out of position and therefore they create forces that feed into the natural bouncing forces of the aircraft body creating a cascading force of damage.
  - Target: Treat as a qualitative note, not verified physics, unless the source explicitly supports it.
  - Action: Preserve as a note and do not convert it into design data without source support.

## Notes
- This is an audit of comments and unresolved questions, not a model rewrite.
- Original source values remain preserved; no value was invented here.
- The unresolved inventory items remain intentionally flagged for source verification rather than model completion.
