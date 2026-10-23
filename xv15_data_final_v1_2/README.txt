XV-15 DATASET v1.2
==================

Purpose
-------
Source-traceable XML dataset for an educational XV-15 tiltrotor simulation.
This is a research dataset, not a validated flight model or real aircraft
flight-control system.

Version 1.2 update
------------------
- Added the latest inertia research audit to inertia.xml.
- Retained the reported 0-degree and 90-degree endpoint inertias.
- Explicitly records that intermediate-angle inertias and a source-documented
  transformation method were not found in the reviewed material.
- Records the NASA CR-166536 warning that the historical XV-15 inertia values
  were reportedly outdated due to omitted airframe modifications and flight-test
  instrumentation mass/locations.
- Records the NASA TM-89428 flight-test identification finding as a research
  lead requiring exact page/table verification before quantitative use.
- Added missing input requirements, source leads, and review flags.
- Component group weights are not treated as sufficient to calculate component
  inertia tensors or centres of mass.

Core data-integrity rules
-------------------------
1. Unknown values remain unknown; no intermediate inertia interpolation is used.
2. The endpoint inertias are source-reported historical values for the 13,000 lb
   gross-weight condition, not asserted here to be validated flight-test values.
3. The supplied NotebookLM audit reports the endpoint table as verified in the
   original NASA TM-X-62407. The original PDF was not independently re-opened
   during this packaging step.
4. References suggested by the audit (Lanham 1979, NASA CR-166537, Bell Report
   301-199-002, and NASA TM-89428) have not all been independently accessed in
   this step; their status is documented in sources.xml and review flags.
5. Existing aerodynamic and wake entries retain their prior evidence cautions.
6. Plotted curve metadata is not a numeric lookup table.
7. XV-15 values must not be presented as CV-22 specifications.
8. This package does not reproduce actual aircraft flight-control software.

Main files
----------
data/inertia.xml                         Endpoint inertias and method/gap status
data/mass.xml                            Weight groups and mass-property readiness
data/sources.xml                         Source register and access caveats
data/review/re-interation/data_gaps.xml  Unresolved parameter/source gaps
data/review/re-interation/review_flags.xml Review and verification actions
