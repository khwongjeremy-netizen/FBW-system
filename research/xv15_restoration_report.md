# XV-15 Restoration Report

## Verified repository baseline
The repository was inspected with git status and git diff before restoration. The only project files changed in the recent erroneous patch were:
- data/engine.xml
- data/geometry.xml
- data/review/re-interation/review_flags.xml

These three files were restored to the repository baseline with `git restore -- ...`, leaving unrelated work untouched.

## What was preserved
- Original XML values, tags, attributes, and ordering.
- Existing user comments and reflective notes.
- Source IDs, page references, units, conditions, and unresolved verification flags.
- Unrelated files, helper files, and repository additions outside the restored set.

## Verification
The repository was checked after restoration and the affected XML files were confirmed to be at the verified baseline state. The XML was also re-validated as parseable/valid after restoration.

## Result
This was a preservation-first restoration. No original source data was overwritten, and no broad destructive git commands were used.
