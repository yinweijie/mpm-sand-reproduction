# Skills Registry

This file records the Codex skill triggers and the matching local commands for this repository.

## mpm-siggraph-2017-baseline

- Purpose: rebuild and verify the `pile_from_spout` regression baseline.
- Codex trigger:

  ```text
  Use $mpm-siggraph-2017-baseline to run scripts/regress_pile_from_spout_baseline.py and verify the tracked pile_from_spout baseline in this repo.
  ```

- Local commands:

  ```bash
  python3 scripts/regress_pile_from_spout_baseline.py
  python3 scripts/regress_pile_from_spout_baseline.py --check-only
  python3 scripts/regress_pile_from_spout_baseline.py --update-baseline
  ```

- Tracked baseline:
  - `outputs/baselines/pile_from_spout_16`
- Scratch run:
  - `outputs/.test/baseline_pile_from_spout_16`

## mpm-siggraph-2017-class-diagram

- Purpose: regenerate and verify the repository UML class diagram for the refactored solver, including ownership between `Simulation`, configuration structs, and internal helper types.
- Use when:
  - class ownership or module boundaries change during refactors
  - `docs/CLASS_DIAGRAM.md` needs to be refreshed after code changes
  - you want to confirm the checked-in `docs/CLASS_DIAGRAM.svg` still matches the source diagram
- Codex trigger:

  ```text
  Use $mpm-siggraph-2017-class-diagram to regenerate docs/CLASS_DIAGRAM.md and docs/CLASS_DIAGRAM.svg in this repo.
  ```

- Local commands:

  ```bash
  python3 scripts/render_class_diagram.py
  python3 scripts/render_class_diagram.py --check
  ```

- Tracked outputs:
  - `docs/CLASS_DIAGRAM.md`
  - `docs/CLASS_DIAGRAM.svg`

Add new skills here as the repository grows. Keep each entry focused on the exact trigger phrase and the repository-local command that goes with it.
