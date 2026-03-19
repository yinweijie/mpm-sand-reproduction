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

Add new skills here as the repository grows. Keep each entry focused on the exact trigger phrase and the repository-local command that goes with it.
