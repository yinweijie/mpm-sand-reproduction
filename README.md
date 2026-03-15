# MPM Sand Reproduction

This repository contains an in-progress C++ reproduction of the sand model from:

- Klár et al. 2016, `Drucker-Prager Elastoplasticity for Sand Animation`

Current scope:

- `3D single-species dry sand`
- `explicit APIC/MPM`
- `Hencky/log-strain elasticity`
- `Drucker-Prager plastic projection`
- `hardening`
- scene configuration, particle export, preview rendering, and Blender-based final rendering

Current non-goals:

- full 2016 supplementary alignment
- implicit solve from `Algorithm 2`
- 2017 water-sand coupling

## Quick Start

Build and run a scene:

```bash
bash ./build.sh release scenes/pile_lab.json
```

Export simulation frames:

```bash
./build/Release/klar2016_sand \
  --scene scenes/pile_lab.json \
  --steps 2400 \
  --export \
  --output-dir outputs/pile_lab_case
```

Render with Blender:

```bash
export BLENDER_BIN=/home/ywj22/blender-5.0.1-linux-x64/blender

bash ./scripts/render_pile_lab_blender.sh \
  outputs/pile_lab_case \
  outputs/pile_lab_case/pile_lab.mp4 \
  scenes/pile_lab.json
```

## Documentation

- [Docs Index](docs/README.md)
- [Current Status](docs/CURRENT_STATUS.md)
- [Roadmap](docs/ROADMAP.md)
- [References](references/README.md)
- [AI Disclosure](AI_DISCLOSURE.md)

## Current Example Output

- [Paper-like pile render](outputs/pile_lab_dense_check_v2/pile_lab_paper_like_v1.mp4)
- [Last frame](outputs/pile_lab_dense_check_v2/pile_lab_paper_like_v1_last.png)
