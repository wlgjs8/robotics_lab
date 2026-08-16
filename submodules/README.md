# Git Submodules

This directory contains the external source checkouts declared in the root
`.gitmodules` file. Each checkout is pinned by a Git link in `robotics_lab`.

## Active Submodules

| Path | Upstream | Role |
| --- | --- | --- |
| `Fast-FoundationStereo/` | `https://github.com/wlgjs8/Fast-FoundationStereo.git` | Stereo perception dependency |
| `controller-manager/` | `https://github.com/PLAIF-dev/controller-manager` | Company controller stack consumed through `cm_bridge` |

Initialize all pinned checkouts with:

```bash
git submodule update --init --recursive
```

## Controller Manager Update Policy

Treat `controller-manager/` as read-only inside this repository. Do not create
commits in the submodule checkout. Submit fixes upstream, then update the pinned
revision only after the bridge compatibility gate passes:

```bash
git -C submodules/controller-manager fetch origin
git -C submodules/controller-manager checkout <reviewed-sha>
make cm-sils-gate
git add submodules/controller-manager
git commit -m "chore: bump controller-manager to <reviewed-sha>"
```

See `cm_bridge/docs/design.md` for the process-boundary integration contract and
the SILS acceptance requirements.
