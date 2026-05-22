# Vendored Upstream Sources

This directory contains source snapshots from company repositories that
`robotics_lab` intends to use directly.

These folders are vendored source trees, not Git submodules. Their internal
`.git` directories are intentionally omitted so a GitHub "Download ZIP" of
`robotics_lab` includes the source files.

## Sources

| Path | Upstream | Branch | Imported commit |
| --- | --- | --- | --- |
| `mo_forcecontroller/` | `https://github.com/PLAIF-dev/mo_forcecontroller` | `main` | `76c29303cd8bf261b26008ee8ad812d12d654cb0` |
| `mo_grippers/` | `https://github.com/PLAIF-dev/mo_grippers` | `main` | `f7e5197a5464027d6fe2688c1656963aa14bfed7` |

## Refresh

Until the root `robotics_lab` repository is initialized, refresh by cloning to a
temporary directory and copying files while excluding `.git`.

After `robotics_lab` becomes a normal root Git repository, prefer Git subtree:

```bash
git remote add mo_forcecontroller https://github.com/PLAIF-dev/mo_forcecontroller
git remote add mo_grippers https://github.com/PLAIF-dev/mo_grippers

git subtree pull --prefix=submodules/mo_forcecontroller mo_forcecontroller main --squash
git subtree pull --prefix=submodules/mo_grippers mo_grippers main --squash
```

Use subtree rather than Git submodule when ZIP export and AI review bundles need
to include the full source.
