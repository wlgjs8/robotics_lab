#!/usr/bin/env bash
# Re-apply the robotics_lab viser client patches and rebuild the client.
#
# Patches (in viser_patches/, copied into the installed viser client's
# src/components/):
#   * NumberInput.tsx — Enter-to-commit, focus-aware mirror. Upstream viser
#     streams every keystroke to the server, so typing a multi-digit TCP target
#     jogs the robot through each intermediate value (and a live state-mirror
#     fights the operator's typing). This makes Enter the sole commit path.
#   * Folder.tsx — a `⟦cols=N⟧` label marker lays a folder's children out in an
#     N-column grid (used to show the left/right arm TCP PTP blocks side by side).
#
# Run this after any `pip install`/upgrade of viser (the edits live in the
# installed package's client source, which a reinstall overwrites).
#
# Usage:
#   rb_gui/scripts/patch_viser_number_enter.sh
#
# Requires node + npm on PATH (the same toolchain viser uses to build its client).
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
patch_dir="${here}/viser_patches"

viser_client_dir="$(python3 - <<'PY'
import os, viser
print(os.path.join(os.path.dirname(viser.__file__), "client"))
PY
)"

if [[ ! -d "${patch_dir}" ]]; then
  echo "ERROR: patch dir not found: ${patch_dir}" >&2
  exit 1
fi
if [[ ! -d "${viser_client_dir}/src" ]]; then
  echo "ERROR: viser client source not found at ${viser_client_dir}/src" >&2
  echo "       (a non-source viser wheel cannot be rebuilt; reinstall from sdist)" >&2
  exit 1
fi

echo "viser client: ${viser_client_dir}"
for patched_src in "${patch_dir}"/*.tsx; do
  name="$(basename "${patched_src}")"
  echo "applying patched ${name} ..."
  cp "${patched_src}" "${viser_client_dir}/src/components/${name}"
done

if [[ ! -d "${viser_client_dir}/node_modules" ]]; then
  echo "installing client deps (npm install) ..."
  (cd "${viser_client_dir}" && npm install)
fi

echo "rebuilding client (npm run build) ..."
(cd "${viser_client_dir}" && npm run build)

echo "done. Restart rb_gui (and hard-refresh the browser) to pick up the new client."
