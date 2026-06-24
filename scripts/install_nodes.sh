#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
install_root="${CREATE3_OAK_HOME:-$HOME/create3_oak}"
nodes_dst="$install_root/nodes"
venv_dst="$install_root/venv"

mkdir -p "$nodes_dst"
install -m 755 "$repo_root/create3_oak/nodes/oak_publisher.py" "$nodes_dst/oak_publisher.py"
install -m 755 "$repo_root/create3_oak/nodes/drive_to_object.py" "$nodes_dst/drive_to_object.py"

if command -v uv >/dev/null 2>&1; then
  uv venv --system-site-packages "$venv_dst"
  # shellcheck disable=SC1091
  source "$venv_dst/bin/activate"
  uv pip install -r "$repo_root/pyproject.toml"
else
  python3 -m venv --system-site-packages "$venv_dst"
  # shellcheck disable=SC1091
  source "$venv_dst/bin/activate"
  python3 -m pip install "depthai==3.7.1"
fi

printf 'Installed nodes to %s\n' "$nodes_dst"
printf 'Installed camera virtual environment to %s\n' "$venv_dst"
