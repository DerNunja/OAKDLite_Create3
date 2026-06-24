#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

config_txt="/boot/firmware/config.txt"
cmdline_txt="/boot/firmware/cmdline.txt"
netplan_src="$repo_root/config/netplan/99-usb0.yaml"
netplan_dst="/etc/netplan/99-usb0.yaml"
cyclone_src="$repo_root/config/cyclonedds.xml"
cyclone_dst="$HOME/cyclonedds.xml"

require_file() {
  if [ ! -f "$1" ]; then
    printf 'Missing required file: %s\n' "$1" >&2
    exit 1
  fi
}

require_file "$config_txt"
require_file "$cmdline_txt"
require_file "$netplan_src"
require_file "$cyclone_src"

if [ ! -e "$config_txt.bak" ]; then
  sudo cp "$config_txt" "$config_txt.bak"
fi

if [ ! -e "$cmdline_txt.bak" ]; then
  sudo cp "$cmdline_txt" "$cmdline_txt.bak"
fi

if ! grep -qxF 'dtoverlay=dwc2,dr_mode=peripheral' "$config_txt"; then
  printf 'Adding USB gadget overlay to %s\n' "$config_txt"
  printf '\ndtoverlay=dwc2,dr_mode=peripheral\n' | sudo tee -a "$config_txt" >/dev/null
else
  printf 'USB gadget overlay already present in %s\n' "$config_txt"
fi

if ! grep -q 'modules-load=dwc2,g_ether' "$cmdline_txt"; then
  if grep -q 'rootwait' "$cmdline_txt"; then
    printf 'Adding dwc2/g_ether module load token to %s\n' "$cmdline_txt"
    sudo sed -i 's/rootwait/rootwait modules-load=dwc2,g_ether/' "$cmdline_txt"
  else
    printf "ERROR: 'rootwait' not found in %s. Edit manually; do not guess.\n" "$cmdline_txt" >&2
    exit 1
  fi
else
  printf 'dwc2/g_ether module load token already present in %s\n' "$cmdline_txt"
fi

cmdline_lines="$(wc -l < "$cmdline_txt")"
if [ "$cmdline_lines" -ne 1 ]; then
  printf 'ERROR: %s has %s lines; it must stay exactly one line.\n' "$cmdline_txt" "$cmdline_lines" >&2
  exit 1
fi

sudo install -m 600 "$netplan_src" "$netplan_dst"
sudo netplan generate

install -m 644 "$cyclone_src" "$cyclone_dst"

if ! grep -qxF 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' "$HOME/.bashrc"; then
  printf '\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n' >> "$HOME/.bashrc"
fi

if ! grep -qxF 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml' "$HOME/.bashrc"; then
  printf 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml\n' >> "$HOME/.bashrc"
fi

printf '\nInstalled configuration files:\n'
printf '  %s\n' "$netplan_dst"
printf '  %s\n' "$cyclone_dst"
printf '\nBoot files were updated if needed. Reboot the Pi before verifying usb0.\n'
