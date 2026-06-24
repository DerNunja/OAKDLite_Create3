# Raspberry Pi Configuration Files

These files document the runtime configuration used on the Raspberry Pi.

## Files

- `cyclonedds.xml`: Cyclone DDS configuration for ROS 2 communication over `usb0` and `wlan0`.
- `netplan/99-usb0.yaml`: static USB network address for the Pi side of the Create 3 USB link.
- `boot/config.txt.append`: line that must be present in `/boot/firmware/config.txt` to enable USB gadget mode.
- `boot/cmdline-token.txt`: token that must be inserted after `rootwait` in `/boot/firmware/cmdline.txt`.

## IP Addresses

- `192.168.186.2`: Create 3 USB interface
- `192.168.186.3`: Raspberry Pi `usb0` interface

## Install

Run from the repository root on the Pi:

```bash
scripts/install_pi_config.sh
sudo reboot
```

After reboot, verify:

```bash
ip addr show usb0
ping -c 3 192.168.186.2
```
