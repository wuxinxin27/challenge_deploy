#!/usr/bin/env bash
# Local two-follower setup; enables communication, sends no arm commands.
set -euo pipefail

if (( EUID != 0 )); then
    exec sudo bash "$(readlink -f -- "$0")" "$@"
fi

command -v ip >/dev/null || { echo 'Missing ip command. Please install iproute2.' >&2; exit 1; }

check_adapter() {
    local iface="$1" expected="$2" side="$3" path serial=''
    if [[ ! -e "/sys/class/net/$iface/device" ]]; then
        echo "$iface not found. Please connect the $side arm CAN adapter." >&2
        exit 1
    fi
    path=$(readlink -f "/sys/class/net/$iface/device")
    while [[ "$path" != / ]]; do
        if [[ -r "$path/serial" ]]; then
            serial=$(<"$path/serial")
            break
        fi
        path=${path%/*}
    done
    if [[ "$serial" != "$expected" ]]; then
        echo "$iface adapter mismatch: found '${serial:-unknown}', expected $expected for the $side arm." >&2
        echo 'Check interface names against configs/local_two_followers.yaml. CAN state has not been changed.' >&2
        exit 1
    fi
    echo "$iface = $side arm ($serial)"
}

# Validate both identities before changing either interface.
check_adapter can0 004A00244648571220363032 left
check_adapter can1 004E00324648571220363032 right

for iface in can0 can1; do
    ip link set dev "$iface" down
    ip link set dev "$iface" type can bitrate 1000000
    ip link set dev "$iface" up
    ip -details link show dev "$iface"
done
echo 'can0 / can1 are UP at 1000000 bit/s. You can now rerun capture.'
