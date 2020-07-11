#!/bin/bash

set -o nounset

function die()
{
    echo >&2 "FAILURE: $*"
    exit 1
}

(sudo modprobe can && sudo modprobe can_raw && sudo modprobe vcan) || die "Cannot setup kernel modules"

iface="vcan0"
sudo ip link add dev $iface type vcan
sudo ip link set $iface mtu 72        || die "Could not configure MTU for CAN FD on $iface"
sudo ip link set up $iface            || die "Could not bring up $iface"
