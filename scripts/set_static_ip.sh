#!/bin/bash
nmcli con mod "Wired connection 1" ipv4.addresses "192.168.84.100/24"
nmcli con mod "Wired connection 1" ipv4.gateway "192.168.84.150"
nmcli con mod "Wired connection 1" ipv4.dns "192.168.84.150"
nmcli con mod "Wired connection 1" ipv4.method manual
nmcli con mod "Wired connection 1" connection.autoconnect yes