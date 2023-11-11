#!/usr/bin/zsh

echo "check WSL IP address"
WSL_ADDRESS=`ipconfig.exe | grep -a IPv4 | cut -d: -f2 | grep -v 192.168 | tr -d ' '`
echo "WSL_ADDRESS = $WSL_ADDRESS"
