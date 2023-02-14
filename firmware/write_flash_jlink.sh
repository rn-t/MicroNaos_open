#!/usr/bin/zsh

WSL_ADDRESS=`ipconfig.exe | grep IPv4 | cut -d: -f2 | grep -v 192.168 | tr -d ' '`

JLinkExe ip $WSL_ADDRESS -device STM32F405RG -if SWD -speed 4000 -autoconnect 1 -CommanderScript write_flash.jlink