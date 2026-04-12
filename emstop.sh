#!/bin/sh
kill -9 `pgrep -f "/home/mito/Wi-SUN_EnergyMeter/sem_app/bin/www"`
kill -9 `pgrep -f "/home/mito/Wi-SUN_EnergyMeter/sem_com.py"`
/usr/bin/sudo -f /usr/bin/rm /tmp/sem.sock
/usr/bin/sudo -f /usr/bin/rm /tmp/curr_pow.txt

