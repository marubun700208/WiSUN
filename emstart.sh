#!/bin/sh
/usr/bin/sudo /usr/bin/rm /tmp/sem.sock
/usr/bin/sudo /usr/bin/rm /tmp/curr_pow.txt
/home/mito/Wi-SUN_EnergyMeter/sem_app/bin/www &
/home/mito/Wi-SUN_EnergyMeter/sem_com.py --delay 60
