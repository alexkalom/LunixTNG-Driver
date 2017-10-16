#!/bin/bash
rmmod lunix
insmod lunix.ko
./lunix_dev_nodes.sh
./lunix-attach /dev/ttyS0