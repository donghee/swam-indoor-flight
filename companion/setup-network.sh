#!/bin/sh

# make static ip
echo "nameserver 8.8.8.8" | tee -a /etc/resolvconf/resolv.conf.d/base

# TODO Change static ip which you want!

cp interfaces /etc/network/
