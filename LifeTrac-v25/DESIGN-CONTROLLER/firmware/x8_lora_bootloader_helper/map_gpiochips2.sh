#!/bin/sh
for d in /sys/bus/gpio/devices/*; do
  base=$(basename $d)
  label=$(cat $d/label 2>/dev/null)
  ngpio=$(cat $d/ngpio 2>/dev/null)
  echo "$base : label='$label' ngpio=$ngpio"
done
