#!/bin/sh
# Run from the project root:  sh sim/run.sh
set -e
mkdir -p sim/out
iverilog -g2012 -s tb_pcie6_link -o sim/out/sim.vvp rtl/*.v tb/tb_pcie6_link.v
vvp sim/out/sim.vvp | tee sim/out/sim.log
