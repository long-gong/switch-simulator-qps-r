#!/usr/bin/env bash

set -e

exp_config_prefix="../experiments/experiments-for-thesis/diff-ports/delay-vs-load-port"
for n in 8 16 32 128
do
  echo "./ssched_simulator ${exp_config_prefix}${n}.json | tee diff-ports-${n}.txt"
  ./ssched_simulator ${exp_config_prefix}${n}.json | tee diff-ports-${n}.txt
done


