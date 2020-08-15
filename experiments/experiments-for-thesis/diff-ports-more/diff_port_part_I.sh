#!/usr/bin/env bash

set -e


# for n in 8 16 32 128 256
# do
#   echo "./ssched_simulator ${exp_config_prefix}${n}.json | tee diff-ports-${n}.txt"
#   ./ssched_simulator ${exp_config_prefix}${n}.json | tee diff-ports-${n}.txt
# done
function my_experiment()
{
  exp_config_prefix="../experiments/experiments-for-thesis/diff-ports-more/delay-vs-load-port"
  echo "./ssched_simulator ${exp_config_prefix}$1.json | tee diff-ports-$1.txt"
  ./ssched_simulator ${exp_config_prefix}$1.json | tee diff-ports-$1.txt
}

export -f my_experiment
parallel -j4 my_experiment ::: 8 16 32 128 256


