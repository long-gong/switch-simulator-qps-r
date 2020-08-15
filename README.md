# (INPUT-QUEUED) SWITCH SIMULATOR for QPS-r

[![Build Status](https://travis-ci.org/long-gong/switch-simulator-qps-r.svg?branch=master)](https://travis-ci.org/long-gong/switch-simulator-qps-r)

Simulation codes for our switching paper:

Long Gong, Jun (Jim) Xu, Liang Liu, and Siva Theja Maguluri. 2020. QPS-r: A Cost-Effective Iterative Switching Algorithm for Input-Queued Switches. In Proceedings of the 13th EAI International Conference on Performance Evaluation Methodologies and Tools (VALUETOOLS ’20). Association for Computing Machinery, New York, NY, USA, 19–26. DOI:https://doi.org/10.1145/3388831.3388836

## Platforms 

This project supports the following platforms

  * Linux
  * Max OS X

## Dependencies 

  * [HdrHistogram_c](https://github.com/HdrHistogram/HdrHistogram_c.git)
  * [json](https://github.com/nlohmann/json.git)
  * [boost-graph](https://github.com/boostorg/graph)
  * [Catch2](https://github.com/catchorg/Catch2)

Dependencies can be installed by 
```bash
chmod +x ./install_dependencies.sh
./install_dependencies.sh
```


## Build 

```bash
mkdir build
cd build 
cmake ..
make 
```

## Run

```bash
./switch_simulator <config-file.json>
```

`<config-file.json>` is a configuration file in `JSON` format. Examples of configurations can be found in [experiments](./experiments)
