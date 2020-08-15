#!/usr/bin/env bash

set -e 


install_dir="$(pwd)/external"

sudo apt-get install -y wget libz-dev libboost-graph-dev

if ! [ -d ${install_dir} ]; then 
mkdir -p "${install_dir}"
fi 

cd /tmp
echo "Install dependencies: lemon to ${install_dir} ..."
rm -rf lemon-1.3.1 lemon-1.3.1.tar.gz
wget http://lemon.cs.elte.hu/pub/sources/lemon-1.3.1.tar.gz
tar -zxvf lemon-1.3.1.tar.gz
cd lemon-1.3.1
mkdir build && cd build 
cmake .. -DCMAKE_INSTALL_PREFIX=${install_dir}
make -j$(nproc)
make install 
cd /tmp
rm -rf lemon-1.3.1 lemon-1.3.1.tar.gz


rm -rf HdrHistogram_c
echo "Install dependencies: HdrHistogram_c to ${install_dir} ..."
git clone https://github.com/HdrHistogram/HdrHistogram_c.git
cd HdrHistogram_c
mkdir build && cd build 
cmake .. -DCMAKE_INSTALL_PREFIX=${install_dir}
make && make install 
cd /tmp 
rm -rf HdrHistogram_c

rm -rf Catch2
echo "Install dependencies: Catch2 to ${install_dir} ..."
git clone https://github.com/catchorg/Catch2.git
cd Catch2
mkdir build && cd build 
cmake .. -DCMAKE_INSTALL_PREFIX=${install_dir}
make && make install 
cd /tmp 
rm -rf Catch2

rm -rf json
echo "Install dependencies: json to ${install_dir} ..."
git clone https://github.com/nlohmann/json.git
cd json 
mkdir build && cd build 
cmake .. -DCMAKE_INSTALL_PREFIX=${install_dir} 
make && make install 
cd /tmp 
rm -rf json
