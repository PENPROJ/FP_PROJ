# Notification

SEDASOM 프로젝트와 연관된 프로젝트입니다.

https://github.com/SEOSUK/-On-going-SEDASOM

# Reference

This repository is from belows:

- https://github.com/gtfactslab/CrazySim
- https://github.com/llanesc/crazyflie-clients-python



# install guide

git clone https://github.com/PENPROJ/FP_PROJ --recursive

“이름 바꾸기, sitl_crazy”

cd sitl_crazy/crazyflie-clients-python/

pip install -e .

cd ..

cd CrazySim/crazyflie-lib-python/

pip install -e .

sudo apt install cmake build-essential

pip install Jinja2

cd

cd sitl_crazy/CrazySim/crazyflie-firmware/

mkdir -p sitl_make/build && cd $_

cmake ..
make all

sm

sw

sitl_gz

“터미널 하나 더 켜고”

sw

cfclient

“Enjoy Flight !!”
