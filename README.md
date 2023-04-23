# SeismoHome
## Simple MPU-6050 based earthquake detector with alarm, data logging and web interface

<div style="width:100%;text-align:center;">
    <p align="center">
        <a href="https://www.youtube.com/@F3RNI"><img alt="YouTube" src="https://img.shields.io/badge/-YouTube-red" ></a>
        <a href="https://f3rni.bandcamp.com"><img alt="Bandcamp" src="https://img.shields.io/badge/-Bandcamp-cyan" ></a>
        <a href="https://soundcloud.com/f3rni"><img alt="SoundCloud" src="https://img.shields.io/badge/-SoundCloud-orange" ></a>
    </p>
</div>
<div style="width:100%;text-align:center;">
    <p align="center">
        <img src="Screenshot_1.png">
    </p>
</div>

## README IN PROGRESS...

### PART 1. Preparation for low-performance systems

Install required packets
```shell
sudo apt-get update
sudo apt-get install git, python3, python3-pip, cmake
sudo apt-get install python3-dev python3-pip gfortran libblas-dev liblapack-dev libatlas-base-dev
sudo apt-get install libopenblas-dev pkg-config
```

Increase size of /tmp
```shell
df -h /tmp
sudo mount -o remount,size=2G /tmp/
```

Edit swap config to increase swap size
```shell
sudo apt-get install dphys-swapfile
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
```

Uncomment and set values of `CONF_SWAPSIZE` and `CONF_MAXSWAP` to `2048`
```shell
CONF_SWAPSIZE=2048
CONF_MAXSWAP=2048
```

Apply changes
```shell
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

### PART 2. Repo cloning and installing libraries

Clone repo and install requirements
```shell
git clone https://github.com/F33RNI/SeismoHome
cd SeismoHome
sudo pip3 install -r requirements.txt
```

### PART 3


