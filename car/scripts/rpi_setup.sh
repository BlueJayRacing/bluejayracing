#!/bin/bash
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
pushd $SCRIPTPATH

sudo apt update -y
sudo apt upgrade -y
sudo apt autoremove -y
sudo apt install -y git emacs gpsd i2c-tools gpsd-clients pigpio libfmt-dev libgps-dev mosquitto mosquitto-clients 
git config --global --add --bool push.autoSetupRemote true
mkdir /home/pi/bin
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/95753fccb927e9730fac57f1039d5bb0af911321/install.sh | BINDIR=/home/pi/bin sh -s 0.34.0
sudo apt install libusb-dev -y
git clone https://github.com/PaulStoffregen/teensy_loader_cli.git /home/pi/teensy_loader_cli
wget https://www.pjrc.com/teensy/00-teensy.rules -P /home/pi/
sudo cp /home/pi/00-teensy.rules /etc/udev/rules.d/00-teensy.rules
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh ./get-docker.sh
sudo groupadd docker
sudo usermod -aG docker $USER
git submodule update --init --recursive
newgrp docker

sudo nmcli device wifi hotspot ssid bjr_wireless_axle_host password bluejayracing ifname wlan0
sudo nmcli connection modify $(nmcli connection show | grep "^Hotspot" | awk '{print $2}') connection.autoconnect yes connection.autoconnect-priority 100

popd



