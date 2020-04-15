#!/bin/bash
# Install dependencies. 
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 304F9BC29914A77D
sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-0.9.8/ all main"
sudo apt-get update
sudo apt-get install carla
echo "alias carla='/opt/carla/bin/CarlaUE4.sh'" >> ~/.bashrc
source ~/.bashrc
