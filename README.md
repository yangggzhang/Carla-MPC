# Carla-MPC
Download stable Carla
```
./setup.sh
```

Download useful reference
```
git submodule update --init --recursive
```

Run Carla and MPC
```
./CarlaSimulator/CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server -benchmark -quality-level=Low -fps=30

cd src
python3 main.py
```
