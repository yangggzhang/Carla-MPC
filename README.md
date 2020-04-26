# Carla-MPC
Download and unzip carla 0.9.6 along with Town06 map (https://github.com/carla-simulator/carla/releases/tag/0.9.6) into CarlaSimulator folder

Download useful reference
```
git submodule update --init --recursive
```

Update root
```
export CARLA_ROOT = 
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla-<VERSION>.egg
```

Run Carla and MPC
```
./CarlaSimulator/CarlaUE4.sh
cd src
python3 main.py
```
