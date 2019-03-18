# Swam-indoor-flight


![Timeline Editor](screenshot-editor.png?raw=true "Timeline Editor")
![Live Viewer](screenshot-live.png?raw=true "Live Viewer")


```
mkdir ~/src
cd ~/src
git clone https://github.com/donghee/swam-indoor-flight 
```

## Groundstation

```
cd ~/src
git clone https://github.com/doojinkang/mission 
```

## Swam node

```
cd ~/src/swam-indoor-flight 
mkdir -p ~/catkin_ws/src
cp -Rf swam ~/catkin_ws/src/
```

## Run groundstation and swam node on docker

### Run 

Docker Image: https://hub.docker.com/r/donghee/swam-indoor-flight

```
cd ~/src/swam-indoor-flight/scripts
./run-server-with-simulator.sh
```

OR

```
cd ~/src/swam-indoor-flight/scripts
./run-server.sh
```

----

### First Run to make environments

Execute on *Docker*

#### Add tmuxinator configures 

```
mkdir ~/.tmuxinator

tmuxinator new simulator
tmuxinator new server

cp ~/src/swam-indoor-flight/simulator.yml ~/.tmuxinator/
cp ~/src/swam-indoor-flight/server.yml ~/.tmuxinator/
```

#### Build Groundstation on docker

```
cd ~/src/mission
yarn install
```

#### Build swam node on docker

```
cd ~/catkin_ws
catkin init
wstool init src
catkin build
source devel/setup.bash
```

#### Build PX4 sitl gazebo on docker

```
git clone https://github.com/PX4/Firmware
cd Firmware
make px4_sitl_default
make px4_sitl_default sitl_gazebo
```

