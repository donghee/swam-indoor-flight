# Swam-indoor-flight

```
mkdir ~/project_swam
mkdir -p ~/catkin_ws/src
```
# Clone Groundstation
```
git clone https://github.com/doojin.kang/mission 
mv mission ~/project_swam
```
# Swam node
```
cp -Rf swam ~/catkin_ws/src/
```
## Build swam node On docker
```
cd ~/catkin_ws
catkin init
wstool init src
source devel/setup.bash
```
# Run groundstation and mavros node on docker image
```
cp -Rf scripts ~/project_swam
```
````
~/project_swam/scripts/run-simulator.sh
```
or
```
~/project_swam/scripts/run-server.sh
```
