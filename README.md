# Swam-indoor-flight

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

### Build Groundstation on docker

```
cd ~/src/mission
yarn install
```

----

## Swam node

```
mkdir -p ~/catkin_ws/src
cp -Rf swam ~/catkin_ws/src/
```

### Build swam node on docker

```
cd ~/catkin_ws
catkin init
wstool init src
catkin build
source devel/setup.bash
```

----

## Run groundstation and swam node on docker

```
tmuxinator new simulator.yaml
cp ~/src/swam-indoor-flight/simulator.yaml ~/.tmuxinator/
```

```
~/src/swam-indoor-flight/scripts/run-simulator.sh
```

OR

```
~/src/swam-indoor-flight/scripts/run-server.sh
```
