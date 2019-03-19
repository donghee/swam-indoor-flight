xhost +local:root

# TODO install to use gazebo
# apt-get install mesa-utils 
# TODO utility
# apt-get install terminator vim
# TODO ros-kinetic-desktop-full
# apt-get install ros-kinetic-desktop-full

docker run -it \
    --privileged \
    --publish 14556:14556/udp \
    --publish 8000:8000 \
    --publish 9090:9090 \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env=LOCAL_USER_ID="$(id -u)" \
    --volume="$HOME:/home/user" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name=swam-indoor-flight donghee/swam-indoor-flight:latest \
    /bin/bash -c "$1" "$2" "$3"

export containerId=$(docker ps -l -q)

# access to root
#docker exec -u 0 -it swam-indoor-flight bash
