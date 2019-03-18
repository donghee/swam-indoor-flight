# Install

1. Update odroid recent kernel to use usb wifi stick
2. Disable network manager since wifi stability
3. Install ros kinetic full desktop
3. Copy tmux configuration
4. Copy tmuxinator configuration
5. Copy bin /home/odroid/
6. Copy rc.local

# Monitor 

terminator -l odroid

# To use remote slave node

.bashrc

machine_ip=(`hostname -I`)
export ROS_IP=${machine_ip[0]}
