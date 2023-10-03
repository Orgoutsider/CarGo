#! /bin/zsh

source /opt/ros/melodic/setup.zsh
source /home/nano/car/car/car_ws/devel/setup.zsh
source /home/nano/catkin_workspace/install/setup.zsh  --extend
export PATH=/home/nano/.local/bin:$PATH
source /home/nano/env/bin/activate;export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export CUDA_ROOT=/usr/local/cuda
export OPENBLAS_CORETYPE=ARMV8

gnome-terminal -- zsh -c "roslaunch motion_controller car_go.launch; \
sleep 20"
wait
exit 0