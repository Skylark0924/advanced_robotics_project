sudo docker rm -f ocrtoc_pb_g

sudo docker run -i -d --gpus all --name ocrtoc_pb_g --network host \
        --privileged -v /dev:/dev -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/Github/advanced_robotics_project:/root/ocrtoc_ws/src \
        -v $HOME/Desktop/docker:/root/upload \
        ocrtoc_pb_g:v1
sudo xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ocrtoc_pb_g`
