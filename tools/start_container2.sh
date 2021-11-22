docker start ocrtoc_wo_nv
sudo xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ocrtoc`
