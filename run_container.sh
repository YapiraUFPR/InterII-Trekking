docker run --rm --name bedman --privileged -it --gpus all --network="host" -v $PWD:/home/user/ws/ -v /tmp.X11-unix:/tmp/.X11-unix -v /dev:/dev -e DISPLAY=$DISPLAY -w /home/user/ws/ --volume="$HOME/.Xauthority:/root/.Xauthority:rw" yapira/bedman:humble bash