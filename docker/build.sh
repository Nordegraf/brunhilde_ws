#!/bin/bash

# needs docker buildx and the path to a ssh key that is authorized for github

docker build --rm -t solo8 .

xhost +
docker run -i --net=host -t solo8 /bin/bash