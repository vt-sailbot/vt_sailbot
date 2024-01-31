# This repository contains every ros2 node that will be used in the 2023-2024 competition!

# Development Notes:
If you would like to compile and run locally in order to develop this as a colcon workspace (colcon is the build system for ros2) or use the simulation, \
all you need to do is to create a new folder for the ros2 packages add a src directory inside of it and then add this repository inside of the source directory. \
The file structure should look like sailbot_ws/src/{THE CONTENTS OF THIS REPOSITORY}. \
After that, go back to the root directory of the colcon workspace and use:
`colcon build`
This builds all of the scripts and prepares them so that you can use: `ros2 run` to run the node

If colcon build fails, this likely means you have the wrong version of setuptools from pip so you should try running the following and trying again:
`pip install setuptools==58.2.0`

If ros2 run does not work the first time, then that means that you have not yet sourced the setup.bash file and added it to your bashrc file.
Open your bashrc file which should be located at ~/.bashrc. And then add the following lines using your favorite text editor:
`source {PATH TO COLCON WORKSPACE}/install/setup.bash`
`source /opt/ros/humble/setup.bash`

Then run the following:
`source ~/.bashrc`
This will refresh your bash terminal and allow you to access the new ros nodes that you just built!


# Notes on Pushing New Changes to This Repository:
Since this is a nested github repository, this repository only holds references to each other repository. This makes a lot of things easier in some ways but harder in others. \
In order to correctly push things you need to do the following in the submodule:
```
git add --all
git commit -m "{NAME OF THE COMMIT}"
git push origin HEAD:main
```
And the following in the src directory:
```
git add --all
git commit -m "Updating References"
git push
```

# Docker Notes
We are using docker for the competition, so you will need to build these modules as docker images before you can use them on our jetson

To build all images:
    `bash build_all_images.sh`

To build a specific image:
    docker build -t {THE NAME OF THE DOCKER IMAGE} --build-arg NODE_NAME={THE NAME OF THE FOLDER YOU WOULD LIKE TO CONTAINERIZE} .

For example:
    `docker build -t sailbot_sailcode --build-arg NODE_NAME=sailcode .`

If you would like to run a docker image then please run the following:
    `docker run --privileged sailbot_{ENTER NODE NAME HERE}`

Please note that the simulation node cannot be built at this time because it doesn't really work with docker


# If You Are Making a New Node:
Whenever you set the launch command in setup.py, name it with the following formula: launch_{NAME OF THE NODE}_node
so for example your setup.py should contain something similar to this for the console scripts if the name of your python script is sailcode_node
`launch_sailcode_node = sailcode.sailcode_node:main`

also, always add a requirements.txt file for all of the pip package requirements for the node


Limitations:
    We are currently only supporting python for docker

https://www.youtube.com/watch?v=y4F1a66VOvs


TODO:
    Label each of the repositories as either telemetry server side, jetson side, or to be flashed on the teensy microcontroller