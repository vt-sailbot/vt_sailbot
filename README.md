This repository contains every ros2 node that will be used in the 2023-2024 competition.

We are using docker for the competition, so you will need to build these modules as docker images before you can use them on our jetson

To build all images:
    `bash build_all_images.sh`

To build a specific image:
    docker build -t {THE NAME OF THE DOCKER IMAGE} --build-arg NODE_NAME={THE NAME OF THE FOLDER YOU WOULD LIKE TO CONTAINERIZE} .

For example:
    `docker build -t sailbot_sailcode --build-arg NODE_NAME=sailcode .`

Please note that the simulation node cannot be built at this time because it doesn't really work with docker

Development Notes:
    if you would like to compile and run locally in order to develop this as a ros2 workspace or use the simulation, \
    all you need to do is to create a new ros2 package and then replace the src directory of the ros2 package with this repository. \
    after that, go back to the root directory of the ros2 workspace and use:
    `colcon build`

If you are making a new node:
    whenever you set the launch command in setup.py, name it with the following formula: launch_{NAME OF THE NODE}_node
    so for example your setup.py should contain something similar to this for the console scripts
    `launch_sailcode_node = sailcode.sailcode_node:main`

    also, always add a requirements.txt file for all of the pip package requirements for the node