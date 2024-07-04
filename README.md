**Welcome to the main autopilot software package for Virginia Tech Sailbot!**

This document will outline how to set up the software stack on theoretically any modern computer.

For more information on how to run simulations, testing, and how to run this code on the jetson, please refer to the documentation linked below.


Before setting up the docker container, in a terminal please type the following command:

```sudo chmod 777 /var/run/docker.sock```

This command gives docker full access to itself which is a little counterintuitive but helps with the scuffed things we are doing like running docker inside docker.
If this doesn't work on your os, then find some way to give full permission to that file



Commands to setup dev environment after building and launching dev docker container:

```sudo chmod -R 777 /home/```

```cd src && bash setup.sh```

```source ~/.bashrc```

Restart vscode by closing it and reopening it to make sure all of the changes refresh properly



TODO for setup dev env stuff:
- Make it so that we don't need to run sudo chmod -R 777 /home/ and sudo chmod -R 777 /var/run/docker.sock explicitly find a way to do that automatically

Known issues:
- In order to get full intellisense, you need to restart the dev container after building it. Idk how to fix this man but just go with it


add the following aliases if they help:
- alias python=python3
- alias 'colcon build'='cd /home/src && colcon build'




Heres where I put potential security risks which don't matter at all right now but may matter in the future.
In the future though if we ever want to open source this and release it as a tool for others to use or a product, this would be where to start.
- dev container uses docker.sock which needs to have permissions granted to it. So I just do it through ~/.bashrc
- sailbot_user ALL=(ALL) NOPASSWD: chmod is appended to /etc/sudoers when building the image. This is not the safest lol
  
Please contact chris (animated__ through discord) if you want to change these things but don't know if we are at a stage where you should/ how to do it