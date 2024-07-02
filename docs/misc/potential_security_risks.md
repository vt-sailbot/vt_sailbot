# Heres where I put potential security risks which don't matter at all right now but may matter in the future.

In the future though if we ever want to open source this and release it as a tool for others to use or a product, this would be where to start.  
- dev container uses docker.sock which needs to have permissions granted to it. So I just do it through ~/.bashrc  
- `sailbot_user ALL=(ALL) NOPASSWD: chmod` is appended to /etc/sudoers when building the image. This is not the safest lol
  
Please contact chris (animated__ through discord) if you want to change these things but don't know if we are at a stage where you should/ how to do it