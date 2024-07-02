!!!NOTE "NOTE: If You Meet Dificulties Installing Docker"
    Ensure that you meet the docker's system requirements listed on their documentation, if not then attempt to contact an officer or look through the docker documentation for what to do if you do not meet the system requirements. Heres the link to the Docker documentation [Docker Documentation](https://docs.docker.com/get-docker/)

# <p style="text-align: center;"> Installing Docker on Windows </p>
In order to install Docker on windows, first we must install WSL (Windows Subsystem in Linux).  

Open the command prompt ***with administrator privileges*** and type the following commands  
``` sh
wsl.exe --install -d Ubuntu-22.04
```   
``` sh
wsl --set-default Ubuntu-22.04
```   
Enter your user info to complete the installation  
__________________________________________________________________________

With these steps, this will have installed WSL and Ubuntu 22.04. Docker requires Ubuntu WSL to work on Windows. 
For more information see the official WSL documentation:  [Official WSL Documentation](https://learn.microsoft.com/en-us/windows/wsl/install)

Next, we must install the docker desktop application and connect it up to WSL.
__________________________________________________________________________
The following link contains the download link for docker desktop on windows. 
Follow the download instructions, and when you are done, you should have docker installed!  
[Docker Desktop for Windows Install Page](https://docs.docker.com/desktop/install/windows-install/)

<br>
# <p style="text-align: center;"> Installing Docker on Ubuntu Linux </p>
____________________________________________________________________________
Please type the following sets of commands in a terminal.
``` sh
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```
``` sh
# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```
```sh
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
____________________________________________________________________________

<br>
# <p style="text-align: center;"> Installing Docker on Mac OS</p>
Please follow the official docker installation instructions as I am not too familiar with Mac computers. However if you own a Mac and would like to contribute to the documentation by writing up explicit installation instructions for Mac, please feel free. [Official Docker Installation Instruction Mac](https://docs.docker.com/desktop/install/mac-install/)

<br>
# <p style="text-align: center;"> Installing Docker on Other Operating Systems </p>
Documentation for the rest of the operating systems docker supports can be found here: [Official Docker Installation Instructions for Every OS](https://docs.docker.com/engine/install/).

