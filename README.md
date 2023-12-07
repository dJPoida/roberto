A cobbled together robot for working with my daughters on STEM related projects.

# Hardware
  - Brain: 
    - [Raspberry Pi 4 (4GB)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)
    - 64gb SD Card
  - Power:
    - Battery: [Turnigy 5000mAh 3S 20C LiPo](https://hobbyking.com/en_us/turnigy-5000mah-3s-20c-lipo-pack-w-xt-90.html)
    - 13v DC Power Regulation: [150W DC-DC Boost Converter](https://core-electronics.com.au/150w-dc-dc-boost-converter-10-32v-to-12-35v-6a.html)
    - 5v DC Power Regulation & USB Power: [40W (5v 8A) DC-DC Buck Converter](https://www.aliexpress.com/i/32947358135.html)
    - Voltage and Current Monitoring: [Gravity I2C Digital Wattmeter](https://core-electronics.com.au/gravity-i2c-digital-wattmeter.html)
  - Drive:
    - Controller: [Arduino Nano](https://docs.arduino.cc/hardware/nano)
    - 7v DC Power Regulation: [LM2596 3A Buck Converter](https://www.aliexpress.com/item/32952599622.html)
    - Chassis: [Generic Aluminium robot chassis](https://www.aliexpress.com/item/1005002849087125.html)
    - Wheels: [Mecanum Wheels from robot chassis](https://www.aliexpress.com/item/1005002849087125.html)
    - Motors: [Standard plastic gear motors](https://www.aliexpress.com/item/1005002849087125.html)

# Installation

## ROS2 on the Development PC
TODO: Document how to install ROS2 on the workstation

## ROS2 (via Docker) on the Raspberri Pi 4
  ### Setup the Raspberry Pi
  - Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to install **Raspberry Pi OS (64Bit)** to the SD Card
    - Change the hostname to `roberto`.local
    - Change the username and password (i.e. `roberto` / `your_password`)
    - Configure the WiFi credentials for your local network (and WiFi region)
  - Insert the SD Card into the Raspberry Pi (and optionally plug in a keyboard, mouse and monitor) and power it on
  - Either open a terminal on the Pi or connect remotely to it via SSH `ssh roberto@roberto.local`
    - To enable SSH use the keyboard / mouse and screen to access the [raspi-config](https://www.raspberrypi.com/documentation/computers/configuration.html) tool
  - Update the operating system packages
    ```
    sudo apt update
    sudo apt upgrade
    ```
  - Enable I2C
    - Run the Raspberry Pi Configuration tool `sudo raspi-config`
    - Enable `Interface Options` -> `I2C`
    - `<Finish>`
  
  ### Install Docker
  - Install Docker & Docker Compose
    ```
    cd ~
    curl -fsSL https://get.docker.com -o get-docker.sh
    chmod +x get-docker.sh 
    ./get-docker.sh
    sudo usermod -aG docker roberto
    sudo systemctl unmask docker
    sudo chmod 666 /var/run/docker.sock
    sudo apt install docker-compose
    sudo systemctl start docker
    ```
  - Restart
    `sudo shutdown -r now`
  - Clone this repo
    ```
    cd ~
    git clone https://github.com/dJPoida/roberto.git roberto
    // or
    git clone git@github.com:dJPoida/roberto.git roberto
    ```
  - Build the container
    ```
    cd ~/roberto/docker
    docker build -t ros2 .
    ```
  - Run the container
    docker-compose up -d
  
  ### Setup VS Code on the Development PC to perform remote work on the Raspberry Pi
  We can use VS Code from the Development PC to work directly on the Pi. This has many advantages including making live changes to the code on the robot.
  - Open Visual Studio Code and install the following extensions
    - [Remote Development](vscode:extension/ms-vscode-remote.vscode-remote-extensionpack)
    - [Docker](vscode:extension/ms-azuretools.vscode-docker)
    - [Docker Explorer](vscode:extension/formulahendry.docker-explorer)
  - Once installed (restart VSCode if required), open a new window and connect to the Raspberry Pi
    - `CTRL+SHIFT+P`
    - `Remote-SSH: Connect to Host...`
    - Enter `roberto@roberto.local`
    - Enter the password specified when [setting up the Raspberry Pi](#setup-the-raspberry-pi)
    - If prompted for which configuration file to use, select `~/.ssh/config`
    - Wait for VS Code to install the remote VS Code services on the Raspberry Pi
  - Install the following extensions on the Raspberry Pi (click "Install in SSH: roberto.local")
    - [Docker](vscode:extension/ms-azuretools.vscode-docker)
    - [Docker Explorer](vscode:extension/formulahendry.docker-explorer)
    - [C/C++](vscode:extension/ms-vscode.cpptools)
    - [PlatformIO IDE](vscode:extension/platformio.platformio-ide)
    - [Python](vscode:extension/ms-python.python)

# Tips and Tricks
## Console Commands for checking the status of the Raspbery Pi
- `top`: List the running processes for diagnosing CPU and Memory Utilisation
- `pinout`: Display the current Pi hardware
- `uname -m`: Output what architecture (32 or 64 bit etc...)
- `docker ps`: Get the status of the docker containers running
- `docker exec -it docker_ros2_1 bash`: Run a bash terminal inside the docker container
- `ip a`: List the network interfaces for determining the IP Address of the Pi

## Reducing Re-authentication on the Raspberry Pi
Every time you connect to the Pi via SSH it will request the password. You can circumvent this by creating an SSH key on the development PC, specifying it as the authentication method and adding it to the trusted connections on the Pi.
1. Create an SSH Key on the Development PC
    - In a terminal on the Development PC run
      ```
      ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
      ```
    - follow the prompts and if this hasn't been done before, save to the default location `~/.ssh/id_rsa`
2. Add the public key to the Raspberry Pi authorized keys
    - Open the `~/.ssh/id_rsa.pub` public key file in a text editor on the Development PC 
    - Copy the contents to clipboard
    - Open an SSH terminal to the Raspberry Pi and run the following command with the pasted contents of the public key
      ```
      echo "__PASTE_KEY_HERE__" >> ~/.ssh/authorized_keys
      ```
3. Update the ssh configuration on the development machine to always use this key when connecting to the Raspberry Pi
    - Open the file `~/.ssh/config`
    - Change the section `roberto.local` to read
      ```
      Host roberto.local
        HostName roberto.local
        User roberto
        Port 22
        PreferredAuthentications publickey
        IdentityFile "~/.ssh/id_rsa"
      ```
4. Connect to the Raspberry Pi via SSH. It should no longer request a password.

# References, Links and Shout-outs
- Official ROS guide to [Installing ROS2 on Windows](https://docs.ros.org/en/crystal/Installation/Windows-Install-Binary.html)
- When getting started on ROS2 on the Raspberry Pi I followed this excellent [Learn ROS with me](https://www.kevsrobots.com/learn/learn_ros/) guide from [Kev's Robots](https://www.kevsrobots.com/)
  