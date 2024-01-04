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

## ROS2 on the Raspberry Pi 4
  ### Setup the Raspberry Pi
  - Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to install **Ubuntu Server 22.04.3 LTS (64Bit)** (Jammy Jellyfish) to the SD Card
    - Change the hostname to `roberto`.local
    - Change the username and password (i.e. `roberto` / `your_password`)
    - Configure the WiFi credentials for your local network (and WiFi region)
    - Enable SSH (under the services tab)
  - Insert the SD Card into the Raspberry Pi (and optionally plug in a keyboard, mouse and monitor) and power it on
  - Either open a terminal on the Pi or connect remotely to it via SSH `ssh roberto@roberto.local`
  - Update the operating system packages
    ```
    sudo apt update
    sudo apt upgrade
    ```
  - Install required packages
    ```
    sudo apt update
    sudo apt install -y python3-gpiozero python3-smbus python3-venv python3-pip raspi-config
    ```
  - Downgrade setuptools
    ```
    pip3 install setuptools==58.2.0
    ```
  - Access the [raspi-config](https://www.raspberrypi.com/documentation/computers/configuration.html) tool
    ```
    sudo raspi-config
    ```
    Configure the following:
    - 3: Interface Options
      - I2 SSH - Enable SSH (if not already enabled during SSD writing)
      - I5 I2C - Enable I2C Communication
    - 4: Performance Options
      - P4 Fan - Enable Cooling Fan (if connected)
    - 6: Advanced Options
      - A1 Expand Filesystem

  ### Install ROS2
  Install [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  - Setup the Ubuntu Universe Repository
    ```
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    ```
  - Add the ROS package source
    ```
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "jammy") main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
  - Install the ROS packages
    ```
    sudo apt update
    sudo apt upgrade
    sudo apt install -y ros-humble-desktop ros-dev-tools
    ```
  ### Install the Roberto code
  - (Optional) [Add an SSH key to GitHub](https://docs.github.com/en/authentication/connecting-to-github-with-ssh) to make pushing code changes easier
  - (Optional) Set your git credentials
    ```
    git config --global user.email "you@example.com"
    git config --global user.name "Your Name"
    ```
  - Clone this repository
    ```
    cd ~
    git clone https://github.com/dJPoida/roberto.git roberto
    // or
    git clone git@github.com:dJPoida/roberto.git roberto
    ```
  - Build the Roberto packages
    ```
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd ~/roberto/roberto_platform
    colcon build
    ```
  - Add the setup scripts to .bashrc to speed up opening new shells
    ```
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/roberto/.bashrc
    echo "source /home/roberto/roberto/roberto_platform/install/setup.bash" >> /home/roberto/.bashrc
    ```
  - Reboot `sudo shutdown -r now`

  ### Setup VS Code on the Development PC to perform remote work on the Raspberry Pi
  We can use VS Code from the Development PC to work directly on the Pi. This has many advantages including making live changes to the code on the robot.
  - Open Visual Studio Code and install the following extensions
    - [Remote Development](vscode:extension/ms-vscode-remote.vscode-remote-extensionpack)
  - Once installed (restart VSCode if required), open a new window and connect to the Raspberry Pi
    - `CTRL+SHIFT+P`
    - `Remote-SSH: Connect to Host...`
    - Enter `roberto@roberto.local`
    - Enter the password specified when [setting up the Raspberry Pi](#setup-the-raspberry-pi)
    - If prompted for which configuration file to use, select `~/.ssh/config`
    - Wait for VS Code to install the remote VS Code services on the Raspberry Pi
  - Install the following extensions on the Raspberry Pi (click "Install in SSH: roberto.local")
    - [C/C++](vscode:extension/ms-vscode.cpptools)
    - [PlatformIO IDE](vscode:extension/platformio.platformio-ide)
    - [Python](vscode:extension/ms-python.python)
    - [ROS](vscode:extension/ms-iot.vscode-ros)

# Tips and Tricks
## Console Commands for checking the status of the Raspbery Pi
- `top`: List the running processes for diagnosing CPU and Memory Utilisation
- `pinout`: Display the current Pi hardware
- `uname -m`: Output what architecture (32 or 64 bit etc...)
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
- Official ROS guide to [Installing ROS2 on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- When getting started on ROS2 on the Raspberry Pi I followed this excellent [Learn ROS with me](https://www.kevsrobots.com/learn/learn_ros/) guide from [Kev's Robots](https://www.kevsrobots.com/)
  