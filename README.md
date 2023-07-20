[comment]: <> (Create index)
# Index
- [INSTALLATION INSTRUCTIONS](#installation-steps)
- [RUNNING INSTRUCTIONS](#run-steps)
  - [RUNNING THE SITL simulation](#running-the-sitl-simulation)
  - [RUNNING THE SIMPLE WEB CONTROLS](#running-the-simple-web-controls)


# INSTALLATION STEPS
## INSTALL SITL (software in the loop)
[For those who are curious, you can find the ardupilot SITL documentation here](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)


This guide provides the steps to setup the ArduPilot's Software in the Loop (SITL) environment **on your raspberry pi**..

### Prerequisites

Before you begin, make sure your Linux distribution is up-to-date and has \`git\` and \`python3\` installed. 

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install git python3 python3-dev python3-pip
pip3 install future
```

### Step 1: Clone the ArduPilot repository

```bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot.git --depth 1
cd ardupilot
```

### Step 2: Update submodules

```bash
git submodule update --init --recursive
```

### Step 3: Run the Setup Script

Navigate to the directory where the setup scripts reside and run the appropriate setup script for your environment. For Ubuntu, run:

```bash
cd Tools/environment_install
./install-prereqs-ubuntu.sh -y
```

When the script finishes execution, reload the path:

```bash
. ~/.profile
```

### Step 4: Add the \`Tools/autotest\` directory to your PATH

```bash
echo -e "\nexport PATH=$PATH:$HOME/ardupilot/Tools/autotest" >> ~/.zshrc
source ~/.zshrc
```

### Step 5: Setup the sim for first use
The ardupilot instructions suggest running the sim with the -w flag on first execution to load all default values. 
Wait for the build process to fully complete! 
Once this is commpleted, and you see messages about battery status, you can exit with `ctrl+c`.
Make sure to run this from the `ArduSub` folder
```bash
cd ~/ardupilot/ArduSub/
sim_vehicle.py -w
```

Note: sim_vehicle.py is located in the ardupilot/Tools/autotest folder

## INSTALL SIMPLE WEB CONTROLS (sliders)
First and foremost
1. Fork this repo to your own github account
2. Clone your forked repo to your laptop. Make sure to do so in the home folder
```bash
git clone https://github.com/<your-gitname>/simple-web-control.git
```

### INSTALL DEPENDENCIES

Create a new virtual environment and install the dependencies

```bash
cd simple-web-control
mkvirtualenv -p python3 simple-web-control
workon simple-web-control
pip install -r requirements.txt
```
# RUN STEPS

## RUNNING THE SITL simulation
### Find your laptop ip address on the RPi network interface
The simulation will be running on your Raspberry Pi, but QGroundcontrol needs to be launched from your laptop.
For the sim to be able to find your laptop, you'll have to specify the ip address of your laptop. This is the ip address on the usb ethernet adapter, not your wlan.
- [Click here](https://support.apple.com/guide/mac-help/find-your-computers-name-and-network-address-mchlp1177/mac) for instructions on **MAC**
- [Click here](https://www.montana.edu/uit/ip/find-info-win.html) for **Windows**
- For **Linux** simply type `ifconfig` in your terminal, and find the adapter starting with `enx`
### Launch the simulator

In the command below, make sure to change the the `<hostname>` section with the correct name or ip.

```bash
cd ArduSub
sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:<hostname>:14550
```
### Finally, launch QGroundcontrol
If you haven't done so yet, you can download it from the [QGroundControl's official download page](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html).



## RUNNING THE SIMPLE WEB CONTROLS
## On the Raspberry Pi



### RUN

```bash
cd ~/simple-web-control
workon simple-web-control
python sliders.py
```

:information_source: **This will only work after you started the SITL simulator, or connected the real ROV**
Keep an eye on the terminal for the output of the script. Wait for the script to verify that the ROV has been armed


## On your laptop

- **Open a browser and go to http://backseat.local:7860**

