ISSUE TEMPLATE ver. 0.1.0

1. Which TurtleBot3 you have?

    - [ ] Burger
    - [ ] Waffle
    - [ ] Waffle Pi

2. Which SBC(Single Board Computer) is working on TurtleBot3?

    - [ ] Raspberry Pi 3
    - [ ] Intel Joule 570x
    - [ ] etc (PLEASE, WRITE DOWN YOUR SBC HERE)

3. Which OS you installed in SBC?

    - [ ] Ubuntu MATE 16.04 or later
    - [ ] Raspbian
    - [ ] etc (PLEASE, WRITE DOWN YOUR OS)

4. Which OS you installed in Remote PC?

    - [ ] Ubuntu 16.04 LTS (Xenial Xerus)
    - [ ] Ubuntu 18.04 LTS (Bionic Beaver)
    - [ ] Linux Mint 18.x
    - [ ] etc (PLEASE, WRITE DOWN YOUR OS)

3. Write down software version and firmware version

    - Software version: [x.x.x]
    - Firmware version: [x.x.x]
 
4. Write down the commands you used in order

    [Remote PC] roscore
    [SBC] roslaunch kaist_bringup kaist_robot.launch
    [Remote PC] roslaunch kaist_teleop kaist_teleop_key.launch
 
5. Copy and Paste your error message on terminal

    ```
    ... logging to /home/darby/.ros/log/0681c5f0-5343-11e8-b6e2-ac2b6e6d08ee/roslaunch-kingod-25077.log
    Checking log directory for disk usage. This may take awhile.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.

    started roslaunch server http://192.168.10.3:46024/

    SUMMARY
    ========

    PARAMETERS
    * /rosdistro: kinetic
    * /rosversion: 1.12.13
    * /kaist_core/baud: 115200
    * /kaist_core/port: /dev/ttyACM0

    NODES
      /
        kaist_core (rosserial_python/serial_node.py)

    auto-starting new master
    process[master]: started with pid [25087]
    ROS_MASTER_URI=http://192.168.10.3:11311

    setting /run_id to 0681c5f0-5343-11e8-b6e2-ac2b6e6d08ee
    process[rosout-1]: started with pid [25100]
    started core service [/rosout]
    process[kaist_core-2]: started with pid [25108]
    [INFO] [1525840798.011640]: ROS Serial Python Node
    [INFO] [1525840798.021582]: Connecting to /dev/ttyACM0 at 115200 baud
    [ERROR] [1525840798.024494]: Error opening serial: [Errno 2] could not open port /dev/ttyACM0: [Errno 2] No such file or directory: '/dev/ttyACM0'
    [kaist_core-2] process has finished cleanly
    log file: /home/darby/.ros/log/0681c5f0-5343-11e8-b6e2-ac2b6e6d08ee/kaist_core-2*.log
    ```
  
6. Please, describe detailedly what difficulty you are in 

    - HERE
