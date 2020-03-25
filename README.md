# Wizzybug

## Install

Note:
**this is a work in progress and not ready for use**

### ROS

Install ros melodic (Ubuntu 18.04) <http://wiki.ros.org/melodic/Installation/Ubuntu> 

### Wizzybug

####  Clone repository:

```    
git clone <new repo here>
```

#### install python 3.8 or other version:

* download from official site: https://www.python.org/

* extract downloaded archive
    ```
    cd Python-3.8.0
    ```
    ```
    sudo ./configure --enable-optimizations
    ```
    ```
    sudo make altinstall
    ```

* create a virtual environment inside repo folder (recommended to use PyCharm)

* install python packages from requirements file
    ```
    cd path/to/repo
    ```
    ```
    source ./venv/bin/activate
    ```
    ```
    pip install -r requirements_python3.txt
    ```


## run

### record bag file

### replay bag file

## Docker

## help

### Troubleshooting

rospkg import error:
export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages

* replay error:

    if you get while replaying a bag file the error: `[FATAL] [1565772155.278016176]: Error reading from file: wanted 4 bytes, read 0 bytes`
    then run `rosbag reindex /path/to/bag/file.bag`

* Display error:

  cannot launch node of type robot_State_publisher...
  run: `sudo apt-get install -y ros-melodic-robot-state-publisher`

* in catkin_make command, missing pcap error:

    run: `sudo apt-get install -y pcap*`

    run: `sudo apt-get install -y libpcap0.8-dev`

### helpful commands

* see the transformations map `rosrun rqt_tf_tree rqt_tf_tree`
* see the nodes and topics map `rosrun rqt_graph rqt_graph`
* list available topics: `rostopic list`
* record bag file all topics: `rosbag record -a`
* record bag file specific topics: `rosbag record /velodyne_points /my_pcl_topic3`
* play bag file: `rosbag play --clock <bag_file_name>`
* source setup.bash `source ~/catkin_ws/devel/setup.bash`
* play ros topic from bag file: `rosbag play 2019-06-19-11-24-22.bag --topic /detections_usrr_lf_cen_8_ip_10_1_1_5 --pause`
* kill process on port: `fuser -k -n tcp 1234`
* ping all ip's in network" `for i in {1..254} ;do (ping 10.1.1.$i -c 1 -w 5  >/dev/null && echo "10.1.1.$i" &) ;done`
* ubuntu network:
  * show network connections: `nmcli con show`
  * turn off connection: `nmcli con down <connection name>`
  * turn on connection: `nmcli con up <connection name>`
