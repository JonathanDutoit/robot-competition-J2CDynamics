# J2C-Dynamics
Interdisciplinary Robot Competition at EPFL


### Connecting to Raspberry pi 5

**Note**: it is required to be in the spot building for wireless connection to the Raspberry pi 5. 

Raspberry pi 5 informations:

| Hostname | Username | Password | 
| -------- | -------- | -------- | 
| duplo-obliterator | j2cdynamics | ******* | 

After being sure that the Raspberry pi 5 is powered up, you can connect by ssh (`user@hostname`): 

```
ssh j2cdynamics@duplo-obliterator 
```

Which will ask for the password above. 


### Dev

#### Setup
```
conda env create -f environment.yml
```

```
chmod +x setup_dev.sh
```

#### Daily Use
```
source setup_dev.sh        # activate + source workspace + set DDS
cd ros2_ws
colcon build --symlink-install
```

#### Reproducibility

For reproducibility across the robot and other devs, pin the lock file after creation:

```
conda env export --no-builds -n ros2_dev > environment.lock.yml
```

```
conda env create -f environment.lock.yml
```

### References 

- [ROS Official documentation Setup ROS 2 with VSCode and Docker](https://docs.ros.org/en/kilted/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html)
- [Tobit Flatscher ROS 2 Template](https://github.com/2b-t/docker-for-robotics/tree/main/templates/ros2)
