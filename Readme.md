ARS_548_RDI Driver
================
Introduction
---
This is a ROS 2 driver for the Continental ARS_548 Radar4d created by the Service Robotics Lab from the Pablo de Olavide University (Spain). For the ROS driver, please refer to the "noetic" branch of this repository.

This driver is used to connect with the ARS_548_RDI radar and get all of the data it sends.

First of all, this driver connects with the radar via Ethernet.\
Then it receives all the data the radar sends, translates it, and after that, it sends it in the structure of custom messages for the user to receive and interact with them.

This driver also creates two Point Clouds and a pose array with some of the information gathered and shows it using RViz 2.

About the code:
---
This project consists of two ROS2 packages: ars548_driver and ars548_messages. Below you can find a brief description of each package.

* ### ars548_driver.
    This package contains all of the code used to obtain the data from the radar, decode it and send it for the user to interact with it. 
    * ### ars548_driver.hpp
        This file contains the class that we will use to connect with the radar, obtain all of the data it sends to us and publish it for later use.

        Firstly it creates the connection with the radar.

        Then, it creates the different publishers that will send the data.

        After that, it enters an infinite loop in wich it filters the data the sensor is sending between three types of message (Status, Object and Detection), translates the data of each of the fields and stores them in an specific Structure for each message and finally it publishes the entire Structure.

    * ### ars548_driver_node.cpp
        This file launches the driver class to execute the project so we can see how it works. 

    * ### ars548_filter_interface.hpp
        This file contains the class that will filter the lists of objects following the condition given by the user and then
        fills and sends a new Pointcloud with the data.

    * ### ars548_filter_example.hpp
        This file contains the class that will be used to test the filtering interface.
    
    * ### ars548_filter_node.cpp
        This file launches the filter example class to execute it and see how it works.  

    * ### radar_setup.cpp
        This file allows the user to modify its default parameters so it can be adapted to their project.

    * ### ars548_data.h
        In this file we save the different structures we will use to store and send the translated data for later use.

    * ### RadarVisualization.rviz
        This file contains all the configuration for rviz 2 to show the point clouds sent from this driver.

    * ### CMakeLists.txt and package.xml
        This two files configure an define all dependencies it needs to execute the project, including libraries, packages, etc. 
        
        They also define the files and executables this project creates when compiled.
    
    * ### configurer.sh
        This file creates a VLan connection to the default radar direction when executed. 

    * ### setup_max_dist.sh
        This file allows the user to change the default radar maximum detection distance when executed. (It is better to execute this file when the radar is starting)

    * ### ars548_launch.xml
        This is the file we will be executing using the following command:
        ```
        ros2 launch ars548_driver ars548_launch.xml
        ```
        Inside this file we create three nodes that will be executed.
        * The first node is the one that executes the code created in this project.\
        This node has three parameters that we can change in case one of them does not correspond to the actual radar value.\
        This parameters are:
            * The radar IP.
            * The connection Port.
            * The Frame ID that we will use to send the messages.

        * The second node opens an RViz 2 window to visualize the data received from the radar.\
        This node uses the **RadarVisualization.rviz** file to configure the window.

        * The third and last node creates an static-transform-publisher to transform the data obtained from the radar to the data shown in RViz 2 (You can also change the arguments so it adapts to your project).
    * ### filter_launch.xml
        This is the file we will be executing using the following command:
        ```
        ros2 launch ars548_driver filter_launch.xml
        ```
        Inside this file we create four nodes that will be executed.
        * The first node is the one that executes the driver created in this project. (The parameters are the same as the ones used in the ars548_launch.xml file)
        * The second node is the one that executes the filter created in this project.\
        This node has two parameters that we can change.\
        This parameters are:
            * The minimum velocity we want to use to filter the data.
            * The Frame ID that we will use to send the filter messages.
        * The third node opens an RViz 2 window to visualize the data received from the radar.\
        This node uses the **filter.rviz** file to configure the window.
        * The third and last node creates an static-transform-publisher to transform the data obtained from the radar to the data shown in RViz 2 (You can also change the arguments so it adapts to your project).

* ### ars548_messages
    This package contains all of the structures for the custom messages sent to the user.
    * ### Object.msg
        This file has the data structure for the object message.
    * ### Detection.msg
        This file has the data structure for the detection message.
    * ### Status.msg
        This file has the data structure for the status message.
    * ### ObjectList.msg
        This file has the data structure for the list of object messages. It has an array of messages from the Object.msg file
    * ### DetectionList.msg
        This file has the data structure for the list of detection messages. It has an array of messages from the Detection.msg file.
    * ### The rest of files are the messages to configure the radar. (They are unused in this project)

    
Before Downloading the code:
---
For the code to Work properly you must first do these steps.

- Have ROS2, RViz 2, Tf2 and colcon installed.
- Configure the Network connection with the radar.

In this file, we are working with the **ROS 2 Humble** distribution on Ubuntu 22.04, so all of the ROS2 commands that we will be using are from this particular distribution. In case you use another ROS 2 distribution, this commands may change a bit.

* ### Install ROS 2:
    To install ROS 2 you can follow this tutorial: <https://docs.ros.org/en/humble/Installation.html>

* ### Install RVIZ 2:
    This tool will be used to show the pointClouds that this driver will create to test it.\
    To install RVIZ 2, run this command on your terminal:
    ```
    sudo apt-get install ros-humble-rviz2 
    ```
* ### Install TF 2:
    To install TF 2 and all of its tools, run the next command on your terminal: 
    ```
    sudo apt-get install ros-humble-tf2-ros
    ```
* ### Install tclap:
    To install tclap, run the next command on your terminal:
    ```
    sudo apt-get install libtclap-dev
    ```
* ### Install Colcon:
    This tool is used to build ROS packages.\
    To install Colcon, execute the following command on your computer:
    ```
    sudo apt install python3-colcon-common-extensions
    ```

* ### Configure the Network to connect with the radar:

    This radar sends data using Ethernet Connection.         
    For being able to communicate with the radar and receive the data, you must first configure your network.\
    You can configure your network in two ways:
    
    * #### Configure it manually: 

        To do this you must follow the next steps:

        1. Open your terminal.

        2. Execute `nm-connection-editor`.

        3. Click on the option `Add a new connection` it is represented by a `+`.

        4. On the new Window  titled **Choose a connection Type** select `VLAN`and click on `Create`. ( **CAUTION**: Do not create a Vlan connection inside another Vlan connection)

        5. After selecting `Create`, a new window titled for editting this connection will appear.\
        In this window, you can change the default name into any other name.

        6. In the field **Parent interface**, inside the  `VLAN` tab you must select your phisical interface.

        7. In the field **VLAN id**, select a valid ID for tour connection.

        8. On the `IPv4 Settings` tab, in the **Method** field, select: **Manual** 

        9. On the `Addresses` field, click on **Add** and fill the new address with the next data:
            + `Address` field: **10.13.1.166**
            + `Netmask` field: **24**
            + `Gateway` field: **10.13.1.1**
            ###### (This values are the default data for the radar, in case you have other, change the values so they match yours)

        10. Click on the button labeled `Save`.

        With all this, you have configured your connection with the radar.

    * #### Configure it using the Configurer.sh file:
        This file automatically creates the connection with the radar using it's default values.\
        To execute this file you must go to the folder containing it and execute the following command:

        ```
        ./configurer.sh
        ```  
        Once you execute the command, the program will ask you if you want to create the vlan connection and after that it will ask you to introduce the parent interface you want to use to create the connection.(It must be your physical interface, otherwise it won't work).\
        After all that it will create the connection with the radar using the default values for the network.
    
    If you configure it manually, you will have to make this process just once. If you do it executing the **Configurer.sh** file, you will have to do it everytime you turn on your computer.

How to execute the driver
---
Once you have installed ROS2, RViz 2, Tf 2, colcon, configured your network and downloaded the project, you can execute this driver.

For executing the driver you should go to the directory in wich you have downloaded this project and execute the next commands:

 ```
    > colcon build --packages-select ars548_driver ars548_messages
    > source install/setup.bash
    > ros2 launch ars548_driver ars548_launch.xml
  ```
The first command is used to build the project.\
The second command is used to source the project.\
The last command is used to launch the project and see the results in Rviz 2.

How to execute the driver using docker
---
In case you want to execute this driver but you dont have ROS2 humble installed, you can use this method.

First you need to build the Dockerfile that is inside the docker folder. 
To do this, after downloading the project, you should go to the docker directory and execute the next command:

```
    > docker build -t ars548_rdi .
```
After that, you can run this command to run the docker:

```
    > docker run --rm -it --network host --ipc=host --pid=host   ars548rdi 
 
```
To check the results you can use rviz and open the provided configuration file at the rviz folder. 

To configure the parameters of the radar
---
If you want to see the initial parameters of the radar you can execute the next command:
```
    > ros2 run ars548_driver radar_setup.cpp
``` 
which lets you see the default parameters of the radar.
In case you want to change any of those parameters, you can execute the same command adding the arguments that are shown inside the file. For example:
```
    >ros2 run ars548_driver radar_setup.cpp -D 99
```
Changes the maximum detection distance of the radar to the value 99.

To change this same parameter, the user can also execute the **setup_max_dist.sh** file. (For it to work properly, the user may need to restart the radar)
```
    >./setup_max_dist.sh
```

The parameter list can be obtained by executing the next command:
```
    >ros2 run ars548_driver radar_setup.cpp -h
```

## Citation

If you find this driver useful for your research, please consider to add the following citation.

```
@article{FERNANDEZCALATAYUD2025102111,
title = {ars548_ros: An ARS 548 RDI radar driver for ROS},
journal = {SoftwareX},
volume = {30},
pages = {102111},
year = {2025},
issn = {2352-7110},
doi = {https://doi.org/10.1016/j.softx.2025.102111},
url = {https://www.sciencedirect.com/science/article/pii/S2352711025000780},
author = {Fernando Fernández-Calatayud and Lucía Coto and David Alejo and José Javier Carpio and Fernando Caballero and Luis Merino},
keywords = {ROS, Linux driver, Radar sensor, ARS 548},
abstract = {The ARS 548 RDI Radar is a premium model of the fifth generation of 77 GHz long-range radar sensors with new RF antenna arrays, which offer digital beamforming. This radar measures independently the distance, speed, and angle of objects without any reflectors in one measurement cycle based on Pulse Compression with New Frequency Modulation. Unfortunately, to the best of our knowledge, there are no open-source drivers available for Linux systems to enable users to analyze the data acquired by the sensor. In this paper, we present a driver that can interpret the data from the ARS 548 RDI sensor and make it available over the Robot Operating System versions 1 and 2 (ROS and ROS2). Thus, these data can be stored, represented, and analyzed using the powerful tools offered by ROS. Besides, our driver offers advanced object features provided by the sensor, such as relative estimated velocity and acceleration of each object, its orientation and angular velocity. We focus on the configuration of the sensor and the use of our driver including its filtering and representation tools. Besides, we offer a video tutorial to help in its configuration process. Finally, a dataset acquired with this sensor and an Ouster OS1-32 LiDAR sensor, to have baseline measurements, is available so that the user can check the correctness of our driver.}
}
```

## Acknowledgements

![Logos](minci.png)

This work was partially supported by the following grants: 1) INSERTION PID2021-127648OB-C31, and 2) RATEC PDC2022-133643-C21 projects, funded by MCIN/AEI/ 10.13039/501100011033 and the "European Union NextGenerationEU / PRTR".
