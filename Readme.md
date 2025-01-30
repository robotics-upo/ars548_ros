ARS_548_RDI Driver:
================
Introduction:
---
This is a ROS driver for the Continental ARS_548 4D Radar created by the Service Robotics Lab from the Pablo de Olavide University (Spain). The driver has been tested in ROS Noetic.

This driver is used to connect with the ARS_548_RDI radar and get all of the data it sends.

First of all, this driver connects with the radar via Ethernet.\
Then it receives all the data the radar sends, translates it, and after that, it sends it in the structure of custom messages for the user to receive and interact with them.

This driver also creates two Point Clouds and a pose array with some of the information gathered and shows it using RViz 2.

About the code:
---
This project is divided in two packages: ARS548_DRIVER and ARS548_MESSAGES.

* ### ars548_driver.
    This package contains all of the code used to obtain the data from the radar, decode it and send it for the user to interact with it. 
    * ### ars548_driver.hpp
        This file contains the class that we will use to connect with the radar, obtain all of the data it sends to us and publish it for later use.

        Firstly it creates the connection with the radar.

        Then, it creates the different publishers that will send the data.

        After that, it enters an infinite loop in wich it filters the data the sensor is sending between three types of message (Status, Object and Detection), translates the data of each of the fields and stores them in an specific Structure for each message and finally it publishes the entire Structure.

    * ### ars548_driver_node.cpp
        This file launches the driver class to execute the project so we can see how it works. 


    * ### ars548_data.h
        In this file we save the different structures we will use to store and send the translated data for later use.

    * ### RadarVisualization.rviz
        This file contains all the configuration for rviz 2 to show the point clouds sent from this driver.

    * ### CMakeLists.txt and package.xml
        This two files configure an define all dependencies it needs to execute the project, including libraries, packages, etc. 
        
        They also define the files and executables this project creates when compiled.

    * ### Configurer.sh
        This file creates a connection to the default radar direction when executed. 

    * ### ars548.launch
        This is the file we will be executing using the following command:
        ```
        roslaunch ars548_driver ars548.launch
        ```
        Inside this file we create three nodes that will be executed.
        * The first node is the one that executes the code created in this project.\
        This node has three parameters that we can change in case one of them does not correspond to the actual radar value.\
        This parameters are:
            * The radar IP.
            * The connection Port.
            * The Frame ID that we will use to send the messages.

        * The second node opens an rviz 2 window to visualize the data received from the radar.\
        This node uses the **RadarVisualization.rviz** file to configure the window.

        * The third and last node creates an static-transform-publisher to transform the data obtained from the radar to the data shown in RViz (You can also change the arguments so it adapts to your project).
        
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
    * ### The rest of files are the messages to configure the radar. (They are unused in this project yet)

    
Before Downloading the code:
---
For the code to Work properly you must first do this steps.

- Have ROS Noetic, RViz, tf2 and catkin installed.
- Configure the Network connection with the radar.

In this file, we are working with the **ROS Noetic** distribution on Ubuntu 20.04, so all of the ROS commands that we will be using are from this particular distribution. In case you use another ROS distribution, this commands may change a bit.

* ### Install ROS:
    To install ROS you can follow this tutorial: <https://docs.ros.org/en/noetic/Installation.html>

* ### Install RVIZ:
    This tool will be used to show the pointClouds that this driver will create to test it.\
    To install RVIZ, run this command on your terminal:
    ```
    sudo apt-get install ros-noetic-rviz
    ```
* ### Install TF 2:
    To install TF 2 and all of its tools, run the next command on your terminal: 
    ```
    sudo apt-get install ros-noetic-tf2-ros 
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
        ./Configurer.sh
        ```  
        Once you execute the command, the program will ask you if you want to create the vlan connection and after that it will ask you to introduce the parent interface you want to use to create the connection.(It must be your phisical interface, otherwise it wont work)\
        After all that it will create the connection with the radar using the default values for the network.
    
    If you configure it manually, you will have to make this process just once. If you do it executing the **Configurer.sh** file, you will have to do it everytime you turn on your computer.

How to execute the driver
---
Once you have installed ROS, Rviz, Tf, configured your network and downloaded the project, you can execute this driver.

For executing the driver you should go to the directory in wich you have downloaded this project and execute the next commands:

* ```
    catkin_make
  ```
* ```
    source devel/setup.bash
  ```
* ``` 
    roslaunch ars548_driver ars548.launch
  ```
The first command is used to build the workspace.\
The second command is used to source the workspace.\
The last command is used to launch the project and see the results in RViz.
