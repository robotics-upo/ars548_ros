#! /bin/bash

#This function is used to ask the user if they want to configure all of the
#connections or default data of the code automatically
configurer(){
    echo "Do you want to have create the vlan connection? (y or n)"
    read a
    if [ "$a" = "y" ] || [ "$a" = "Y" ] 
    then 
        configureConnection
    else
        if  [ "$a" != "n" ] && [ "$a" != "n" ]
            then
            echo "Unknown answer. Try with y or n"
            exit
        fi
    fi
    echo "All configured!!"
}
#This function calls the nm-connection-editor so the user can create a virtual connection to the radar.
configureConnection(){
    echo "Write your parent interface name"
    read parent
    sudo ip link add link $parent name radarConnection type vlan id 19
    sudo ip addr add 10.13.1.166/24 dev radarConnection
    sudo ip link set radarConnection up
    sudo ip route replace default via 10.13.1.1 dev radarConnection                                                                      
    echo "Connection Created"
}


#This first lines of the main code ask the user if they want to 
#install all of the necesary elements to execute the project.
#If they say no, it just exites the program 
echo "this File will configure all of the necesary data to execute the Project."
configurer 
exit