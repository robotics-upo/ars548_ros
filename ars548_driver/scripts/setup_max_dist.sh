#! /bin/bash

modifyMaxdist(){
    echo "Do you want to change the maximum detection distance of the radar?"
    read a
    if [ "$a" = "y" ]|| [ "$a" = "Y" ]
    then
        sendMessage
    else
        if ["$a" != "n" ] && [ "$a" != "N" ] 
            then
            echo "Unknown answer. Try with y or n"
            exit
        fi
    fi
    echo "Max distance changed!!"
}

sendMessage(){
    echo "Please introduce the new maximum distance you want the radar to use (it has to be between 93 and 1514)"
    read n
    while ![["$n"=~[0-9]+$ ]];do
        echo "Invalid value. Please insert a number between 93 and 1514"
        read n
    done
    while true; do
    ros2 run ars548_driver radar_setup -D "$n" 
    if [ $? -eq 0 ]; then
        echo "Configuration is equal to status. Breaking the loop."
        break
    else
        echo "Configuration is different to status. Retrying in 1 second (Maybe you should try to restart the radar)"
        sleep 1
    fi
done
}
echo this file will change the Maximum Detection Distance of the radar
modifyMaxdist
exit