#!/bin/bash
echo "Current Topics Running: "
rostopic list
echo -e "\nEnter a Topic: "
read TOPIC
echo -e "\nMessage Type:" 
rostopic type $TOPIC
echo -e "\n"
MSGTYPE=`rostopic type $TOPIC`
rosmsg show $MSGTYPE
echo "To publish: rostopic pub [topic] [msg_type] [args]"

