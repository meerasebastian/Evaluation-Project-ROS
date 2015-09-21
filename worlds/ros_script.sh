#!/bin/bash

VERSION=jade  
MAJOR_VERSION_NO=`cat /etc/issue | cut -d " " -f  2 | cut -d "." -f 1`
MINOR_VERSION_NO=`cat /etc/issue | cut -d " " -f  2 | cut -d "." -f 2`
if [ "$MAJOR_VERSION_NO" -eq "14" ]
then VERSION=jade
elif [ "$MAJOR_VERSION_NO" -eq "12" ]
then VERSION=hydro
elif [ "$MAJOR_VERSION_NO" -eq "13" -a "$MINOR_VERSION_NO" -eq "04" ]
then VERSION=hydro
else VERSION=indigo
fi

echo "ROS_DISTRO: $VERSION"
echo "source /opt/ros/$VERSION/setup.bash" >> ~/.bashrc
echo $ROS_PACKAGE_PATH
source ~/.bashrc
exec bash
