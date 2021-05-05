#!/bin/sh

rproc_class_dir="/sys/class/remoteproc/remoteproc0"
fmw_dir="/lib/firmware"
fmw_name="LA_M4_FW_CM4.elf"

cd $(/usr/bin/dirname $(/usr/bin/readlink -f $0))

if [ $1 == "start" ]
then
        # Start the firmware
        /bin/rm -f /lib/firmware/$fmw_name
        /bin/ln -s $PWD/lib/firmware/$fmw_name $fmw_dir
        /bin/echo -n "$fmw_name" > $rproc_class_dir/firmware
        /bin/echo -n start > $rproc_class_dir/state
fi

if [ $1 == "stop" ]
then
        # Stop the firmware
        /bin/echo -n stop > $rproc_class_dir/state
fi
