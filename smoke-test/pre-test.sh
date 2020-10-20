#!/bin/bash

# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

# Put here those instructions we need to execute before running the test

yarp where
if [ $? -eq 0 ]; then
   export stop_yarpserver="no"
   echo "stop_yarpserver == no"
else
   export stop_yarpserver="yes"
   echo "stop_yarpserver == yes"
   yarpserver --write &
   sleep 1
fi
