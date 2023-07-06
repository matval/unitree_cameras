#!/usr/bin/env bash

# First, kill all the processes using the cameras
kill $(ps aux | grep '[c]amera' | awk '{print $2}')
kill $(ps aux | grep '[i]mage' | awk '{print $2}')