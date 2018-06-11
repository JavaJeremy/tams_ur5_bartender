#!/bin/bash
#Use default browser - assuming that it is firefox
#Not opening if firefox is already open
if ! pgrep -x "firefox" > /dev/null
then
	fileFragment=$"*tams_ur5_bartender_gui/public_html/index.html"
	fullPath=$(find ../ -path $fileFragment)
	xdg-open $fullPath &
fi
