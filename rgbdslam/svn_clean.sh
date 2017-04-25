#!/bin/bash
if [ -z $1 ]
then 
	echo "haha"
	exit
fi
find $1 -name .svn -type d | xargs -n 1 rm -rf
