#!/bin/bash
if [ $# -lt 1 ]; then
	echo "Usage: $0 DEVICE-ADDRESS"
	exit 1
fi

curl --data "op_mode=rect" http://$1/settings/ > /dev/null
