#!/bin/bash

RANGE=10
DIVIDE_X=5
DIVIDE_Y=5
NUM_SAMPLE=500

for ((i=0 ; i<$NUM_SAMPLE ; i++))
do
	let "RandNum_X = $RANDOM % $RANGE"
	let "RandNum_Y = $RANDOM % $RANGE"
#        echo $RandNum_X "," $RandNum_Y
	if [ $RandNum_X -gt 5 ]||[ $RandNum_Y -gt 5 ] ; then
		echo "-1 1:$RandNum_X 2:$RandNum_Y"
	else
		echo "1 1:$RandNum_X 2:$RandNum_Y"
	fi  
done

