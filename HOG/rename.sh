#!/bin/bash

for fileName in $1/*
do 
#echo $fileName
newFileName=`echo $fileName | sed s/_[0-9]*x[0-9]*//g` 
newFileName=`echo $newFileName | sed s/\.png//g` 
#echo $newFileName
mv $fileName $newFileName
done
