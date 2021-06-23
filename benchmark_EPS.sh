#!/bin/bash
for file in dataset/merged-mesh/$1/* ;do
  echo $file
  array=(${file//// });
  directory_name=${array[2]};
  f="$(basename -- $file)";
  array2=(${f//-/ });
  map_name=${array2[0]};
#  echo   ./bin/testEPS $directory_name $map_name
  ./bin/testEPS $directory_name $map_name

done


