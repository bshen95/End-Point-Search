#!/bin/bash

echo Making directory for CPD and visibility_graph
mkdir dataset/cpd
mkdir dataset/visibility_graph
mkdir dataset/cpd/$1
mkdir dataset/visibility_graph/$1
for file in dataset/merged-mesh/$1/* ;do

  echo Creating visibility graph and cpd for $file ..........
  array=(${file//// });
  directory_name=${array[2]};
  f="$(basename -- $file)";
  array2=(${f//-/ });
  map_name=${array2[0]};
#echo ./bin/build_visibility_graph dataset/merged-mesh/$directory_name/$map_name-merged.mesh dataset/grid/$directory_name/$map_name.map dataset/visibility_graph/$directory_name/$map_name.vis
./bin/build_visibility_graph dataset/merged-mesh/$directory_name/$map_name-merged.mesh dataset/grid/$directory_name/$map_name.map dataset/visibility_graph/$directory_name/$map_name.vis

#echo ./bin/construct_cpd dataset/visibility_graph/$directory_name/$map_name.vis dataset/cpd/$directory_name/$map_name
./bin/construct_cpd dataset/visibility_graph/$directory_name/$map_name.vis dataset/cpd/$directory_name/$map_name
  echo Finish preprocessing .......................................................................
  echo
  echo
done

