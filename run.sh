#!/bin/bash
FOLDER=${1:-/$HOME/data}    
SEQUENCE_ID=${2:-"assoc.txt"}    

docker run --rm --user $(id -u):$(id -g) -v$(pwd):/opt/ -v$FOLDER:$FOLDER direct_icp:latest bash -c "build/main_tum $FOLDER $SEQUENCE_ID"
