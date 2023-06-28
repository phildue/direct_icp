#!/bin/bash
docker build . -t direct_icp
docker run -it --rm --user $(id -u):$(id -g) -v$(pwd):/opt/ direct_icp /bin/bash -c "mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4"
