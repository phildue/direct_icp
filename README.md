# direct_icp
Implementation of direct image alignment of two RGBD frames, minimizing photometric and depth error with respect to 6-DOF pose as in [DirectIcp](doc/direct_icp.pdf) and [DVO](doc/kerl13iros.pdf).

# Prerequisits

- Installation of docker a la:
```
sudo apt install docker.io
sudo usermod -aG docker $(whoami)
sudo reboot
```

# Build
```
./build.sh
```
# Prepare Data

1. Download sequence e.g.:
```
mkdir ./data  && cd ./data && wget https://cvg.cit.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_desk.tgz && tar zxvf rgbd_dataset_freiburg2_desk.tgz 
```

2. Create association file as described [here](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/tools):
```
wget https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/associate.py
docker --rm -v$(pwd):/opt direct_icp python associate.py ./data/rgbd_dataset_freiburg2_desk/depth ./data/rgbd_dataset_freiburg2_desk/rgb > ./data/rgbd_dataset_freiburg2_desk/assoc.txt
```

# Run
1. Start pipeline e.g.:
```
./run.sh ./data/rgbd_dataset_freiburg2_desk assoc.txt
```

# Evaluate
```
./run.sh ./data/rgbd_dataset_freiburg2_desk assoc.txt > ./trajectory.txt
wget https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/evaluate_rpe.py
docker --rm -v$(pwd):/opt direct_icp python evaluate_rpe.py --verbose --fixed_delta ./data/groundtruth.txt trajectory.txt
```

