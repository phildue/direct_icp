# direct_icp
Implementation of direct image alignment of two RGBD frames, minimizing photometric and depth error with respect to 6-DOF pose as in [DirectIcp](doc/direct_icp.pdf) and [DVO](doc/kerl13iros.pdf).

# Build & Run (docker)
1. Build image
```
docker build . -t direct_icp
```
2. Start interactive session
```
docker run -it --rm --user $(id -u):$(id -g) -v$(pwd):$(pwd) --workspace_dir $(pwd) direct_icp
```
3. Follow steps as in (#Run)

# Build & Run (native)
1. Install dependencies
```
sudo ./install_dependencies.sh
```
2. Run build script
```
./build.sh
```

# Run
## Prepare data
1. Download tum tools:
```
wget https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/associate.py
wget https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/evaluate_rpe.py
```
2. Download sequence e.g.:
```
mkdir ./data
cd ./data
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_desk.tgz
tar zxvf rgbd_dataset_freiburg2_desk.tgz 
```
3. Create association file following [here](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/tools):
```
python associate.py ./data/rgbd_dataset_freiburg2_desk/depth ./data/rgbd_dataset_freiburg2_desk/rgb > ./data/rgbd_dataset_freiburg2_desk/assoc.txt
```
## Start pipeline
```
./build/main_tum ./data/rgbd_dataset_freiburg2_desk assoc.txt
```

## Evaluate
```
./build/main_tum ./data/rgbd_dataset_freiburg2_desk assoc.txt > ./trajectory.txt
python evaluate_rpe.py --verbose --fixed_delta ./data/groundtruth.txt trajectory.txt
```

