Removert
---


No ros version!

pcd are enough to run this program. Need transformed and pose in VIEWPOINT. Please reference our dufomap benchmark for more detail.

TODO add DUFOmap benchmark link Here!!!

We will output all the scripts and dataset! Don't worry. We are working on it. 
Message from [Qingwen ZHANG*](https://kin-zhang.github.io/) and [Daniel Duberg*](https://github.com/danielduberg)

## Install & Build

Test computer and System:

- Desktop setting: i9-12900KF, 64GB RAM, Swap 90GB, 1TB SSD
- System setting: Ubuntu 20.04
- Test Date: 2023/04/05
- Modified Version commit: https://github.com/irapkaist/removert/tree/726f48b36b27eae9a84c75f07b953f8c791caf9b


Dependencies:
### PCL


### glog gflag (only for debug)
glog gflag for debug only, will remove on release version
```sh
sh -c "$(wget -O- https://raw.githubusercontent.com/Kin-Zhang/Kin-Zhang/main/Dockerfiles/latest_glog_gflag.sh)"
```

### yaml-cpp
Please set the FLAG, check this issue if you want to know more: https://github.com/jbeder/yaml-cpp/issues/682, [TOOD inside the CMakeLists.txt](https://github.com/jbeder/yaml-cpp/issues/566)

If you install in Ubuntu 22.04, please check this commit: https://github.com/jbeder/yaml-cpp/commit/c86a9e424c5ee48e04e0412e9edf44f758e38fb9 which is the version could build in 22.04

```sh
cd ${Tmp_folder}
git clone https://github.com/jbeder/yaml-cpp.git && cd yaml-cpp
env CFLAGS='-fPIC' CXXFLAGS='-fPIC' cmake -Bbuild
cmake --build build --config Release
sudo cmake --build build --config Release --target install
```
### Build
```bash
mkdir build && cd build
cmake .. && make
```

## RUN

```
./removert_run /home/kin/workspace/DUFOMap/data/KITTI_00 ../config/seq_00.yaml
```

I recommend to run 5-10 frames first to check the result, then run the whole sequence. Like here:
```
./removert_run /home/kin/workspace/DUFOMap/data/KITTI_00 ../config/seq_00.yaml 5
```


## Demo



**But DUFOMap is better** than ERASOR and fastest! please check our benchmark here [TODO](TODO)

DUFOMap Timing print in my desktop setting:
```bash
DUFOMap Timings
         Component      Total   Last    Mean    StDev    Min     Max     Steps
        1. Read          0.34   0.0022  0.0024  0.0007  0.0020  0.0079     141
        2. Integrate     8.79   0.0587  0.0624  0.0136  0.0472  0.1694     141
        3. Propagate     1.57   0.0107  0.0111  0.0033  0.0073  0.0320     141
        4. Clustering    0.23   0.2308  0.2308    nan   0.2308  0.2308       1
        5. Query         1.33   1.3294  1.3294    nan   1.3294  1.3294       1
        6. Write         8.23   8.2331  8.2331    nan   8.2331  8.2331       1
```