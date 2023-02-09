# MV_SDK

## Requirement
* SW: mv sdk v:1.1.9
* HW: Atlflight pro
* Build on flight pro docker with hexagon SDk and run on Atlflight pro


## RUN
```
$ stop snav
$ imu_app &
$ rosrun ros_mv119 mvVISLAM
```

## sample output:
```
63] Running (total: 0)...
mvTrackingPose: HIGH_QUALITY
1658, 0.0045, check intrinsic camera cal , -0.2169, 0.6572, -0.7218, -51.1098, -0.7228, 0.3889, 0.5713, -10.2136, 0.6562, 0.6456, 0.3906, -10.1594, 1.3685, -0.1276, -0.0010

```


## ros output:
rostopic echo /vislam/pointcloud
---
header: 
  seq: 199
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: "vislam"
points: 
  - 
    x: 0.111269243062
    y: 0.194755464792
    z: 1.0002373457
  - 
    x: -0.309773206711
    y: 0.987472593784
    z: 1.07829880714
  - 
    x: -0.34716245532
    y: -0.781939983368
    z: 0.908532679081
