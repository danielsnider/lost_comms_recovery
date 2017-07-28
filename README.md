# lost_comms_recovery [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__lost_comms_recovery__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__lost_comms_recovery__ubuntu_xenial_amd64__binary)

If your robot loses connection to the base station it will navigate to a configurable home.

Full documentation on wiki: [http://wiki.ros.org/lost_comms_recovery](http://wiki.ros.org/lost_comms_recovery)

## Quick Start

1. Install:

```
$ sudo apt-get install ros-kinetic-lost-comms-recovery
```
2. Launch node:

```
$ roslaunch lost_comms_recovery lost_comms_recovery.launch
```

3. Normal output:

```
$ roslaunch lost_comms_recovery lost_comms_recovery.launch ips_to_monitor:=192.168.190.136

[INFO] Monitoring base station on IP(s): 192.168.190.136.
[INFO] Connected to base station.
[INFO] Connected to base station.
...
[ERROR] No connection to base station.
[INFO] Connected to move_base.
[INFO] Executing move_base goal to position (x,y) 0.0, 0.0.
[INFO] Inital goal status: PENDING
[INFO] This goal has been accepted by the simple action server
[INFO] Final goal status: SUCCEEDED
[INFO] Goal reached.
```

**Full documentation on wiki: [http://wiki.ros.org/lost_comms_recovery](http://wiki.ros.org/lost_comms_recovery)**
