## Steps so far

```

mavlink-routerd -c mrouter.conf

python3 fcu_heartbeat_shim_v2.py   --in 127.0.0.1:14560   --out 127.0.0.1:14555   --sysid 255 --compid 191 --autopilot 0 --rate 1

ros2 run mavros mavros_node --ros-args -r __ns:=/uas1   -p fcu_url:="udp://@:14555" -p fcu_protocol:="v2.0"

```