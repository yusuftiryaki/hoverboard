"""Per stack (EKF restarts), the first EKF outputs and the inputs around them."""
import math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = {"/odom", "/imu/data_raw", "/odometry/filtered/local", "/tf_static", "/tf"}
ev = []
while r.has_next():
    topic, data, t = r.read_next()
    if topic in want:
        ev.append((t * 1e-9, topic, deserialize_message(data, get_message(types[topic]))))
ekf = [e for e in ev if e[1] == "/odometry/filtered/local"]
starts = [ekf[0][0]] + [b[0] for a, b in zip(ekf, ekf[1:]) if b[0] - a[0] > 2.0]
t0 = ev[0][0]
print("bag starts", 0.0, "| stacks (first ekf msg):", [round(s - t0, 2) for s in starts])
for s in starts:
    print(f"\n######## stack whose first EKF output is at t={s - t0:.2f}")
    win = [e for e in ev if s - 1.5 <= e[0] <= s + 0.25]
    n_imu = 0
    for t, topic, m in win:
        st = getattr(m, "header", None)
        stamp = (st.stamp.sec + st.stamp.nanosec * 1e-9) if st else float("nan")
        if topic == "/odometry/filtered/local":
            p = m.pose.pose.position
            print(f"  {t - s:+.3f} EKF stamp-rec={stamp - t:+.3f} x={p.x:+.3e} vx={m.twist.twist.linear.x:+.3e} covxx={m.pose.covariance[0]:.2e}")
        elif topic == "/imu/data_raw":
            n_imu += 1
            if n_imu <= 4 or abs(m.linear_acceleration.x) > 1 or abs(stamp - t) > 0.05:
                q = m.orientation
                print(f"  {t - s:+.3f} IMU stamp-rec={stamp - t:+.3f} ax={m.linear_acceleration.x:+.3f} az={m.linear_acceleration.z:+.3f} wz={m.angular_velocity.z:+.4f} q=({q.x:.1f},{q.y:.1f},{q.z:.1f},{q.w:.1f}) frame={st.frame_id}")
        elif topic == "/odom":
            pass
        else:
            for tr in m.transforms:
                if "imu" in tr.child_frame_id or tr.child_frame_id == "base_link":
                    print(f"  {t - s:+.3f} {topic} {tr.header.frame_id}->{tr.child_frame_id}")
    odoms = [e for e in win if e[1] == "/odom"]
    if odoms:
        print(f"  odom msgs in window: {len(odoms)}, first at {odoms[0][0] - s:+.3f}")
    print(f"  imu msgs in window: {n_imu}")
