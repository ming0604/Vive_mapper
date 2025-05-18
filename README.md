---
title: vive_mapper_readme

---

# Vive_mapper
## 操作步驟
1.需要==LiDAR scan topic==以及==ground truth pose(以odom topic發佈）==
設置要online跑用真實時間 or 用bag跑用虛擬clock時間
設置scan topic及ground truth odom topic

    <param name="/use_sim_time" value="true" />
    <arg name="scan_topic" default="front/scan" />
    <arg name="gt_odom_topic" default="base_gt_odom" />

2.設置參數
其中range_used_max決定使用的scan的最遠距離
frame id 也要確定好
 <!-- Launch Vive_mapper node -->
    <node pkg="Vive_mapper" type="Vive_mapper" name="Vive_mapper" output="screen">
        <!-- Set parameters for mapping -->
        <param name="resolution" value="0.025" />
        <param name="width" value="1000" />
        <param name="height" value="1000" />
        <param name="occ_update_prob" value="0.7" />
        <param name="free_update_prob" value="0.4" />
        <param name="occ_threshold" value="0.95" />
        <param name="free_threshold" value="0.15" />
        <param name="stationary_distance_threshold" value="0.025" />
        <param name="stationary_angle_threshold" value="0.017453" />
        <param name="range_used_max" value="30.0" />
        <param name="base_frame_id" value="base_link" />
        <param name="Lidar_frame_id" value="front_laser" />
        <!-- Optional remapping -->
        <remap from="scan" to="$(arg scan_topic)"/>
        <remap from="base_gt_odom" to="$(arg gt_odom_topic)"/>
    </node>