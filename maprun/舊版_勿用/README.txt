這三支腳本啟動的是**舊版** smartnav_navigation 套件，不是現在在用的
smartnav_navigation_cc。移到這裡是為了避免誤用。

  run_brain.sh   -> 舊的 brain.launch.py
  run_nav2.sh    -> 舊的 nav2.launch.py
  create_map.sh  -> 舊的建圖入口（沒有帶 _cc 的參數）

舊版已知的問題（2026-07-31 實測，詳見 smartnav_navigation_cc/README.md 第 1 節）：
  - 建完圖之後導航就壞掉：slam_toolbox 從來沒有被 deactivate，
    /map 與 map->odom_combined 都有兩個發布者
  - planner_server 連噴 542 次 "Start occupied"
  - 恢復行為裡有 <Spin>，阿克曼車不能原地轉，每次都走到逾時才失敗
  - 全域定位時車子完全不動（發 TwistStamped 但底盤訂 Twist，型別不符）

現在該用的是：
  ~/maprun/run_sensors_cc.sh      底盤 + 雷達 + IMU 補償
  ~/maprun/run_nav_cc.sh          導航（auto / mapping）
  ~/maprun/create_map_cc.sh       建圖
  ~/maprun/kill_node_cc.sh        安全停節點（不要用 pkill -f）
