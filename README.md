# 雙手臂 ROS 2 主控（A→B 與獨立 B→C 測試）

本工作區只實作 ROS 2 主控、MoveIt 規劃與 CM530 的 ROS 轉接。視覺演算法、CM530 韌體及 ESP32 吸盤控制程式由其他端維護。第一階段已提供可執行的單手臂 A→B 流程；手臂 2 現在有可獨立啟動的 B→C 流程與 C 區八格分配，詳見 [B→C 使用與對接](docs/B到C使用與對接.md)。兩臂尚未接上中央 B 區仲裁，請分開測試。

## B→C 獨立測試入口

建置後執行 `ros2 launch dual_arm_control arm2.launch.py mode:=demo`，另開操作者介面 `ros2 run dual_arm_control operator_console --ros-args -p arm:=arm2`，再呼叫 `/arm2/b_ready`（`std_srvs/srv/Trigger`）。每次搬一件，八格滿停止；設定、急停與安全心跳、實機校正門檻、故障復歸、最小整合介面及測試方式見 [B→C 文件](docs/B到C使用與對接.md)。

驗證結果見 [B→C 驗證紀錄](docs/B到C驗證紀錄.md)：41 項單元測試、14 種 B→C 模擬整合情境及既有 A→B 回歸通過；實機尚待校正與驗收。

## A→B 既有流程

主控在 Home_A 透過 `/vision/env/get_zone_state` 等待 A 有物件，透過 `/vision/arm1/get_pick_target` 取得以 `arm1_base` 表示的座標，確認座標在 A 區及固定高度範圍內。接著 MoveIt 規劃到 A 上方、直線下降、等待吸盤吸取確認、直線抬起並回 Home_A；B 空且手臂已離開時取得 B 區鎖，再規劃到固定 B 座標、下降、等待吸盤釋放確認、抬起並回 Home_A，最後由視覺確認 B 有物件且手臂已離開才釋放 B 鎖。

吸盤操作目前由人員在 **PlatformIO Home 操作 ESP32**。主控沒有 ESP32 通訊協定，也不會對它送出命令；`/arm1/status` 發出一次性確認碼後，人員實際操作並確認，流程才會繼續。`auto_confirm_demo` 僅能在模擬模式使用。

模型改編自 [phantomx_pincher 的 ros2 分支](https://github.com/snt-spacer/phantomx_pincher)。隨附的吸盤 TCP 延伸長度與示範座標只是佔位值，詳見[模型來源與待校正項目](src/dual_arm_description/UPSTREAM.md)。

## 建置與模擬

環境是 Ubuntu 24.04／ROS 2 Jazzy，需安裝 `colcon`、`robot_state_publisher`、MoveIt 2 的 `move_group`、KDL、OMPL、simple controller manager，以及 `control_msgs`、`moveit_msgs`、`pyserial`。此工作區路徑含中文，因此建置輸出預設放在 `/tmp/dual-arm-ros2`。

```bash
cd /home/orin/雙手臂
bash scripts/build.sh
source /tmp/dual-arm-ros2/install/setup.bash
ros2 launch dual_arm_control arm1.launch.py mode:=demo
```

另開終端，載入同一個環境後執行：

```bash
source /tmp/dual-arm-ros2/install/setup.bash
ros2 run dual_arm_control operator_console
```

再開終端啟動單次搬運：

```bash
source /tmp/dual-arm-ros2/install/setup.bash
ros2 service call /arm1/start std_srvs/srv/Trigger '{}'
```

示範模式使用假視覺座標、理想化 CM530 回應和真正的 MoveIt 規劃／執行介面。操作者介面會在吸取及釋放時要求輸入 `yes`；即使在示範模式，預設也不會自動確認。可用 `ros2 topic echo /arm1/status` 觀察狀態。`/arm1/start` 的成功回覆只代表工作已接受，最終結果以 `/arm1/status` 的 `SUCCEEDED`／`FAULT` 為準。

整合測試會在獨立 ROS domain 啟動示範環境，並分別驗證完整流程、等待吸取時取消、釋放失敗：

```bash
bash scripts/test_integration.sh --scenario full
bash scripts/test_integration.sh --scenario cancel
bash scripts/test_integration.sh --scenario release_failure
```

## 接真實設備前必須完成

編輯 [`arm1_hardware.yaml`](src/dual_arm_control/config/arm1_hardware.yaml)：量測四軸零位與方向、可動範圍、Home_A、A 區範圍、固定取物高度、B 放置座標、工作面高度、圓片尺寸與 CM530 序列埠。更換或校正 URDF 中的真實吸盤安裝位置、`arm1_suction_tcp`、關節限制與碰撞模型。視覺服務回傳的 XYZ 必須是同一個 `arm1_base` 座標系、公尺單位，且是吸盤 TCP 的接觸目標；這需要與視覺端共同標定。兩端服務欄位來自[視覺端介面鏡像](src/vision_interfaces/UPSTREAM.md)，若視覺端更新，需同步介面版本。

完成量測與低速空載驗證後，設定 `task_coordinator.calibrated: true`、`cm530_bridge.hardware_calibrated: true` 和 `serial_port` 才能啟動硬體模式。此模式預設不使馬達上扭力，必須先確認量測狀態並呼叫 `/cm530/arm1/enable`；它先對當下位置送 HOLD，再使能扭力，不會擅自返回 Home。實體緊急停止仍須由硬體提供；`/arm1/cancel` 是軟體取消，不是急停。

```bash
source /tmp/dual-arm-ros2/install/setup.bash
ros2 launch dual_arm_control arm1.launch.py mode:=hardware config:=/absolute/path/to/measured_arm1_hardware.yaml
ros2 service call /cm530/arm1/enable std_srvs/srv/Trigger '{}'
```

發生故障時主控會停止自動流程並保留物件狀態與 B 區鎖。人員處理現場後，必須確認吸盤為空、B 區空且無手臂、量測關節在 Home_A，再用 `/arm1/status` 中的 recovery token 呼叫 `/arm1/recover`。重新啟動也不會自動消除未完成搬運的故障記錄。

CM530 轉接依據[團隊的 CM530 ROS Bridge](https://github.com/xian1022/CM530_ROS_BRIDGE.git) 的 v17/protocol 5 指令：`READ`、`AX`、`HOLD`、`TORQUE` 等；沒有修改韌體。序列逾時或回覆不符會鎖定故障，避免在不確定前一個命令結果時重送。
