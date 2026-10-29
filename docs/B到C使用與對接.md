# B→C 獨立控制與第 17 版對接

本模組位於 `dual_arm_control`，沿用既有 MoveIt、`FollowJointTrajectory`、CM530 protocol 5 和既有 `GetZoneState`、`GetGridState`、`VerifyPlace` 視覺服務。不修改 CM530 韌體、不假設實機尺寸、不控制 ESP32。預設使用相機逐格驗證，再由操作者複核；吸盤仍由操作者操作及確認。A→B 尚未與本模組共同執行；測試時手臂一必須固定在手臂二所有路徑之外。

依據：`CM530_ROS_BRIDGE-main/17 ROS to CM530 ver. dual arm/ROS_CM530_INTERFACE_SPEC.md`。arm2 的 j1..j4 對應 ID **12、1、8、16**，57600 baud／8N1。序列埠仍只由一個 `cm530_bridge` 持有。

## 1. 建置、啟動與手動單次測試

```bash
cd /home/orin/雙手臂
bash scripts/build.sh
source /tmp/dual-arm-ros2/install/setup.bash
ros2 launch dual_arm_control arm2.launch.py mode:=demo
```

第二個終端：

```bash
source /tmp/dual-arm-ros2/install/setup.bash
ros2 run dual_arm_control operator_console --ros-args -p arm:=arm2
```

第三個終端：

```bash
source /tmp/dual-arm-ros2/install/setup.bash
ros2 service call /arm2/b_ready std_srvs/srv/Trigger '{}'
ros2 topic echo /arm2/status
```

首次測試須清空 C 八格；已有持久化紀錄時依紀錄核對現場。每次觸發搬一件；`/arm2/start` 是相同操作的別名。成功回覆只表示接受任務，結果看 `/arm2/status` 的 `SUCCEEDED`、`FULL`、`FAULT`。一次完成後再觸發下一件；不自動重播 B_READY，也不排隊接受重複觸發。依最小編號空格選格，初始空臺即依 1→8；第九次請求會被拒絕。

人工介面會依序要求：核對 C 全部占用與目標格空位 → 實際吸取確認 → 實際釋放確認 → 確認物件位於指定格內。模擬時可回答 yes；實機必須以現場觀察為準。`--ros-args -p arm:=arm2` 不會改變舊的 arm1 使用方式。

`auto_confirm_demo:=true` 只可搭配 demo，自動確認是合成資料，不能代表真實吸取或視覺驗證。

## 2. 動作與狀態

1. B_READY → 檢查安全心跳、B 有物件、B 無手臂，且 C 總數與保存紀錄相符。初始讀回必須已在 Home_C，讓手眼相機二的標定有效。
2. 在 Home_C 等待影像更新，查既有 get_grid_state，比對八格占用與配置座標，再人工複核；套用工作面、障礙物、已放置物件碰撞模型。
3. 預檢 B/C 接觸與上方姿態的碰撞感知 IK；規劃回 Home_C。四軸不能任意達成所有六維姿態，無解就停止。
4. 再確認 B 有料且無手臂，保存 B_RESERVED／BUSY → 規劃到 B 上方 → 直線下降。
5. 吸取確認 → 加入持物碰撞模型 → 直線抬升 → 規劃回 Home_C。
6. 視覺確認 B_EMPTY 且手臂已離開，才釋放本機 B 鎖。
7. 規劃到 C 下一格上方 → 直線下降 → 釋放確認 → 移除持物模型 → 直線抬升。
8. 加入該格物件模型 → 回 Home_C → 全景視覺確認 C 總數加一 → 既有 verify_place 確認指定格已有物件，get_grid_state 比對完整八格預期占用 → 人工複核 → 持久保存占用。
9. 全滿轉 FULL，拒絕新任務，保留 B 區後續餘料。

一般移動沿用 OMPL；下降／抬升使用避碰 Cartesian path，要求完成率至少 0.999，並設關節跳變門檻。每一段在執行前重新規劃與驗證；預檢 IK 不是整條未來路徑一定可行的保證。規劃、服務、人工確認、區域等待、軌跡執行及關節到位均有期限。

## 3. 實際沿用與新增介面

| 介面 | 型別 | 契約 |
|---|---|---|
| `/vision/env/get_zone_state` | `vision_interfaces/srv/GetZoneState` | **既有**：success、b_has_object、b_arm_clear、c_object_count；success=false 不採信資料 |
| `/vision/arm2/get_grid_state` | `vision_interfaces/srv/GetGridState` | **既有**：occupied[8]、place_x/y/z[8]、all_full、success；核對占用與固定座標 |
| `/vision/arm2/verify_place` | `vision_interfaces/srv/VerifyPlace` | **既有**：輸入 slot=1..8，檢查 success 與 placed；尺寸排序另由 require_size_order 控制 |
| `/joint_states` | `sensor_msgs/msg/JointState` | **既有**：成功 READ 換算為 rad，主控拒絕過期狀態 |
| `/arm2_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | **既有 bridge 已支援 arm2**；讀回穩定後才宣告完成 |
| `/cm530/arm2/enable` | `std_srvs/srv/Trigger` | **既有 bridge 介面**；HOLD 成功後 TORQUE,1，無隱含 HOME |
| `/arm2/b_ready`、`/arm2/start` | `std_srvs/srv/Trigger` | **新增**：一次性任務觸發；busy/fault/full 時拒絕，仍須查視覺及安全狀態 |
| `/arm2/cancel` | `std_srvs/srv/Trigger` | **新增**：取消流程；不是實體急停 |
| `/arm2/status` | `std_msgs/msg/String` JSON | **新增**：state、busy、cycle_id、holding、b_locked、slot、occupied[8]、message、確認 token |
| `/arm2/confirm` | `dual_arm_interfaces/srv/ConfirmStep` | 沿用既有型別；必須帶當前一次性 token，拒絕舊碼與重複確認 |
| `/arm2/recover` | 同上 | 僅接受 idle FAULT/FULL 的 recovery token；人工清臺後再核對視覺及 Home_C |
| `/handoff/b_state` | `std_msgs/msg/String` JSON | **新增**：signal=BUSY/B_EMPTY/UNKNOWN；B_RESERVED 時 signals 另含 B_OCCUPIED；owner、cycle_id、busy、standalone |
| `/handoff/arm1_clear` | `std_msgs/msg/Bool` | **新增安全心跳**：true 表示手臂一固定在所有手臂二路徑之外，持續更新 |
| `/safety/estop` | `std_msgs/msg/Bool` | **新增安全心跳**：false=急停迴路正常；true 鎖存軟體停止，後續 false 不自動解除 |

兩項安全心跳必須都已收到且未超過 `safety_max_age`（預設 2 秒），建議安全監督端至少 10 Hz 發布。沒有監督端時實機不會開始；不能以盲目固定發布 true/false 替代硬體迴路與現場判斷。demo 專用 `arm2_mock_vision` 才會提供假心跳。

原工作區第一階段只鏡像兩個 service；本次從現有 `three-camera-vision-ros2-main` 原樣補入 `GetGridState.srv` 與 `VerifyPlace.srv`，沿用真正的 `/vision/arm2/get_grid_state`、`/vision/arm2/verify_place`。相機資料只用來驗證，不會覆蓋 YAML 的固定目標。任一服務失敗、占用不符或座標偏差超過 `grid_position_tolerance` 都停止。服務回應沒有時間戳；沿用視覺節點內建影像新鮮度檢查，並在 Home_C 等待 `home_settle` 後呼叫。

`require_size_order=false` 因本任務只要求按格號放置，不要求依物件大小排序；若另有尺寸遞增需求可設 true，同時要求 order_ok 且 wrong_slots 為空。即使不檢查排序，success=false 仍不採信。

尚未啟動手眼相機二時，可明確設 `grid_verification=false` 使用人工逐格確認與全景計數的替代測試模式；不會在服務失敗時自動降級。此模式禁止 auto_confirm_demo，避免把無相機／無人工的格位驗證當成成功。正式實機驗收應使用預設 true。

## 4. B、C、姿態與碰撞配置

- `config/arm2_demo.yaml`：僅供示範，以現有 arm1 範例模型更名為 arm2，並非真實手臂二的量測。
- `config/arm2_hardware.yaml`：刻意留下 `.nan` 與 false 校正旗標，不能直接實機動作。
- `b_pick` 是 `[x,y,z,qx,qy,qz,qw]`；`c_poses` 是依 1..8 順序串接的 56 個數值。單位公尺，預設 `arm2_base`；四元數須正規化。這是**吸盤 TCP 接觸物件頂面**的姿態。
- 二列四欄的編號慣例：靠近 B 的第一列由左至右 1、2、3、4；第二列由左至右 5、6、7、8（對應既有視覺 warp_rotation_k 校正）。實際每格位置個別量測，不根據假定間距推算。
- `approach_height` 沿基座 +Z 抬升，接觸和抬升時維持同一四元數；若平台傾斜需先重新設計接近路徑，不能直接使用本假設。
- `home_joints` 是量測的 Home_C，rad；`base_frame`、`tcp_link`、`joint_names` 對應所提供 URDF/SRDF；啟用既有 grid 服務時，base_frame 必須為其契約指定的 arm2_base。
- `table_size`、`table_center` 與 `obstacle_boxes`（每組 `[cx,cy,cz,sx,sy,sz]`）描述工作面、平台、固定手臂一及周邊障礙。物件模型用量測的 `disk_radius`、`disk_thickness`；目前限圓片物件。
- 馬達零位、方向、raw 限位、rad 限位、回讀誤差、穩定次數與序列排程，須依實機校正。預設 512 不是已校正 Home_C。

實機模式另外要求：`calibrated=true`、非空 `calibration_id`、`standalone_arm1_secured=true`，bridge 的 `hardware_calibrated=true` 及明確序列埠，以及明確提供**量測過的** `robot_urdf`、`robot_srdf`。改旗標本身不是校正，必須先完成低速空載驗證。

```bash
# 完成上述量測、安全監督與視覺服務後才可使用
ros2 launch dual_arm_control arm2.launch.py mode:=hardware \
  config:=/absolute/path/to/measured_arm2.yaml \
  robot_urdf:=/absolute/path/to/measured_arm2.urdf \
  robot_srdf:=/absolute/path/to/measured_arm2.srdf
# 確認現場與讀回姿態後，明確啟用（不自動回 Home）
ros2 service call /cm530/arm2/enable std_srvs/srv/Trigger '{}'
```

## 5. 故障、急停與保存

CM530 開埠核對 VERSION,5／PONG，查詢 GET_HOME；不把韌體 ACK 當成到位。沿用一問一答、AX 逐點送出、READ 容差及連續穩定次數。逾時／回覆錯配不自動重送，通訊失同步時也不冒險送 HOLD。完整錯誤與軌跡取消會停止後續排程，若連線同步才請求 HOLD。

急停輸入同時進入主控與 bridge；bridge 鎖存拒絕後續軌跡與使能，正在執行時停止新增點並嘗試 HOLD。正在傳輸的一筆命令不能撤回。HOLD、ROS topic、軟體取消都**不是硬體急停**，沒有保證停止距離；實機必須有獨立硬體急停迴路。程式退出或 ROS 通訊失效也不能保證物理停止。真實吸盤供電與掉落防護需由硬體處理。

`~/.ros/dual_arm/arm2_journal.json` 保存每步狀態、八格占用、B 鎖與持物狀態。以暫存檔 fsync 後原子替換；同一 journal 有程序鎖。未完成任務或任務配置／URDF／SRDF 指紋改變後重啟轉 FAULT；成功紀錄保留占用，FULL 重啟也不清臺。不要以刪 journal 作為復歸方法。

取消／故障不自動放物、不自動回家、不重試。若故障時仍在 B 或未驗證離區，B 鎖保留；離區已驗證才不保留。尚未確認的 C 格不寫成已占用，整體 FAULT 阻止再次使用該格。

復歸採保守的**全清臺**策略：人工移除工具物件、清空 B/C，手臂二回 Home_C、手臂一固定於安全區，再用 recovery token 確認。節點讀回 Home_C、驗證 B/C 空與 B 無手臂，並確認手眼相機八格全空後清除八格紀錄，不發送復歸動作。急停鎖存需先處理實體原因及檢查，再重啟；重啟仍保留未完成任務故障。

模擬 FULL/FAULT 清臺：先呼叫 `/arm2_mock/clear_table`，再透過操作者介面確認 recovery；此服務只改合成影像狀態，不控制設備。

## 6. 日後接 A→B

本版本是**獨立測試器**。`/handoff/b_state` 是本次搬運最後驗證結果的狀態通知（可含重啟後保留資料），不是即時相機觀測，也不是跨程序互斥鎖；既有 arm1 主控還沒有訂閱或遵守它。現在不可同時啟動 `arm1.launch.py` 與 `arm2.launch.py` 接同一設備，也不可認為 B_EMPTY 已授權另一臂進場。

整合時須用一個中央區域仲裁器，將現有 arm1 的本機 B 鎖及 arm2 的本機 B 鎖改接同一個 acquire/release 介面。中央控制應依序：arm1 放置 → 視覺確認 B_OCCUPIED 及 arm1 離區 → 授權 arm2 並呼叫 B_READY → arm2 BUSY → arm2 回 Home_C 且視覺確認 B_EMPTY → 歸還權限。所有故障、訊號逾時與狀態不明維持鎖定；C_FULL 停止新增搬運。兩臂共用**唯一** bridge 及完整雙臂碰撞模型；現有 bridge 以全域 busy 一次接受一條軌跡，中央主控須序列排程，尚未實作雙臂並行軌跡；本次不預先宣稱已完成此多臂仲裁。

## 7. 測試與修改範圍

本次實際執行結果見 [驗證紀錄](B到C驗證紀錄.md) 與 [結果摘要](B到C驗證結果.json)。

```bash
PYTHONPATH=src/dual_arm_control python3 -m pytest src/dual_arm_control/test -q
bash scripts/test_arm2_integration.sh --scenario eight
bash scripts/test_arm2_integration.sh --scenario cancel
bash scripts/test_arm2_integration.sh --scenario release_failure
bash scripts/test_arm2_integration.sh --scenario verify_failure
bash scripts/test_arm2_integration.sh --scenario estop
bash scripts/test_arm2_integration.sh --scenario safety_loss
bash scripts/test_arm2_integration.sh --scenario stale_safety
bash scripts/test_arm2_integration.sh --scenario blocked_b
bash scripts/test_arm2_integration.sh --scenario count_failure
bash scripts/test_arm2_integration.sh --scenario collision
bash scripts/test_arm2_integration.sh --scenario recovery
bash scripts/test_arm2_integration.sh --scenario wrong_slot
bash scripts/test_arm2_integration.sh --scenario grid_failure
bash scripts/test_arm2_integration.sh --scenario grid_mismatch
```

整合測試使用獨立 ROS domain 164、真正 MoveIt、合成視覺與理想馬達，無序列埠。`eight` 完成 1..8 並拒絕第九件；各情境都會用同一 journal 重啟，確認 FULL/FAULT 與占用不會消失。每次報告位於 `/tmp/dual-arm-ros2/arm2-integration/<scenario>-<id>/result.json`；測試縮短模擬馬達時間與相機等待，不用作實機時間參數。

新增：兩份既有視覺 service 的原樣鏡像、arm2 狀態機、主控、設定驗證、mock、launch、兩份配置、測試與本文。共用 MoveIt client 增加明確四元數／可選控制器／逐格物件 ID／IK 預檢；bridge 增加急停及可選安全心跳檢查、GET_HOME 查詢；operator_console 增加 arm2 選項。arm1 的預設 topic、執行入口與舊配置保持相容。

可在軟體驗證：狀態順序、互鎖條件、滿格拒絕、確認 token、占用持久化、故障保留、規劃失敗處理、範例模型的 IK／路徑／碰撞及既有 protocol 5 解析。

仍需實機驗證：尺寸與座標／TCP 標定、四軸零位方向與限位、八格真實可達性、碰撞場景、負載與吸盤、相機延遲與誤判、Home_C homography／格號方向與圓片辨識、CM530 韌體燒錄與 READ/HOLD 回應、序列排程及到位容差、硬體急停效能，以及後續雙臂仲裁。模擬通過不能代替這些驗收。
