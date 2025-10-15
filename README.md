# **ROS2 專案 - 基於 ROS 2 的自主探索與建圖系統**

**ROS2 Project - ROS2-based Autonomous Exploration and Mapping**

## **1. 專案總覽 (Project Overview)**

本專案旨在為一台搭載 RGB-D 相機和 IMU 的室內移動機器人 (iRobot Create 2)，提供一套完整的、基於 ROS 2 Humble 的自主探索與建圖解決方案。

系統的核心功能分為兩個主要階段：
1.  **自主探索 (Autonomous Exploration)**：在完全未知的環境中，機器人能夠自主地探索空間，並使用 SLAM 技術即時建立一張二維佔用網格地圖。
2.  **自主導航 (Autonomous Navigation)**：在地圖建立完成後，使用者可以指定地圖上的一個目標點，機器人能夠自主地規劃路徑、避開動態與靜態障礙物，並準確地到達目的地。

本專案採用了分散式架構，將輕量級的硬體驅動與感測器融合任務放在機器人本身的 Mini PC 上，而將計算密集型的 SLAM 和導航決策任務放在一台功能更強大的遠端工作站上。

## **2. 主要功能 (Key Features)**

*   **即時同步定位與建圖 (SLAM)**：
    *   使用 **RTAB-Map** 作為核心 SLAM 演算法，融合 RGB-D 影像、IMU 以及輪式里程計數據，建立視覺特徵與幾何結構豐富的地圖。
    *   設定了合理的參數 (`Rtabmap/DetectionRate`) 以在確保建圖品質的同時，降低對 CPU 資源的消耗。

*   **高精度感測器融合里程計 (Sensor Fusion Odometry)**：
    *   使用 **`robot_localization` (EKF)** 套件，將低漂移的輪式里程計（平移準確）和高品質的 IMU（旋轉準確）進行融合。
    *   這產生了一個非常穩健、準確的 `/odom` topic，為上層的 SLAM 和導航提供了堅實的基礎，徹底解決了純視覺里程計在低紋理或空曠環境中會丟失的問題。

*   **全功能自主導航 (Autonomous Navigation)**：
    *   整合了完整的 **Nav2** 導航框架，包括路徑規劃器 (`planner_server`)、路徑控制器 (`controller_server`)、行為伺服器 (`behavior_server`) 和代價地圖 (`costmaps`)。
    *   對控制器 (DWB/MPPI) 和代價地圖的參數進行了精細調校，以適應差速驅動機器人的運動學特性，並解決了在終點附近震盪和路徑規劃失敗等常見問題。

*   **客製化的自主探索 (Customized Exploration)**：
    *   使用 **`explore-lite`** 作為探索策略的核心。
    *   已對其原始程式碼進行**修改**，將永久性的黑名單機制，改進為帶有「**耐心值 (patience value)**」的重試機制，極大地提升了在有限前向視野下探索的成功率和穩健性。
    *   實現了「**到達目標點後原地旋轉**」的行為，以彌補前向攝影機視野的不足，最大化每次移動所獲取的環境資訊。

*   **手動/自主控制無縫切換 (Control Arbitration)**：
    *   整合了 **`twist_mux`**，建立了一個安全的控制仲裁機制。
    *   允許高優先級的人工控制（例如，搖杆或鍵盤）隨時介入並覆寫 Nav2 的自主導航指令，鬆手後即可自動恢復。

## **3. 系統架構 (System Architecture)**

本系統採用分散式架構，由兩台電腦協同工作，透過專用的有線或 5GHz Wi-Fi 網路連接，並使用 **Chrony** 進行嚴格的時間同步。

*   **實體機器人端 (Mini PC)**：
    *   **職責**：執行所有與硬體直接交互的、輕量級的、需要即時反應的任務。
    *   **運行節點**：
        *   `create_robot` / `libcreate`：iRobot Create 2 底盤驅動，發佈 `/wheel/odom` 等。
        *   `realsense-ros`: RealSense D435i 驅動，發佈原始的 `/camera/...` 和 `/imu/data`。
        *   `robot_state_publisher`：發佈基於 `xacro` 合併後的完整 URDF 的靜態 TF。
        *   `robot_localization (ekf_node)`：融合輪式里程計和 IMU，發佈高品質的 `/odom` 和 `odom -> base_link` TF。
        *   （可選）`image_transport`：用於影像壓縮，以適應低頻寬網路。

*   **遠端工作站端 (開發電腦)**：
    *   **職責**：執行所有計算密集型的高階演算法。
    *   **運行節點**：
        *   `rtabmap_slam`：進行 SLAM 運算，建立並發佈 `/map` 和 `map -> odom` TF。
        *   `nav2_bringup`：啟動完整的 Nav2 導航堆疊。
        *   `explore_lite`：進行自主探索決策。
        *   `rviz2`：用於視覺化和互動。

## **4. 安裝與設定 (Installation & Setup)**

1.  **複製本倉庫**：
    ```bash
    git clone <your_repo_url>
    ```
2.  **初始化並更新子模組 (Submodules)**：
    本專案使用 Git Submodules 來管理外部依賴（如 `create_robot`, `m-explore-ros2`）。
    ```bash
    cd <your_repo>
    git submodule init
    git submodule update
    ```
3.  **安裝依賴**：
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
4.  **編譯工作空間**：
    ```bash
    colcon build --symlink-install
    ```
5.  **設定網路**：
    *   確保 Mini PC 和開發電腦連接到同一個區域網路。
    *   強烈建議為兩台電腦設定**靜態 IP** 或在路由器上進行 **DHCP 保留**。
    *   **必須**設定 **Chrony** 進行 NTP 時間同步，將開發電腦設為主機，Mini PC 設為從機。
    *   在兩台電腦的 `.bashrc` 中設定相同的 `export ROS_DOMAIN_ID=<some_number>`。

## **5. 使用方式 (Usage)**

1.  **Source 工作空間**：
    ```bash
    source install/setup.bash
    ```
2.  **在 Mini PC 上啟動底層驅動**：
    ```bash
    # (假設你有一個 mini_pc.launch.py)
    ros2 launch create_autonomy_bringup mini_pc.launch.py
    ```
3.  **在開發電腦上啟動自主探索**：
    ```bash
    ros2 launch create_autonomy_bringup explore.launch.py
    ```
4.  **儲存地圖**：
    當探索完成後，使用 `nav2_map_server` 提供的工具來儲存地圖。
    ```bash
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_office_map
    ```