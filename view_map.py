'''
import cv2
import numpy as np
import yaml

filename = "rtabmap2"

# === Step 1: 讀取地圖與設定 ===
map_img = cv2.imread(f"{filename}.pgm", cv2.IMREAD_UNCHANGED)  # 原始灰階值
scale = 4
map_img = cv2.resize(map_img, (map_img.shape[1]*scale, map_img.shape[0]*scale), interpolation=cv2.INTER_NEAREST)
with open(f"{filename}.yaml", 'r') as f:
    map_info = yaml.safe_load(f)

resolution = map_info['resolution']
origin = map_info['origin']  # [x, y, theta]

# === Step 2: 建立彩色版本地圖 ===
height, width = map_img.shape
colored_map = np.zeros((height, width, 3), dtype=np.uint8)

# # 把未知區域（像素值==0）變成灰色
# colored_map[map_img == 0] = [127, 127, 127]      # 灰
# # 把可通行區域（255）變成白色
# colored_map[map_img == 255] = [255, 255, 255]    # 白
# # 把障礙區（其他值）變成黑色
# colored_map[(map_img > 0) & (map_img < 255)] = [0, 0, 0]  # 黑

# 依照 YAML 的 threshold 映射
colored_map[map_img > 200] = 255    # 空地 → 白
colored_map[(map_img <= 200) & (map_img >= 128)] = 128  # 未知 → 灰
colored_map[map_img < 128] = 0      # 障礙物 → 黑

# # === Step 3: 計算原點在圖片中的像素位置 ===
# origin_x, origin_y, _ = origin
# origin_px = int(-origin_x / resolution)
# origin_py = int(height + (origin_y / resolution))  # y 軸需反轉

# # 確保原點在圖片範圍內再畫
# if 0 <= origin_px < width and 0 <= origin_py < height:
#     cv2.drawMarker(
#         colored_map,
#         position=(origin_px, origin_py),
#         color=(0, 0, 255),  # 紅色
#         markerType=cv2.MARKER_CROSS,
#         markerSize=20,
#         thickness=2
#     )
#     cv2.putText(
#         colored_map,
#         'Origin',
#         (origin_px + 10, origin_py - 10),
#         fontFace=cv2.FONT_HERSHEY_SIMPLEX,
#         fontScale=0.5,
#         color=(0, 0, 255),
#         thickness=1
#     )

# 在圖上畫出原點（左下角是 origin）
# 將 meter 轉 pixel，並注意影像 y 軸向下
h, w = map_img.shape
resolution = 0.05
origin = [-3.72195, -4.92325]  # from yaml
origin_pixel_x = int(-origin[0] / resolution)
origin_pixel_y = int(h - (-origin[1] / resolution))

cv2.circle(colored_map, (origin_pixel_x, origin_pixel_y), 4, (255, 0, 0), -1)

# === Step 4: 顯示或儲存結果 ===
cv2.imshow("Colored Map with Origin", colored_map)
cv2.imwrite("map_colored.png", colored_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''
import cv2
import numpy as np
import math

# 讀取 PGM 地圖
scale = 4

map_img = cv2.imread("rtabmap2.pgm", cv2.IMREAD_UNCHANGED)

map_img = cv2.resize(map_img, (map_img.shape[1]*scale, map_img.shape[0]*scale), interpolation=cv2.INTER_NEAREST)

# 根據 YAML 參數進行反轉處理
negate = False  # from rtabmap2.yaml
occupied_thresh = 0.5
free_thresh = 0.196
resolution = 0.05  # 每格0.05m
origin = [-4.5366, -5.44831, 0.0]

# 建立彩色地圖
color_map = np.zeros((map_img.shape[0], map_img.shape[1], 3), dtype=np.uint8)

for y in range(map_img.shape[0]):
    for x in range(map_img.shape[1]):
        val = map_img[y, x]
        if val == 205:       # 未探索 (灰色)
            color_map[y, x] = (127, 127, 127)
        elif val >= 254:     # 空地 (白色)
            color_map[y, x] = (255, 255, 255)
        elif val <= 0:       # 障礙物 (黑色)
            color_map[y, x] = (0, 0, 0)

# 轉換世界座標到像素座標
def world_to_map(x, y):
    map_x = int((x - origin[0]) / resolution) * scale
    map_y = int((y - origin[1]) / resolution) * scale
    map_y = map_img.shape[0] - map_y  # y軸翻轉
    return map_x, map_y

# Robot pose (世界座標和角度)
robot_x, robot_y = 0.0, 0.0
robot_theta = 0.0  # 弧度，逆時針為正方向

# 畫出紅色箭頭表示機器人
arrow_start = world_to_map(robot_x, robot_y)
arrow_length = 20  # 箭頭長度（像素）
end_x = int(arrow_start[0] + arrow_length * math.cos(robot_theta))
end_y = int(arrow_start[1] - arrow_length * math.sin(robot_theta))  # 注意 y 軸方向

cv2.arrowedLine(color_map, arrow_start, (end_x, end_y), (0, 0, 255), 2, tipLength=0.4)

# 顯示
cv2.imshow("Map with Robot", color_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
