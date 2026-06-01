import cv2

# 1. ArUcoマーカーの辞書を用意 (例として4x4の50個の辞書を使用)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_size = 200  # 生成する画像のサイズ（ピクセル）

# 2. 1から49までループ処理 (range(1, 50) は 1〜49 を意味します)
for marker_id in range(0, 50):
    
    # 各IDに対応するマーカー画像を生成
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    
    # フォルダ内に保存
    cv2.imwrite(f"aruco/aruco_marker_{marker_id}.png", marker_img)
    print(f"aruco/aruco_marker_{marker_id}.png を保存しました。")