# CanSat2026 Peiのブランチ
## 内容
* ImageProcessorクラスの作成
    * ARマーカー検出、有用画像判定、ゴール色検出、画像圧縮を担当

## ARマーカー検出
* etect_single_aruco_marker_for_capture_checkメソッド
    画像からARマーカーを検出し、検出結果やマーカーの位置などを含んだ辞書を返す
    result["key"]で欲しい値を参照
    keyはメソッド内を見て

## 有用画像判定

## ゴール色(赤)検出
* detect_red_ratioメソッド
    画像中の赤色領域を検出し、赤色の占有率を返す

## 画像圧縮
* compress_imageメソッド
    画像圧縮を行う
    現在はjpeg用になっているので、実際の画像形式が分かり次第修正

## その他
* load_imageメソッド
    指定された画像ファイルを読み込む
    これを実行するとimageに指定した画像が格納される
* save_imageメソッド
    imageをoutput_pathに保存する