class NavigationTargetConfig:
    """NavigationController.__init__()で設定し、方位・距離計算で使用する。"""

    TARGET_LATITUDE_DEG = 35.0         #目標緯度
    TARGET_LONGITUDE_DEG = 139.0       #目標経度


class CameraCaptureConfig:
    """SensorManager.capture_front_frame()で使用する前方カメラ設定。"""

    WIDTH = 1920
    HEIGHT = 1080
    HDR = True
    TIMEOUT_MS = 2000


class NavigationPdConfig:
    """NavigationController.drive_toward_heading()で使用するPDゲイン。"""

    KP = 0.80
    KD = 0.05


class NavigationMotionConfig:
    """NavigationControllerの汎用的な直進・旋回メソッドで使用する。

    FOLLOW_FORWARD_*:
        NavigationController.follow_forward()
    ROTATE_*:
        NavigationController.rotate_by_angle()
    """

    FOLLOW_FORWARD_BASE_SPEED = 100.0
    FOLLOW_FORWARD_LOOP_INTERVAL_S = 0.02

    ROTATE_SPEED = 30.0
    ROTATE_TOLERANCE_DEG = 3.0
    ROTATE_TIMEOUT_S = 10.0
    ROTATE_LOOP_INTERVAL_S = 0.01


class PostureRestoreConfig:
    """NavigationController.restore_posture()で使用する姿勢復帰設定。"""

    MAX_ATTEMPTS = 3                                  #各軸の姿勢復帰動作を行う回数
    ACCEL_THRESHOLD_MPS2 = 7.0                        #姿勢判定の加速度境界値[m/s^2]
    INITIAL_FLIP_PULSE_TIME_S = 1.0                   #X軸の姿勢異常時にflip()を続ける時間
    FLIP_PULSE_INCREMENT_S = 0.5                      #flip()に失敗するたびに次の駆動時間をこの秒数だけ増加する
    REVERSE_STABILIZER_SPEED = 50.0                   #reverse_stabilizer()のモーター出力
    ACTION_WAIT_S = 3.0                               #各反転動作終了後に、機体の揺れが収まるまで待機する時間


class FollowTargetConfig:
    """NavigationController.follow_target()で使用するGPS目標追従設定。

    GNSS_RECOVERY_*はNavigationController._move_for_gnss_recovery()でも
    使用する。
    """

    TIMEOUT_S = 120.0                                 #走行開始してから終了するまでのタイムアウト[s]
    GOAL_RADIUS_M = 3.0                               #ゴール到達範囲の半径、目標座標と現在地の距離の閾値[m]
    BASE_SPEED = 70.0                                 #目標のGNSS座標まで進む際のモーター出力の基準[%]
    LOOP_INTERVAL_S = 0.02                            #PD制御の周期、方位取得から衝突判定をこの周期で実行
    TARGET_UPDATE_INTERVAL_S = 1                      #GNSSの現在地から目標までの距離・方位を計算する周期[s]
    GNSS_LOST_GRACE_S = 6.0                           #GNSSが取得できなかった際に、直前の目標方位に従って走行を続ける時間[s]、これを超えると停止する。
    GNSS_RETRY_INTERVAL_S = 1.0                       #GNSSを取得できなかった際に、再取得を行う時間[s]
    GNSS_RECOVERY_FAILURE_LIMIT = 3                   #GNSS取得に何回失敗したら場所を移動するかのカウント数。
    GNSS_RECOVERY_MOVE_SPEED = 50.0                   #GNSSを再取得するときに動く際のモーター出力[%]
    GNSS_RECOVERY_MOVE_DURATION_S = 1.0               #GNSSを再取得するときに動く秒数[s]


class StuckAvoidanceConfig:
    """NavigationController.avoid_stuck()と_run_stuck_escape()で使用する。"""

    ENABLED = True                                    #衝突検知機能のON/OFF
    SENSOR_FORWARD_AXIS = "x"                         #機体前方に対応するセンサー軸（x/y/z）
    SENSOR_FORWARD_SIGN = 1.0                         #センサー軸の正方向が機体前方なら1.0、負方向なら-1.0
    FORWARD_ACCEL_THRESHOLD_MPS2 = -9.0               #前方向加速度がこの値以下なら衝突候補
    FORWARD_JERK_THRESHOLD_MPS3 = -850.0              #前方向加速度の変化率がこの値以下なら衝突候補
    STARTUP_IGNORE_S = 1.5                            #走行開始直後の加速を衝突判定から除外する時間
    SAMPLE_INTERVAL_S = 0.02                          #衝突判定のサンプリング間隔
    STOP_RAMP_STEPS = 20                              #衝突検知後に前進出力を0%まで下げる段階数
    STOP_RAMP_INTERVAL_S = 0.02                       #衝突検知後の各減速段階の間隔[s]
    REVERSE_SPEED = 60.0                              #衝突検知後に後退する出力
    REVERSE_DURATION_S = 1.0                          #衝突検知後に後退する時間
    RIGHT_TURN_ANGLE_DEG = 90.0                       #後退後に右に旋回する目標角度
    RIGHT_TURN_SPEED = 30.0                           #右旋回時のモーター出力
    RIGHT_TURN_TOLERANCE_DEG = 3.0                    #旋回完了を許容する誤差[°]
    RIGHT_TURN_TIMEOUT_S = 10.0                       #右旋回を続ける最大時間


class ParachuteAvoidanceConfig:
    """NavigationController.avoid_parachute()で使用する回避設定。"""

    PURPLE_THRESHOLD = 0.01                           #紫の検知割合の閾値、これを超えるとパラ検知する
    MOVE_SPEED = 100.0                                #紫色が検出されずに前進する際のモータ-出力
    MOVE_DURATION_S = 3.0                             #前進する際の時間[s]
    ROTATE_ANGLE_DEG = 90.0                           #紫色を検出したとき、1回につき右へ旋回する目標角度
    ROTATE_SPEED = 30.0                               #旋回時のモーター出力
    ROTATE_TOLERANCE_DEG = 3.0                        #旋回時の許容誤差[°]
    ROTATE_TIMEOUT_S = 10.0                           #旋回を続けられる時間[s]
    MAX_ATTEMPTS = 10                                 #「撮影→紫色判定→紫色があれば右旋回」を繰り返す回数
    POST_ROTATION_PAUSE_S = 0.2                       #旋回後に次の画像をさs津栄するまでの待機時間


class RedConeConfig:
    """赤コーンの探索・誘導で使用する設定。

    使用メソッド・関数:
        navigation_goal._find_red_cone_in_view()
        navigation_goal.guide_to_red_cone()
    """

    RED_THRESHOLD = 0.00001                           #画像全体で赤を検出したと判定する最小割合
    GOAL_ANGLE_RED_THRESHOLD = 0.90                   #正面範囲の赤割合がこの値以上ならゴール到達と判定
    GOAL_ANGLE_MIN_DEG = -6.6                         #ゴール判定に使う正面範囲の左端角度[°]
    GOAL_ANGLE_MAX_DEG = 6.6                          #ゴール判定に使う正面範囲の右端角度[°]
    RED_COLUMN_THRESHOLD = 0.005                      #赤ピーク列として扱う最小赤割合
    RED_COLUMN_AVERAGE_WIDTH = 31                     #列ごとの赤割合を平滑化する横幅[pixel]
    SCAN_ANGLE_DEG = 60.0                             #赤を見失ったときの1回の探索旋回角度[°]
    HORIZONTAL_FOV_DEG = 66.0                         #前方カメラの水平視野角[°]
    MAX_SCAN_STEPS = 6                                #1回の誘導で赤を探索する最大撮影回数
    MAX_GUIDANCE_STEPS = 30                           #探索・旋回・前進を繰り返す最大回数
    FORWARD_DURATION_S = 1.5                          #赤割合テーブルに該当しない場合の前進時間[s]
    FORWARD_DURATION_BY_RED_RATIO = (                 #画像全体の赤割合に応じた前進時間[(割合, 秒)]
        (0.30, 0.10),
        (0.25, 0.15),
        (0.20, 0.20),
        (0.10, 0.50),
        (0.05, 0.80),
    )
    FORWARD_SPEED = 60.0                              #前進時の基準モーター出力[%]
    GOAL_FINAL_FORWARD_DURATION_S = 0.30              #ゴール判定後に追加で前進する時間[s]
    ROTATE_SPEED = 30.0                               #探索・位置合わせ旋回時のモーター出力[%]
    ROTATE_TOLERANCE_DEG = 3.0                        #旋回完了とみなす角度誤差[°]
    ROTATE_TIMEOUT_S = 10.0                           #1回の旋回を続けられる最大時間[s]
    LOOP_INTERVAL_S = 0.10                            #方位制御しながら前進する制御周期[s]


class RedBallConfig:
    """赤ボール誘導で使用する設定。"""

    # 赤ボール検出
    SWITCH_RED_RATIO = 0.005                          #赤コーン誘導からボール認識へ切り替える画像内赤割合
    RED_COLUMN_THRESHOLD = 0.005                      #サイズ候補抽出で赤領域列として扱う最小赤割合
    RED_COLUMN_AVERAGE_WIDTH = 31                     #サイズ候補抽出で列の赤割合を平滑化する横幅[pixel]
    HORIZONTAL_FOV_DEG = 66.0                         #前方カメラの水平視野角[°]

    # 中央合わせとターゲットロック
    CAMERA_LATERAL_OFFSET_M = 0.025                    #進行方向基準で右側（正面から見て左側）へのカメラずれ[m]
    RED_BALL_RADIUS_M = 0.10                           #中央合わせ補正に使う赤ボール半径[m]
    MAX_CENTERING_STEPS = 30                          #撮影と微旋回による中央合わせの最大回数
    CENTERING_TOLERANCE_DEG = 3.0                     #ボールが中央に合ったとみなす角度誤差[°]
    CENTERING_ROTATE_TOLERANCE_DEG = 3.0              #中央合わせ旋回の完了許容誤差[°]
    CENTERING_ROTATE_SPEED = 25.0                     #中央合わせ旋回時のモーター出力[%]
    CENTERING_TURN_GAIN = 0.1                         #小角度の中央合わせで旋回角へ掛ける補正倍率
    CENTERING_FULL_GAIN_ANGLE_DEG = 10.0              #大角度用の旋回補正倍率へ切り替える角度[°]
    CENTERING_LARGE_ANGLE_TURN_GAIN = 0.9             #大角度の中央合わせ・隣接球旋回へ掛ける補正倍率
    CENTERING_TARGET_LOCK_POSITION_SCALE_PX = 180.0   #位置類似度が約0.37まで下がる横ずれ[pixel]
    CENTERING_TARGET_LOCK_POSITION_WEIGHT = 1.0       #ロックスコアでx座標の近さへ掛ける重み
    CENTERING_TARGET_LOCK_SIZE_WEIGHT = 2.0           #ロックスコアで前回より小さくない候補を優先する重み
    ROTATE_TIMEOUT_S = 10.0                           #中央合わせ・隣接球旋回の最大継続時間[s]

    # 距離センサを使う接近
    TARGET_DISTANCE_M = 0.80                           #ボール表面までの停止目標距離[m]
    DISTANCE_TOLERANCE_M = 0.05                       #停止目標距離に対する固定許容誤差[m]
    REVERSE_SPEED = 40.0                              #ボールへ近づきすぎた場合の後退出力[%]
    REVERSE_DURATION_S = 0.12                         #ボールへ近づきすぎた場合の1回の後退時間[s]
    MAX_APPROACH_STEPS = 40                           #中央合わせ・測距・前後進を繰り返す最大回数
    CONE_FORWARD_DURATION_BY_RED_RATIO = (            #ボール認識へ切り替える前の赤割合別前進時間[(割合, 秒)]
        (0.005, 0.30),
        (0.003, 0.50),
        (0.002, 0.80),
        (0.001, 1.20),
        (0.0005, 1.40),
    )
    FORWARD_DURATION_S = 0.10                         #距離テーブルに該当しない場合の微前進時間[s]
    FORWARD_DURATION_BY_DISTANCE_ERROR_M = (          #目標までの残り距離に応じた前進時間[(距離差[m], 秒)]
        (1.6, 1.00),
        (1.2, 0.80),
        (0.8, 0.60),
        (0.6, 0.50),
        (0.4, 0.30),
        (0.3, 0.20),
        (0.2, 0.10),
        (0.1, 0.05),

    )

    # スクエアゾーン誘導
    ADJACENT_MIN_ANGLE_DEG = 15                       #正面の球を除外して隣接球とみなす最小角度[°]
    FARTHEST_MIN_SIZE_RATIO_TO_LARGEST = 0.35         #遠方選択で最大候補に対して許容する最小直径比
    INITIAL_SIDE_TURN_ANGLE_DEG = 40.0                #初回ボールが左右寄りだった場合の事前旋回角度[°]
    FINAL_TARGET_DISTANCE_M = 0.20                    #終了判定後に正面のボールへ近づく距離[m]
    CENTER_OF_ZONE_REPEAT_COUNT = 1                   #中心誘導で①～⑤を初回後に繰り返す回数
    CENTER_OF_ZONE_GOAL_DISTANCE_M = 0.40             #中心誘導の最終サイクルで目標にする距離[m]
    CENTER_OF_ZONE_OPPOSITE_TURN_ANGLE_DEG = 60.0     #非対角判定後に逆方向へ旋回する角度[°]
    MAX_SQUARE_TARGETS = 6                            #隣のボールへ向き直して接近する最大回数


class DriveControllerConfig:
    """DriveControllerのモータードライバ設定。

    PWM_FREQUENCY_HZ:
        DriveController._setup()
    INVERT_*:
        DriveController.__init__()、set_motor_inversion()、
        _motor_direction_pins()
    *_MOTOR_GAIN:
        DriveController.__init__()、set_motor_gain()、_set_duty_cycles()
    DIRECTION_CHANGE_DELAY_S:
        DriveController._prepare_motion()
    SOFT_START_*:
        DriveController._soft_start()
    RAMP_STOP_*:
        DriveController.ramp_stop_forward()、NavigationController.follow_forward()
    STABILIZER_*:
        DriveController.reverse_stabilizer()、flip()
    PD_FORWARD_SPEED:
        NavigationController.drive_toward_heading()のデフォルト基準出力
    """

    PWM_FREQUENCY_HZ = 100                            #PWM周期
    SOFT_START_STEP_PERCENT = 5.0                     #停止状態から始動するときの出力の増加割合[%]
    SOFT_START_INTERVAL_S = 0.03                      #出力を増加させる間隔[s]
    DIRECTION_CHANGE_DELAY_S = 0.1                    #前進・後退・旋回などの動作を開始する前の待機時間。モーターへの急な逆転負荷を減らす。

    INVERT_LEFT_MOTOR = True                          #左モーターの回転方向
    INVERT_RIGHT_MOTOR = False                        #右モーターの回転方向
    LEFT_MOTOR_GAIN = 1.0                             #左モーターの出力補正倍率
    RIGHT_MOTOR_GAIN = 1.0                            #右モーターの出力補正倍率

    PD_FORWARD_SPEED = 70.0                           #PD制御で直進する際のデフォルト基準出力[%]

    # ramp_stop_forward()とfollow_forward()の減速停止で使用。
    RAMP_STOP_STEPS = 10                              #現在の左右モーター出力を段階的に下げるためのステップ数。
    RAMP_STOP_INTERVAL_S = 0.01                       #出力を下げる際の各ステップ間の間隔
    STABILIZER_SPEED = 100.0                          #flip()とreverse_stabilizer()のデフォルト出力。
    STABILIZER_PULSE_TIME_S = 0.5                     #スタビライザー動作のデフォルト継続時間。


class ReleaseJudgeConfig:
    """judge.judge_release()で使用する放出判定設定。"""

    PRESSURE_MEASUREMENT_INTERVAL_S = 0.2
    PRESSURE_RELEASE_TIMEOUT_S = 60.0


class LandingJudgeConfig:
    """judge.judge_landing()で使用する着地判定設定。"""

    TARGET_ACCEL_MPS2 = 9.8                           #着地判定の9軸の閾値[m/s^2]
    TOLERANCE_MPS2 = 1.0                              #閾値からの許容誤差[m/s^2]
    CONTINUOUS_DURATION_S = 10.0                      #この秒数閾値範囲を記録したら着地判定
    MEASUREMENT_INTERVAL_S = 0.5                      #測定周期


class FusingConfig:
    """fusing.fuse()とfuse_and_kick()で使用する溶断・キック設定。"""

    FUSE_DURATION_S = 3.0                             #溶断回路の起動時間[s]
    KICK_SPEED = 100.0                                #溶断後にモーターを動作させる際の出力[%]
    KICK_PULSE_TIME_S = 0.1                           #溶断後のモーター動作時間[s]
