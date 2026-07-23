"""CanSatの動作調整に使用するデフォルト値。

通信プロトコルや通信機器に関する設定は、このファイルでは管理しない。
各クラスのコメントには、その設定を使用する主なメソッドを記載する。
"""


class NavigationTargetConfig:
    """NavigationController.__init__()で設定し、方位・距離計算で使用する。"""

    TARGET_LATITUDE_DEG = 35.0         #目標緯度
    TARGET_LONGITUDE_DEG = 139.0       #目標経度


class CameraCaptureConfig:
    """前方カメラを使用するナビゲーション処理の撮影設定。

    使用メソッド:
        NavigationController.avoid_parachute()
        NavigationController._find_red_cone_in_view()
        navigation_goal.guide_to_red_cone()
        GoalNavigator.detect_ball()
    """

    WIDTH = 1920
    HEIGHT = 1080
    HDR = True
    TIMEOUT_MS = 2000


class NavigationPdConfig:
    """NavigationController._drive_pd_toward_heading()で使用するPDゲイン。"""

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

    MAX_ATTEMPTS = 3
    ACCEL_THRESHOLD_MPS2 = 7.0
    INITIAL_FLIP_PULSE_TIME_S = 1.0
    FLIP_PULSE_INCREMENT_S = 0.5
    REVERSE_STABILIZER_SPEED = 50.0
    ACTION_WAIT_S = 3.0


class FollowTargetConfig:
    """NavigationController.follow_target()で使用するGPS目標追従設定。

    GNSS_RECOVERY_*はNavigationController._move_for_gnss_recovery()でも
    使用する。
    """

    TIMEOUT_S = 120.0                                 #走行開始してから終了するまでのタイムアウト[s]
    GOAL_RADIUS_M = 3.0                               #ゴール到達範囲の半径、目標座標と現在地の距離の閾値[m]
    BASE_SPEED = 70.0                                 #目標のGNSS座標まで進む際のモーター出力の基準[%]
    LOOP_INTERVAL_S = 0.02                            #PD制御の周期、方位取得からスタック判定をこの周期で実行
    TARGET_UPDATE_INTERVAL_S = 1                      #GNSSの現在地から目標までの距離・方位を計算する周期[s]
    GNSS_LOST_GRACE_S = 6.0                           #GNSSが取得できなかった際に、直前の目標方位に従って走行を続ける時間[s]、これを超えると停止する。
    GNSS_RETRY_INTERVAL_S = 1.0                       #GNSSを取得できなかった際に、再取得を行う時間[s]
    GNSS_RECOVERY_FAILURE_LIMIT = 3                   #GNSS取得に何回失敗したら場所を移動するかのカウント数。
    GNSS_RECOVERY_MOVE_SPEED = 50.0
    GNSS_RECOVERY_MOVE_DURATION_S = 1.0


class StuckAvoidanceConfig:
    """NavigationController.avoid_stuck()と_run_stuck_escape()で使用する。"""

    ENABLED = True
    ACCEL_X_UPPER_MPS2 = 0.30
    ACCEL_Y_UPPER_MPS2 = 0.30
    DETECTION_DURATION_S = 2.0
    SAMPLE_INTERVAL_S = 0.05
    REVERSE_SPEED = 60.0
    REVERSE_DURATION_S = 1.0
    RIGHT_TURN_ANGLE_DEG = 90.0
    RIGHT_TURN_SPEED = 30.0
    RIGHT_TURN_TOLERANCE_DEG = 3.0
    RIGHT_TURN_TIMEOUT_S = 10.0
    FORWARD_SPEED = 60.0
    FORWARD_DURATION_S = 1.5


class ParachuteAvoidanceConfig:
    """NavigationController.avoid_parachute()で使用する回避設定。"""

    PURPLE_THRESHOLD = 0.01
    MOVE_SPEED = 100.0
    MOVE_DURATION_S = 3.0
    ROTATE_ANGLE_DEG = 90.0
    ROTATE_SPEED = 30.0
    ROTATE_TOLERANCE_DEG = 3.0
    ROTATE_TIMEOUT_S = 10.0
    MAX_ATTEMPTS = 10
    POST_ROTATION_PAUSE_S = 0.2


class RedConeConfig:
    """赤コーンの探索・誘導で使用する設定。

    使用メソッド・関数:
        NavigationController._find_red_cone_in_view()
        navigation_goal.guide_to_red_cone()
        GoalNavigator.detect_ball()
    """

    RED_THRESHOLD = 0.001
    GOAL_CENTER_THRESHOLD = 0.90
    RED_BLOCK_THRESHOLD = 0.005
    SCAN_ANGLE_DEG = 60.0
    CAMERA_FOV_DEG = 75.0
    MAX_SCAN_STEPS = 6
    MAX_GUIDANCE_STEPS = 30
    FORWARD_DURATION_S = 1.5
    FORWARD_DURATION_BY_RED_RATIO = (
        (0.30, 0.10),
        (0.25, 0.15),
        (0.20, 0.20),
        (0.10, 0.50),
        (0.05, 0.80),
    )
    FORWARD_SPEED = 60.0
    GOAL_FINAL_FORWARD_DURATION_S = 0.30
    ROTATE_SPEED = 30.0
    ROTATE_TOLERANCE_DEG = 3.0
    ROTATE_TIMEOUT_S = 10.0
    LOOP_INTERVAL_S = 0.10


class DriveControllerConfig:
    """DriveControllerのモータードライバ設定。

    PIN_*とPWM_FREQUENCY_HZ:
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
        DriveController.ramp_stop_forward()
    STABILIZER_*:
        DriveController.reverse_stabilizer()、flip()
    """

    PIN_STBY = 21
    PIN_PWMA = 12
    PIN_AIN1 = 8
    PIN_AIN2 = 7
    PIN_PWMB = 19
    PIN_BIN1 = 25
    PIN_BIN2 = 26

    PWM_FREQUENCY_HZ = 100
    SOFT_START_STEP_PERCENT = 5.0
    SOFT_START_INTERVAL_S = 0.03
    DIRECTION_CHANGE_DELAY_S = 0.1

    INVERT_LEFT_MOTOR = True
    INVERT_RIGHT_MOTOR = False
    LEFT_MOTOR_GAIN = 1.0
    RIGHT_MOTOR_GAIN = 1.0

    # ramp_stop_forward()で使用。GNSSロスト時とGNSS再取得移動後で共通。
    RAMP_STOP_STEPS = 100
    RAMP_STOP_INTERVAL_S = 0.03
    STABILIZER_SPEED = 100.0
    STABILIZER_PULSE_TIME_S = 0.5


class GoalNavigatorConfig:
    """GoalNavigator.detect_ball()とrider_forward()で使用する。"""

    RED_RATIO_THRESHOLD = RedConeConfig.RED_THRESHOLD
    RED_BLOCK_THRESHOLD = RedConeConfig.RED_BLOCK_THRESHOLD
    RED_SCAN_ANGLE_DEG = 30.0          #カメラでのゴール検知の探索角度
    RED_SCAN_STEPS = 12                #カメラでのゴール検知の探索ステップ、探索角度との積が360°になるように変更する
    CAMERA_FOV_DEG = RedConeConfig.CAMERA_FOV_DEG
    CENTER_RED_RATIO_THRESHOLD = 0.01  #赤検知の際の画面中央の赤色割合の閾値、これを超えると距離センサでの接近に移行
    ROTATION_SPEED = RedConeConfig.ROTATE_SPEED
    TURN_TOLERANCE_DEG = RedConeConfig.ROTATE_TOLERANCE_DEG
    ROTATION_TIMEOUT_S = None          #カメラでのゴール検知における旋回処理のタイムアウト
    CLOCKWISE = True                   #旋回方向、Trueが時計回り
    DISTANCE_SCAN_ANGLE_DEG = 10.0     #距離センサで旋回して探索するときに刻む角度
    DISTANCE_SCAN_STEPS = 36           #距離センサで探索するときのステップ数
    TARGET_DISTANCE_M = 2.0            #距離探索でボールを発見したと判断する距離。
    FORWARD_STOP_DISTANCE_M = 0.5      #距離センサで対象物に接近する際の最終停止距離の閾値、これよりも小さくなったら停止する
    FORWARD_SPEED = 60.0               #ボールに接近する際のモーターの速度[%]
    FOLLOW_FORWARD_DURATION_S = 1.0    #赤検知した際に、画面中央の赤色割合が閾値以下だった際に直進する時間
    LOOP_INTERVAL_S = 0.01             #距離センサの検知周期
    MEASUREMENT_PAUSE_S = 0.3          #

class ReleaseJudgeConfig:
    """judge.judge_release()で使用する放出判定設定。"""

    PRESSURE_MEASUREMENT_INTERVAL_S = 0.2
    PRESSURE_RELEASE_TIMEOUT_S = 60.0


class LandingJudgeConfig:
    """judge.judge_landing()で使用する着地判定設定。"""

    TARGET_ACCEL_MPS2 = 9.8
    TOLERANCE_MPS2 = 1.0
    CONTINUOUS_DURATION_S = 10.0
    MEASUREMENT_INTERVAL_S = 0.5       #測定周期


class FusingConfig:
    """fusing.fuse()とfuse_and_kick()で使用する溶断・キック設定。"""

    GPIO_PIN = 24
    FUSE_DURATION_S = 3.0              #溶断回路の起動時間[s]
    KICK_SPEED = 100.0                 #溶断後にモータを動作させる際の出力[%]
    KICK_PULSE_TIME_S = 0.1            #溶断後のモータ動作時間[s]
