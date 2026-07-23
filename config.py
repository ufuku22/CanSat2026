"""CanSatの動作調整に使用するデフォルト値。

通信プロトコルや通信機器に関する設定は、このファイルでは管理しない。
各クラスのコメントには、その設定を使用する主なメソッドを記載する。
"""


class NavigationTargetConfig:
    """NavigationController.__init__()で設定し、方位・距離計算で使用する。"""

    TARGET_LATITUDE_DEG = 35.0
    TARGET_LONGITUDE_DEG = 139.0


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
    FOLLOW_FORWARD_STOP_RAMP_STEPS = 100
    FOLLOW_FORWARD_STOP_RAMP_INTERVAL_S = 0.03

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

    TIMEOUT_S = 120.0
    GOAL_RADIUS_M = 3.0
    BASE_SPEED = 70.0
    LOOP_INTERVAL_S = 0.02
    TARGET_UPDATE_INTERVAL_S = 1.0
    STOP_RAMP_STEPS = 100
    STOP_RAMP_INTERVAL_S = 0.02
    GNSS_LOST_GRACE_S = 6.0
    GNSS_RETRY_INTERVAL_S = 1.0
    GNSS_RECOVERY_FAILURE_LIMIT = 3
    GNSS_RECOVERY_MAX_MOVES = 3
    GNSS_RECOVERY_MOVE_SPEED = 50.0
    GNSS_RECOVERY_MOVE_DURATION_S = 1.0
    GNSS_RECOVERY_STOP_RAMP_STEPS = 10
    GNSS_RECOVERY_STOP_RAMP_INTERVAL_S = 0.01


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
    STOP_RAMP_STEPS = 8
    STOP_RAMP_INTERVAL_S = 0.01
    GOAL_FINAL_FORWARD_DURATION_S = 0.30
    ROTATE_SPEED = 30.0
    ROTATE_TOLERANCE_DEG = 3.0
    ROTATE_TIMEOUT_S = 10.0
    LOOP_INTERVAL_S = 0.10


class NavigationControllerConfig:
    """NavigationControllerが互換性を保ったまま参照する設定の一覧。

    値の用途は、参照先となる上記の機能別Configクラスに記載している。
    NavigationControllerはこのクラスを継承するため、既存の
    ``NavigationController.RED_CONE_*`` 形式も引き続き利用できる。
    """

    DEFAULT_TARGET_LATITUDE_DEG = NavigationTargetConfig.TARGET_LATITUDE_DEG
    DEFAULT_TARGET_LONGITUDE_DEG = NavigationTargetConfig.TARGET_LONGITUDE_DEG
    PD_KP = NavigationPdConfig.KP
    PD_KD = NavigationPdConfig.KD

    CAPTURE_WIDTH = CameraCaptureConfig.WIDTH
    CAPTURE_HEIGHT = CameraCaptureConfig.HEIGHT
    CAPTURE_HDR = CameraCaptureConfig.HDR
    CAPTURE_TIMEOUT_MS = CameraCaptureConfig.TIMEOUT_MS

    FOLLOW_TARGET_TIMEOUT_S = FollowTargetConfig.TIMEOUT_S
    FOLLOW_TARGET_GOAL_RADIUS_M = FollowTargetConfig.GOAL_RADIUS_M
    FOLLOW_TARGET_BASE_SPEED = FollowTargetConfig.BASE_SPEED
    FOLLOW_TARGET_LOOP_INTERVAL = FollowTargetConfig.LOOP_INTERVAL_S
    FOLLOW_TARGET_UPDATE_INTERVAL = FollowTargetConfig.TARGET_UPDATE_INTERVAL_S
    FOLLOW_TARGET_STOP_RAMP_STEPS = FollowTargetConfig.STOP_RAMP_STEPS
    FOLLOW_TARGET_STOP_RAMP_INTERVAL = FollowTargetConfig.STOP_RAMP_INTERVAL_S
    FOLLOW_TARGET_GNSS_LOST_GRACE_S = FollowTargetConfig.GNSS_LOST_GRACE_S
    FOLLOW_TARGET_GNSS_RETRY_INTERVAL = FollowTargetConfig.GNSS_RETRY_INTERVAL_S
    FOLLOW_TARGET_GNSS_RECOVERY_FAILURE_LIMIT = (
        FollowTargetConfig.GNSS_RECOVERY_FAILURE_LIMIT
    )
    FOLLOW_TARGET_GNSS_RECOVERY_MAX_MOVES = (
        FollowTargetConfig.GNSS_RECOVERY_MAX_MOVES
    )
    FOLLOW_TARGET_GNSS_RECOVERY_MOVE_SPEED = (
        FollowTargetConfig.GNSS_RECOVERY_MOVE_SPEED
    )
    FOLLOW_TARGET_GNSS_RECOVERY_MOVE_DURATION_S = (
        FollowTargetConfig.GNSS_RECOVERY_MOVE_DURATION_S
    )
    FOLLOW_TARGET_GNSS_RECOVERY_STOP_RAMP_STEPS = (
        FollowTargetConfig.GNSS_RECOVERY_STOP_RAMP_STEPS
    )
    FOLLOW_TARGET_GNSS_RECOVERY_STOP_RAMP_INTERVAL = (
        FollowTargetConfig.GNSS_RECOVERY_STOP_RAMP_INTERVAL_S
    )

    STUCK_AVOIDANCE_ENABLED = StuckAvoidanceConfig.ENABLED
    STUCK_ACCEL_X_UPPER_MPS2 = StuckAvoidanceConfig.ACCEL_X_UPPER_MPS2
    STUCK_ACCEL_Y_UPPER_MPS2 = StuckAvoidanceConfig.ACCEL_Y_UPPER_MPS2
    STUCK_DETECTION_DURATION_S = StuckAvoidanceConfig.DETECTION_DURATION_S
    STUCK_SAMPLE_INTERVAL_S = StuckAvoidanceConfig.SAMPLE_INTERVAL_S
    STUCK_REVERSE_SPEED = StuckAvoidanceConfig.REVERSE_SPEED
    STUCK_REVERSE_DURATION_S = StuckAvoidanceConfig.REVERSE_DURATION_S
    STUCK_RIGHT_TURN_ANGLE_DEG = StuckAvoidanceConfig.RIGHT_TURN_ANGLE_DEG
    STUCK_RIGHT_TURN_SPEED = StuckAvoidanceConfig.RIGHT_TURN_SPEED
    STUCK_RIGHT_TURN_TOLERANCE_DEG = (
        StuckAvoidanceConfig.RIGHT_TURN_TOLERANCE_DEG
    )
    STUCK_RIGHT_TURN_TIMEOUT_S = StuckAvoidanceConfig.RIGHT_TURN_TIMEOUT_S
    STUCK_FORWARD_SPEED = StuckAvoidanceConfig.FORWARD_SPEED
    STUCK_FORWARD_DURATION_S = StuckAvoidanceConfig.FORWARD_DURATION_S

    AVOID_PARACHUTE_PURPLE_THRESHOLD = (
        ParachuteAvoidanceConfig.PURPLE_THRESHOLD
    )
    AVOID_PARACHUTE_MOVE_SPEED = ParachuteAvoidanceConfig.MOVE_SPEED
    AVOID_PARACHUTE_MOVE_DURATION_S = ParachuteAvoidanceConfig.MOVE_DURATION_S
    AVOID_PARACHUTE_ROTATE_ANGLE_DEG = (
        ParachuteAvoidanceConfig.ROTATE_ANGLE_DEG
    )
    AVOID_PARACHUTE_ROTATE_SPEED = ParachuteAvoidanceConfig.ROTATE_SPEED
    AVOID_PARACHUTE_ROTATE_TOLERANCE_DEG = (
        ParachuteAvoidanceConfig.ROTATE_TOLERANCE_DEG
    )
    AVOID_PARACHUTE_ROTATE_TIMEOUT_S = (
        ParachuteAvoidanceConfig.ROTATE_TIMEOUT_S
    )
    AVOID_PARACHUTE_MAX_ATTEMPTS = ParachuteAvoidanceConfig.MAX_ATTEMPTS
    AVOID_PARACHUTE_POST_ROTATION_PAUSE_S = (
        ParachuteAvoidanceConfig.POST_ROTATION_PAUSE_S
    )

    RED_CONE_RED_THRESHOLD = RedConeConfig.RED_THRESHOLD
    RED_CONE_GOAL_CENTER_THRESHOLD = RedConeConfig.GOAL_CENTER_THRESHOLD
    RED_CONE_RED_BLOCK_THRESHOLD = RedConeConfig.RED_BLOCK_THRESHOLD
    RED_CONE_SCAN_ANGLE_DEG = RedConeConfig.SCAN_ANGLE_DEG
    RED_CONE_CAMERA_FOV_DEG = RedConeConfig.CAMERA_FOV_DEG
    RED_CONE_MAX_SCAN_STEPS = RedConeConfig.MAX_SCAN_STEPS
    RED_CONE_MAX_STEPS = RedConeConfig.MAX_GUIDANCE_STEPS
    RED_CONE_FORWARD_DURATION_S = RedConeConfig.FORWARD_DURATION_S
    RED_CONE_FORWARD_DURATION_BY_RED_RATIO = (
        RedConeConfig.FORWARD_DURATION_BY_RED_RATIO
    )
    RED_CONE_FORWARD_SPEED = RedConeConfig.FORWARD_SPEED
    RED_CONE_STOP_RAMP_STEPS = RedConeConfig.STOP_RAMP_STEPS
    RED_CONE_STOP_RAMP_INTERVAL = RedConeConfig.STOP_RAMP_INTERVAL_S
    RED_CONE_GOAL_FINAL_FORWARD_DURATION_S = (
        RedConeConfig.GOAL_FINAL_FORWARD_DURATION_S
    )
    RED_CONE_ROTATE_SPEED = RedConeConfig.ROTATE_SPEED
    RED_CONE_ROTATE_TOLERANCE_DEG = RedConeConfig.ROTATE_TOLERANCE_DEG
    RED_CONE_ROTATE_TIMEOUT_S = RedConeConfig.ROTATE_TIMEOUT_S
    RED_CONE_LOOP_INTERVAL = RedConeConfig.LOOP_INTERVAL_S

    RESTORE_POSTURE_MAX_ATTEMPTS = PostureRestoreConfig.MAX_ATTEMPTS
    RESTORE_POSTURE_ACCEL_THRESHOLD_MPS2 = (
        PostureRestoreConfig.ACCEL_THRESHOLD_MPS2
    )
    RESTORE_POSTURE_INITIAL_FLIP_PULSE_TIME_S = (
        PostureRestoreConfig.INITIAL_FLIP_PULSE_TIME_S
    )
    RESTORE_POSTURE_FLIP_PULSE_INCREMENT_S = (
        PostureRestoreConfig.FLIP_PULSE_INCREMENT_S
    )
    RESTORE_POSTURE_REVERSE_STABILIZER_SPEED = (
        PostureRestoreConfig.REVERSE_STABILIZER_SPEED
    )
    RESTORE_POSTURE_ACTION_WAIT_S = PostureRestoreConfig.ACTION_WAIT_S

    FOLLOW_FORWARD_DEFAULT_BASE_SPEED = (
        NavigationMotionConfig.FOLLOW_FORWARD_BASE_SPEED
    )
    FOLLOW_FORWARD_DEFAULT_LOOP_INTERVAL = (
        NavigationMotionConfig.FOLLOW_FORWARD_LOOP_INTERVAL_S
    )
    FOLLOW_FORWARD_DEFAULT_STOP_RAMP_STEPS = (
        NavigationMotionConfig.FOLLOW_FORWARD_STOP_RAMP_STEPS
    )
    FOLLOW_FORWARD_DEFAULT_STOP_RAMP_INTERVAL = (
        NavigationMotionConfig.FOLLOW_FORWARD_STOP_RAMP_INTERVAL_S
    )
    ROTATE_BY_ANGLE_DEFAULT_SPEED = NavigationMotionConfig.ROTATE_SPEED
    ROTATE_BY_ANGLE_DEFAULT_TOLERANCE_DEG = (
        NavigationMotionConfig.ROTATE_TOLERANCE_DEG
    )
    ROTATE_BY_ANGLE_DEFAULT_TIMEOUT_S = NavigationMotionConfig.ROTATE_TIMEOUT_S
    ROTATE_BY_ANGLE_DEFAULT_LOOP_INTERVAL = (
        NavigationMotionConfig.ROTATE_LOOP_INTERVAL_S
    )


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

    RAMP_STOP_STEPS = 100
    RAMP_STOP_INTERVAL_S = 0.03
    STABILIZER_SPEED = 100.0
    STABILIZER_PULSE_TIME_S = 0.5


class GoalNavigatorConfig:
    """GoalNavigator.detect_ball()とrider_forward()で使用する。"""

    RED_RATIO_THRESHOLD = RedConeConfig.RED_THRESHOLD
    RED_BLOCK_THRESHOLD = RedConeConfig.RED_BLOCK_THRESHOLD
    RED_SCAN_ANGLE_DEG = 30.0
    RED_SCAN_STEPS = 12
    CAMERA_FOV_DEG = RedConeConfig.CAMERA_FOV_DEG
    CENTER_RED_RATIO_THRESHOLD = 0.01
    ROTATION_SPEED = RedConeConfig.ROTATE_SPEED
    TURN_TOLERANCE_DEG = RedConeConfig.ROTATE_TOLERANCE_DEG
    ROTATION_TIMEOUT_S = None
    CLOCKWISE = True
    DISTANCE_SCAN_ANGLE_DEG = 10.0
    DISTANCE_SCAN_STEPS = 36
    TARGET_DISTANCE_M = 2.0
    FORWARD_STOP_DISTANCE_M = 0.5
    FORWARD_SPEED = 60.0
    FOLLOW_FORWARD_DURATION_S = 1.0
    LOOP_INTERVAL_S = 0.01
    MEASUREMENT_PAUSE_S = 0.3

    # GoalNavigatorが従来公開していた属性名。
    DEFAULT_RED_RATIO_THRESHOLD = RED_RATIO_THRESHOLD
    DEFAULT_RED_BLOCK_THRESHOLD = RED_BLOCK_THRESHOLD
    DEFAULT_RED_SCAN_ANGLE_DEG = RED_SCAN_ANGLE_DEG
    DEFAULT_RED_SCAN_STEPS = RED_SCAN_STEPS
    DEFAULT_CAMERA_FOV_DEG = CAMERA_FOV_DEG
    DEFAULT_CENTER_RED_RATIO_THRESHOLD = CENTER_RED_RATIO_THRESHOLD
    DEFAULT_ROTATION_SPEED = ROTATION_SPEED
    DEFAULT_TURN_TOLERANCE_DEG = TURN_TOLERANCE_DEG
    DEFAULT_DISTANCE_SCAN_ANGLE_DEG = DISTANCE_SCAN_ANGLE_DEG
    DEFAULT_DISTANCE_SCAN_STEPS = DISTANCE_SCAN_STEPS
    DEFAULT_TARGET_DISTANCE_M = TARGET_DISTANCE_M
    DEFAULT_FORWARD_STOP_DISTANCE_M = FORWARD_STOP_DISTANCE_M
    DEFAULT_FORWARD_SPEED = FORWARD_SPEED
    DEFAULT_FOLLOW_FORWARD_DURATION_S = FOLLOW_FORWARD_DURATION_S
    DEFAULT_LOOP_INTERVAL_S = LOOP_INTERVAL_S
    DEFAULT_MEASUREMENT_PAUSE_S = MEASUREMENT_PAUSE_S


class ReleaseJudgeConfig:
    """judge.judge_release()で使用する放出判定設定。"""

    PRESSURE_MEASUREMENT_INTERVAL_S = 0.2
    PRESSURE_RELEASE_TIMEOUT_S = 60.0


class LandingJudgeConfig:
    """judge.judge_landing()で使用する着地判定設定。"""

    TARGET_ACCEL_MPS2 = 9.8
    TOLERANCE_MPS2 = 1.0
    CONTINUOUS_DURATION_S = 10.0
    MEASUREMENT_INTERVAL_S = 0.5


class FusingConfig:
    """fusing.fuse()とfuse_and_kick()で使用する溶断・キック設定。"""

    GPIO_PIN = 24
    FUSE_DURATION_S = 3.0
    KICK_SPEED = 100.0
    KICK_PULSE_TIME_S = 0.1
