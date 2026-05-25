class DriveController:
    """
    左右のタイヤ用モータを制御し、前進・後退・旋回・停止を行うクラス（最下層スタッフ）
    """
    def __init__(self):
        # 【素子の動作確認用】ここにモータのピン番号などの初期設定をあとで書きます
        print("DriveControllerが準備できました")

    def forward(self):
        # 【足の仕事】前進する命令
        print("モータを動かして：前進します")

    def stop(self):
        # 【足の仕事】停止する命令
        print("モータを止めて：停止します")