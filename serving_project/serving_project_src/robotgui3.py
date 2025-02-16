import sys
import os
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")

        Form.resize(400, 300)

        # 현재 스크립트 파일의 디렉토리 (상대경로 사용)
        self.base_path = os.path.dirname(os.path.abspath(__file__))

        # 안내 메시지 라벨
        self.label = QLabel(Form)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 240, 391, 20))

        # YESB 버튼
        self.YESB = QPushButton(Form)
        self.YESB.setObjectName(u"YESB")
        self.YESB.setGeometry(QRect(160, 270, 89, 25))
        self.YESB.setText("OK")
        self.YESB.clicked.connect(self.on_button_click)

        # GIF 애니메이션 설정
        self.gif = QLabel(Form)
        self.gif.setObjectName(u"gif")
        self.gif.setGeometry(QRect(30, 10, 341, 221))

        self.retranslateUi(Form)

        # 상대 경로로 GIF 설정
        self.movie = QMovie("/home/yigd/Downloads/444.GIF")  
        self.gif.setMovie(self.movie)  
        self.movie.start()

        # GIF 크기 자동 조정
        self.gif.setScaledContents(True)

        # 로봇 상태 라벨
        self.robot_label = QLabel(Form)
        self.robot_label.setObjectName(u"robot_label")
        self.robot_label.setGeometry(QRect(100, 200, 200, 20))

        # 진행 상태 바 (초기에는 숨김)
        self.progressBar = QProgressBar(Form)
        self.progressBar.setObjectName(u"progressBar")
        self.progressBar.setGeometry(QRect(140, 270, 118, 23))
        self.progressBar.setRange(0, 100)
        self.progressBar.setValue(0)
        self.progressBar.setVisible(False)  # 초기에는 숨김

        # 타이머 설정
        self.timer = QTimer(Form)
        self.timer.timeout.connect(self.updateProgress)

        # 진행 상태 초기값
        self.currentValue = 0  

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.label.setText(QCoreApplication.translate("Form", u"메뉴 도착! 주문하신 메뉴를 꺼낸 뒤 로봇을 보내주세요.", None))
        self.YESB.setText(QCoreApplication.translate("Form", u"OK", None))

    def on_button_click(self):
        """버튼 클릭 시 실행될 동작"""
        self.label.setText("로봇이 주방으로 돌아갑니다.")  # 텍스트 변경

        # GIF 변경 (상대경로 사용)
        new_movie = QMovie("/home/yigd/Downloads/555.gif")  
        self.gif.setMovie(new_movie)
        new_movie.start()


        # 버튼 숨기고 프로그래스 바 보이기
        self.YESB.setVisible(False)
        self.progressBar.setVisible(True)

        # 타이머 시작 (100ms마다 updateProgress 실행)
        self.timer.start(100)  

    def updateProgress(self):
        """무한 루프 진행 상태 바 업데이트"""
        self.currentValue += 0.5  # 5%씩 증가
        if self.currentValue > 100:
            self.currentValue = 0  # 100% 도달 시 다시 0%로 초기화
        self.progressBar.setValue(self.currentValue)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = QWidget()  
    ui = Ui_Form()
    ui.setupUi(win)
    win.show()
    sys.exit(app.exec_())
