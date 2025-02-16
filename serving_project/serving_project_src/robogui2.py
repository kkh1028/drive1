import sys
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(400, 300)
        
        # QProgressBar 설정
        self.progressBar = QProgressBar(Form)
        self.progressBar.setObjectName(u"progressBar")
        self.progressBar.setGeometry(QRect(140, 260, 118, 23))
        self.progressBar.setValue(0)  # 초기값을 0으로 설정

        # ProgressBar의 색상 변경 (노란색)
        self.progressBar.setStyleSheet("""
            QProgressBar {
                background-color: #e0e0e0;  # 배경 색 (회색)
                border-radius: 5px;  # 모서리 둥글게
            }
            QProgressBar::chunk {
                background-color: yellow;  # 진행 상태 색 (노란색)
                border-radius: 5px;  # 진행 상태 바도 둥글게
            }
        """)

        # QLabel 설정
        self.label = QLabel(Form)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(100, 230, 231, 20))

        # GIF 애니메이션 설정
        self.gif = QLabel(Form)
        self.gif.setObjectName(u"gif")
        self.gif.setGeometry(QRect(30, 10, 341, 221))

        self.retranslateUi(Form)
        
        # QMovie로 GIF 애니메이션 설정
        movie = QMovie("/home/yigd/Downloads/222.gif")  # GIF 파일 경로로 수정
        self.gif.setMovie(movie)  # QLabel에 QMovie 설정
        movie.start()  # GIF 애니메이션 시작

        # setScaledContents(True)로 QLabel 크기에 맞게 이미지 크기 조정
        self.gif.setScaledContents(True)

        # 타이머로 진행 상태 업데이트
        self.timer = QTimer(Form)
        self.timer.timeout.connect(self.updateProgress)
        self.timer.start(50)  # 10ms마다 updateProgress 호출 (더 짧은 시간 간격)

        self.currentValue = 0  # 진행 상태 값

        QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.label.setText(QCoreApplication.translate("Form", u"주문하신 메뉴를 배달 중입니다..", None))
        self.gif.setText(QCoreApplication.translate("Form", u"TextLabel", None))

    def updateProgress(self):
        """진행 상태 업데이트"""
        if self.currentValue < 100:
            self.currentValue += 0.3  # 진행 상태 값 증가 (1씩 증가)
            self.progressBar.setValue(self.currentValue)  # ProgressBar 값 업데이트
        else:
            self.timer.stop()  # 진행이 완료되면 타이머 멈춤


class MyWindow(QMainWindow, Ui_Form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MyWindow()
    win.show()
    sys.exit(app.exec_())
