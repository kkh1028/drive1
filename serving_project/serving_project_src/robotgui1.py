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

        # 현재 스크립트가 위치한 폴더 경로 가져오기
        self.base_path = os.path.dirname(os.path.abspath(__file__))

        self.label = QLabel(Form)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(130, 250, 301, 20))

        self.gif = QLabel(Form)
        self.gif.setObjectName(u"gif")
        self.gif.setGeometry(QRect(30, 10, 341, 221))

        self.retranslateUi(Form)

        # GIF 애니메이션 설정 (상대경로)
        gif_path = os.path.join(self.base_path, "111.gif")  
        if os.path.exists(gif_path):  # 파일 존재 여부 확인
            self.movie = QMovie(gif_path)
            self.gif.setMovie(self.movie)
            self.movie.start()
        else:
            print(f"Error: GIF 파일을 찾을 수 없습니다. ({gif_path})")

        # QLabel 크기에 맞게 GIF 조정
        self.gif.setScaledContents(True)

        QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.label.setText(QCoreApplication.translate("Form", u"주문 대기 중입니다.", None))


class MyWindow(QMainWindow, Ui_Form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MyWindow()
    win.show()
    sys.exit(app.exec_())
