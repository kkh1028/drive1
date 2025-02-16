import sys
import sqlite3
from datetime import datetime
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *
from ament_index_python.packages import get_package_share_directory


class ScrollingLabel(QLabel):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        # 텍스트 이동 범위 설정
        self.text_width = self.fontMetrics().width(self.text())
        self.label_width = self.width()

        # 애니메이션 설정
        self.animation = QPropertyAnimation(self, b"pos")
        self.animation.setDuration(5000)
        self.animation.setLoopCount(-1)
        self.animation.setStartValue(QPoint(self.label_width, 0))
        self.animation.setEndValue(QPoint(-self.text_width, 0))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # 텍스트 너비와 애니메이션 종료 지점 업데이트
        self.text_width = self.fontMetrics().width(self.text())
        self.label_width = self.width()
        self.animation.setStartValue(QPoint(self.label_width, 0))
        self.animation.setEndValue(QPoint(-self.text_width, 0))

    def start_scrolling(self):
        self.animation.start()


class Ui_Todayb(object):
    def __init__(self):
        # ROS2 패키지 경로에서 데이터베이스 경로 설정
        self.package_name = 'serving_project'  # 자신의 ROS2 패키지 이름으로 변경
        self.pkg_path = get_package_share_directory(self.package_name)
        self.db_path = f"{self.pkg_path}/db/restaurant.db"

    def setupUi(self, Today):
        if not Today.objectName():
            Today.setObjectName(u"Today")
        Today.resize(992, 366)

        # 테이블 위젯 설정
        self.tableWidget = QTableWidget(Today)
        self.tableWidget.setColumnCount(7)  # comp_1 ~ comp_5 열
        self.tableWidget.setRowCount(2)  # day와 comp들로 2행
        # 헤더 숨기기
        self.tableWidget.horizontalHeader().setHidden(True)  # 수평 헤더 숨기기
        self.tableWidget.verticalHeader().setHidden(True)  # 수직 헤더 숨기기
        
        self.tableWidget.setObjectName(u"tableWidget")
        self.tableWidget.setGeometry(QRect(120, 60, 731, 291))

        # 첫 번째 행 높이 설정
        self.tableWidget.setRowHeight(0, 35)  # 첫 번째 행 높이
        self.tableWidget.setRowHeight(1, 35 * 5)  # 두 번째 행 높이를 첫 번째 행의 5배로 설정
        self.tableWidget.setFixedSize(self.tableWidget.horizontalHeader().length(), self.tableWidget.verticalHeader().length())
        
        # 스크롤링 라벨 추가
        self.label = ScrollingLabel("🍞 천원의 아침밥 식당의 주간 빵식 메뉴를 확인하세요! 🍞", Today)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(120, 10, 750, 30))

        self.retranslateUi(Today)
        QMetaObject.connectSlotsByName(Today)

        # 데이터 로드
        self.load_menu_data()

    def get_db_data(self, query):
        """
        SQLite DB에서 쿼리를 실행하고 결과를 반환
        :param query: 실행할 SQL 쿼리
        :return: 쿼리 결과 리스트
        """
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute(query)
            results = cursor.fetchall()
            return results if results else []
        except sqlite3.Error as e:
            print(f"DB Error: {e}")
            return []
        finally:
            if conn:
                conn.close()

    def load_menu_data(self):
        """
        DB에서 데이터를 가져와 테이블에 표시
        """
        query = """
        SELECT day, comp_1, comp_2, comp_3, comp_4, comp_5
        FROM daily_menu
        WHERE day BETWEEN DATE('now', '-7 days') AND DATE('now')
        AND menu_id = 2
        """
        menu_data = self.get_db_data(query)

        if menu_data:
            for row, data in enumerate(menu_data):
                try:
                    # 날짜만 추출
                    day = datetime.strptime(data[0], "%Y-%m-%d %H:%M:%S").strftime("%Y-%m-%d")
                    comps = '\n'.join(str(comp) for comp in data[1:] if comp)  # comp_1 ~ comp_5 결합
                    self.tableWidget.setItem(0, row, QTableWidgetItem(day))  # 날짜
                    self.tableWidget.setItem(1, row, QTableWidgetItem(comps))  # 메뉴
                except ValueError:
                    print(f"날짜 형식 오류: {data[0]}")
        else:
            print("빵식 메뉴 데이터를 가져오는 데 실패했습니다.")

    def retranslateUi(self, Today):
        Today.setWindowTitle(QCoreApplication.translate("Today", u"주간 빵식 메뉴", None))
        self.label.setText(QCoreApplication.translate("Today", u"🍞 천원의 아침밥 식당의 주간 빵식 메뉴를 확인하세요! 🍞", None))


class TodayWindowb(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_Todayb()
        self.ui.setupUi(self)
        self.ui.label.start_scrolling()  # 전광판 효과 시작

    def closeEvent(self, event):
        print("창이 닫힙니다.")
        event.accept()  # 창 닫기를 허용


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TodayWindowb()
    window.show()
    sys.exit(app.exec_())

