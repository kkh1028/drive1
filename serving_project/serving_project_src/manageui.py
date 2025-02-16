import sys
import os
import sqlite3
from PySide2.QtCore import (QCoreApplication, QMetaObject, QRect, Qt, QDateTime, QSize)
from PySide2.QtGui import (QBrush, QPen)
from PySide2.QtWidgets import (
    QApplication, QWidget, QPushButton, QTableWidget, QTableWidgetItem,
    QLabel, QComboBox, QCommandLinkButton, QProgressBar,
    QVBoxLayout, QFrame, QHeaderView
)
import matplotlib.ticker as ticker
# --- matplotlib 임베딩을 위한 설정 ---
import matplotlib
matplotlib.use("Agg")  # Qt와 호환되는 백엔드
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(782, 647)
        # 뒤로가기 버튼
        self.gobackb = QPushButton(Form)
        self.gobackb.setObjectName(u"gobackb")
        self.gobackb.setGeometry(QRect(340, 600, 89, 25))

        # 주문 내역표 (예: 급식/빵식 주문수 등)
        self.menuta = QTableWidget(Form)
        if (self.menuta.columnCount() < 2):
            self.menuta.setColumnCount(2)
        __qtablewidgetitem = QTableWidgetItem()
        self.menuta.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.menuta.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        if (self.menuta.rowCount() < 3):
            self.menuta.setRowCount(3)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.menuta.setVerticalHeaderItem(0, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.menuta.setVerticalHeaderItem(1, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        self.menuta.setVerticalHeaderItem(2, __qtablewidgetitem4)
        self.menuta.setObjectName(u"menuta")
        self.menuta.setGeometry(QRect(20, 100, 241, 121))

        # "일별 매출" 라벨
        self.dailysales = QLabel(Form)
        self.dailysales.setObjectName(u"dailysales")
        self.dailysales.setGeometry(QRect(20, 20, 71, 20))

        # 콤보박스 (예: 날짜 선택 등)
        self.chooseday = QComboBox(Form)
        self.chooseday.setObjectName(u"chooseday")
        self.chooseday.setGeometry(QRect(20, 40, 241, 25))
        self.load_dates_to_combobox()
        
        self.chooseday.currentTextChanged.connect(self.on_chooseday_changed)
        # commandLinkButton
        self.commandLinkButton = QCommandLinkButton(Form)
        self.commandLinkButton.setObjectName(u"commandLinkButton")
        self.commandLinkButton.setGeometry(QRect(180, 590, 131, 31))

        self.label = QLabel(Form)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(80, 260, 111, 17))

        self.label_2 = QLabel(Form)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(480, 70, 111, 20))

        self.label_3 = QLabel(Form)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(490, 320, 111, 20))

        # 원래 QwtPlot이었던 부분을 QFrame으로 대체
        self.weeklysel = QFrame(Form)
        self.weeklysel.setObjectName(u"weeklysel")
        self.weeklysel.setGeometry(QRect(300, 100, 430, 200))
        self.weeklysel.setFrameShape(QFrame.StyledPanel)
        self.weeklysel.setFrameShadow(QFrame.Raised)

        self.qwtPlot_2 = QFrame(Form)
        self.qwtPlot_2.setObjectName(u"qwtPlot_2")
        self.qwtPlot_2.setGeometry(QRect(320, 350, 400, 220))
        self.qwtPlot_2.setFrameShape(QFrame.StyledPanel)
        self.qwtPlot_2.setFrameShadow(QFrame.Raised)

        self.label_4 = QLabel(Form)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(90, 430, 91, 17))

        # 조리시간 평균표
        self.cookavg = QTableWidget(Form)
        self.cookavg.setColumnCount(2)  # 2열로 설정
        self.cookavg.setRowCount(1)  # 행을 1로 설정

        # 첫 번째 행, 첫 번째 열: 메뉴 1 데이터
        self.cookavg.setHorizontalHeaderItem(0, QTableWidgetItem("Menu 1"))
        # 첫 번째 행, 두 번째 열: 메뉴 2 데이터
        self.cookavg.setHorizontalHeaderItem(1, QTableWidgetItem("Menu 2"))
        
        # 행 이름 설정 (항목은 그대로 두기)
        self.cookavg.setVerticalHeaderItem(0, QTableWidgetItem("Cooking Time"))
        
        self.cookavg.setObjectName(u"cookavg")
        self.cookavg.setGeometry(QRect(20, 470, 241, 61))

        # 슬라이드바 제거: QTableWidget의 세로/가로 슬라이더 숨기기
        self.cookavg.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.cookavg.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # 프로그레스바: 급식/빵식 비율
        self.fbratiobar = QProgressBar(Form)
        self.fbratiobar.setObjectName(u"fbratiobar")
        self.fbratiobar.setGeometry(QRect(40, 290, 191, 51))
        self.fbratiobar.setValue(24)

        self.label_5 = QLabel(Form)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(40, 350, 67, 17))
        self.label_6 = QLabel(Form)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(200, 350, 31, 17))

        self.retranslateUi(Form)
        QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.gobackb.setText(QCoreApplication.translate("Form", u"돌아가기", None))
        ___qtablewidgetitem = self.menuta.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("Form", u"주문 수", None))
        ___qtablewidgetitem1 = self.menuta.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("Form", u"누적 금액", None))
        ___qtablewidgetitem2 = self.menuta.verticalHeaderItem(0)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("Form", u"급식", None))
        ___qtablewidgetitem3 = self.menuta.verticalHeaderItem(1)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("Form", u"빵식", None))
        ___qtablewidgetitem4 = self.menuta.verticalHeaderItem(2)
        ___qtablewidgetitem4.setText(QCoreApplication.translate("Form", u"합계", None))
        self.dailysales.setText(QCoreApplication.translate("Form", u"일별 매출", None))
        self.commandLinkButton.setText(QCoreApplication.translate("Form", u"식품나라 바로가기", None))
        self.label.setText(QCoreApplication.translate("Form", u"급/빵식 주문비율", None))
        self.label_2.setText(QCoreApplication.translate("Form", u"주간 매출 추이", None))
        self.label_3.setText(QCoreApplication.translate("Form", u"시간대별 주문 수", None))
        self.label_4.setText(QCoreApplication.translate("Form", u"평균 조리시간", None))
        ___qtablewidgetitem5 = self.cookavg.horizontalHeaderItem(0)
        ___qtablewidgetitem5.setText(QCoreApplication.translate("Form", u"급식", None))
        ___qtablewidgetitem6 = self.cookavg.horizontalHeaderItem(1)
        ___qtablewidgetitem6.setText(QCoreApplication.translate("Form", u"빵식", None))
        ___qtablewidgetitem7 = self.cookavg.verticalHeaderItem(0)
        ___qtablewidgetitem7.setText(QCoreApplication.translate("Form", u"조리시간", None))
        self.fbratiobar.setFormat("")
        self.label_5.setText(QCoreApplication.translate("Form", u"급식", None))
        self.label_6.setText(QCoreApplication.translate("Form", u"빵식", None))
        
    def load_dates_to_combobox(self):
        query = "SELECT DISTINCT DATE(day) FROM daily_menu WHERE menu_id=1;"
        dates = self.get_db_data(query)
        
        if dates != "No data found.":
            for date in dates:
                self.chooseday.addItem(date)

    def get_db_data(self, query, params=()):
        """
        주어진 SQL 쿼리를 실행하고 결과를 반환
        """
        try:
            conn = sqlite3.connect('/home/kyoheekim/restaurant_all.db')
            cursor = conn.cursor()

            # SQL 쿼리 실행 (파라미터 바인딩)
            cursor.execute(query, params)
            result = cursor.fetchall()  # fetchall()로 여러 행 가져오기

            if result:
                return [row[0] for row in result]  # 날짜 값만 반환
            else:
                return "No data found."  # 결과가 없을 경우 표시

        except sqlite3.Error as e:
            return f"DB Error: {e}"  # 오류 발생 시 메시지 반환

        finally:
            # 데이터베이스 연결 닫기
            if conn:
                conn.close() 
                
    def insert_avg_cooking_times(self):
        # SQLite 데이터베이스 연결
        conn = sqlite3.connect('/home/kyoheekim/restaurant_all.db')  # 데이터베이스 경로
        cursor = conn.cursor()

        try:
            # 메뉴별 평균 조리시간을 계산하는 쿼리 실행
            cursor.execute("""
                SELECT
                    o.menu_id,
                    AVG(julianday(oc.time) - julianday(o.order_time)) * 24 * 60 AS avg_cooking_time_minutes
                FROM
                    order_confirm oc
                JOIN
                    orders o ON oc.id = o.id
                GROUP BY
                    o.menu_id
            """)

            # 쿼리 결과 가져오기
            result = cursor.fetchall()
            print(result)

            # 결과에 따라 QTableWidget에 삽입
            for row, data in enumerate(result):
                menu_id, avg_time = data
                if menu_id == 1:
                    # 첫째 행에 menu_id=1의 평균 조리시간 삽입
                    self.cookavg.setItem(0, 0, QTableWidgetItem(str(round(avg_time, 2))))  # 평균값 소수점 2자리로 설정
                elif menu_id == 2:
                    # 둘째 행에 menu_id=2의 평균 조리시간 삽입
                    self.cookavg.setItem(0, 1, QTableWidgetItem(str(round(avg_time, 2))))  # 평균값 소수점 2자리로 설정

            # 커밋하여 변경사항 저장
            conn.commit()

        except sqlite3.Error as e:
            print(f"Error while calculating average cooking time: {e}")
        
        finally:
            # 연결 종료
            cursor.close()
            conn.close()   
                
    def load_menu_data_for_date(self, selected_date):
        # 선택한 날짜에 해당하는 메뉴의 주문 총량을 구하는 쿼리
        query = """
        SELECT SUM(o.num) AS total_orders
        FROM orders o
        JOIN daily_menu dm ON o.menu_id = dm.menu_id
        WHERE DATE(o.order_time) = DATE(?) AND DATE(dm.day) = DATE(?)
        AND o.menu_id = 1;
        """
        total_orders_menu_1 = self.get_db_data(query, (selected_date, selected_date))
        print(f"total_orders_menu_1: {total_orders_menu_1}")
        
        if total_orders_menu_1 != "No data found.":
            total_orders_menu_1 = int(total_orders_menu_1[0])  # 문자열을 정수로 변환
            self.menuta.setItem(0, 0, QTableWidgetItem(str(total_orders_menu_1)))  # 첫 번째 행, 첫 번째 열에 총 주문량 표시
        else:
            total_orders_menu_1 = 0
            self.menuta.setItem(0, 0, QTableWidgetItem("0"))  # 데이터가 없으면 0으로 표시
            
        # menu_id=2에 대한 총 주문량 구하기
        query = """
        SELECT SUM(o.num) AS total_orders
        FROM orders o
        JOIN daily_menu dm ON o.menu_id = dm.menu_id
        WHERE DATE(o.order_time) = DATE(?) AND DATE(dm.day) = DATE(?)
        AND o.menu_id = 2;
        """
        total_orders_menu_2 = self.get_db_data(query, (selected_date, selected_date))

        if total_orders_menu_2 != "No data found.":
            total_orders_menu_2 = int(total_orders_menu_2[0])  # 문자열을 정수로 변환
            self.menuta.setItem(1, 0, QTableWidgetItem(str(total_orders_menu_2)))  # 두 번째 행, 첫 번째 열에 총 주문량 표시
        else:
            total_orders_menu_2 = 0
            self.menuta.setItem(1, 0, QTableWidgetItem("0"))  # 데이터가 없으면 0으로 표시

        query = """
        SELECT SUM(o.num * m.price) AS total_sales
        FROM orders o
        JOIN daily_menu dm ON o.menu_id = dm.menu_id
        JOIN menu m ON o.menu_id = m.id
        WHERE DATE(o.order_time) = DATE(?) AND DATE(dm.day) = DATE(?)
        AND o.menu_id = 1;
        """
        total_sales_menu_1 = self.get_db_data(query, (selected_date, selected_date))

        if total_sales_menu_1 != "No data found.":
            total_sales_menu_1 = int(total_sales_menu_1[0])  # 문자열을 정수로 변환
            self.menuta.setItem(0, 1, QTableWidgetItem(str(total_sales_menu_1)))  # 첫 번째 행, 두 번째 열에 매출 표시
        else:
            total_sales_menu_1 = 0
            self.menuta.setItem(0, 1, QTableWidgetItem("0"))  # 데이터가 없으면 0으로 표시

        # menu_id=2에 대한 매출 계산
        query = """
        SELECT SUM(o.num * m.price) AS total_sales
        FROM orders o
        JOIN daily_menu dm ON o.menu_id = dm.menu_id
        JOIN menu m ON o.menu_id = m.id
        WHERE DATE(o.order_time) = DATE(?) AND DATE(dm.day) = DATE(?)
        AND o.menu_id = 2;
        """
        total_sales_menu_2 = self.get_db_data(query, (selected_date, selected_date))
        
        if total_sales_menu_2 != "No data found.":
            total_sales_menu_2 = int(total_sales_menu_2[0])  # 문자열을 정수로 변환
            self.menuta.setItem(1, 1, QTableWidgetItem(str(total_sales_menu_2)))  # 두 번째 행, 두 번째 열에 매출 표시
        else:
            total_sales_menu_2 = 0
            self.menuta.setItem(1, 1, QTableWidgetItem("0"))  # 데이터가 없으면 0으로 표시

        
        # 첫 번째 메뉴와 두 번째 메뉴의 주문 총량을 합산
        total_sum = total_orders_menu_1 + total_orders_menu_2  # 합산값 계산
        print(f"Total Sum: {total_sum}")
        
        # 합산값을 세 번째 행에 설정
        self.menuta.setItem(2, 0, QTableWidgetItem(str(total_sum)))  # 세 번째 행에 합산값 표시
        
        total_sum_sales = total_sales_menu_1 + total_sales_menu_2  # 매출 합산
        print(f"Total Sales Sum: {total_sum_sales}")
        self.menuta.setItem(2, 1, QTableWidgetItem(str(total_sum_sales)))  # 세 번째 행, 두 번째 열에 매출 합산값 표시
    
    def on_chooseday_changed(self):
        selected_date = self.chooseday.currentText()  # chooseday에서 선택한 날짜
        print("Selected date:", selected_date)  # 날짜 출력
        self.load_menu_data_for_date(selected_date)  # 선택된 날짜에 대해 데이터 로드
        
    def set_fbratiobar(self):
        """
        하나의 QProgressBar(fbratiobar)에
        menu_id=1(급식) vs menu_id=2(빵식)의 주문 비율을 표시
        """
        # 1) DB에서 메뉴1(급식) 주문 수량
        self.cur.execute("SELECT SUM(num) FROM orders WHERE menu_id=1")
        sum1 = self.cur.fetchone()[0] or 0

        # 2) DB에서 메뉴2(빵식) 주문 수량
        self.cur.execute("SELECT SUM(num) FROM orders WHERE menu_id=2")
        sum2 = self.cur.fetchone()[0] or 0

        total = sum1 + sum2
        ratio1 = 0.0
        ratio2 = 0.0

        # 3) 전체 대비 메뉴1, 메뉴2 비율 계산
        if total > 0:
            ratio1 = (sum1 / total) * 100  # 급식 비율
            ratio2 = (sum2 / total) * 100  # 빵식 비율

        # 4) QProgressBar 설정
        self.ui.fbratiobar.setRange(0, 100)
        self.ui.fbratiobar.setValue(int(ratio1))
        # 예: "급식 60.0% | 빵식 40.0%"
        self.ui.fbratiobar.setFormat(f"급식 {ratio1:.1f}% | 빵식 {ratio2:.1f}%")


###############################################################################
# 2) Matplotlib 임베딩을 위한 헬퍼 클래스 (MplCanvas)
###############################################################################
class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        super(MplCanvas, self).__init__(self.fig)


###############################################################################
# 3) 메인 로직: PySide2 위젯 + DB 쿼리 + matplotlib 그래프
###############################################################################
class AnalyForm(QWidget):
    DB_PATH = "/home/kyoheekim/restaurant_all.db"  # 실제 DB 경로로 수정
    
                
    def __init__(self):
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        # DB 연결
        db_path = os.getenv("DB_PATH", "/default/path/to/database.db")
        self.conn = sqlite3.connect(self.DB_PATH)
        self.cur = self.conn.cursor()
        self.ui.insert_avg_cooking_times()
        # 1) 프로그레스바: 급식 주문 비율
        self.set_fbratiobar()

        # 2) weeklysel에 최근 7일 매출(막대그래프)
        self.weekly_canvas = MplCanvas(self, width=5, height=3)
        self.embed_matplotlib(self.ui.weeklysel, self.weekly_canvas)
        self.draw_weekly_bar(self.weekly_canvas)

        # 3) qwtPlot_2에 시간대별 주문 건수(꺾은선 그래프)
        self.time_canvas = MplCanvas(self, width=5, height=3)
        self.embed_matplotlib(self.ui.qwtPlot_2, self.time_canvas)
        self.draw_time_line(self.time_canvas)

        # 4) menuta, cookavg 등은 필요 시 메소드로 데이터를 세팅 가능
        #    여기서는 간단히 빈 상태로 둡니다.

    def embed_matplotlib(self, parent_widget, canvas):
        """
        parent_widget(QFrame) 안에 QVBoxLayout을 만들어
        matplotlib FigureCanvas를 붙이는 헬퍼 함수
        """
        layout = QVBoxLayout(parent_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(canvas)
        parent_widget.setLayout(layout)

    def set_fbratiobar(self):
        """
        전체 주문량 중 급식(menu_id=1) 비율을 ProgressBar로 표시
        """
        self.cur.execute("SELECT SUM(num) FROM orders")
        total = self.cur.fetchone()[0] or 0

        self.cur.execute("SELECT SUM(num) FROM orders WHERE menu_id=1")
        rice_count = self.cur.fetchone()[0] or 0

        ratio = 0
        if total > 0:
            ratio = (rice_count / total) * 100

        self.ui.fbratiobar.setRange(0, 100)
        self.ui.fbratiobar.setValue(int(ratio))
        self.ui.fbratiobar.setFormat(f"{ratio:.2f}%")

    def draw_weekly_bar(self, canvas: MplCanvas):
        """
        최근 7일 (오늘 포함) 일자별 주문 건수 -> (주문수 1000)를 막대그래프로 표시
        x축 라벨: ["D-6","D-5","D-4","D-3","D-2","D-1","D-Day"]
        """
        # 1) 최근 7일 데이터 (오래된 날짜 -> 최신 날짜)
        query = """
        SELECT date(order_time) AS d,
            SUM(orders.num)*1000 AS amt
        FROM orders
        WHERE order_time >= date('now','-6 days')
        GROUP BY date(order_time)
        ORDER BY d
        """
        self.cur.execute(query)
        rows = self.cur.fetchall()  
        # 예: [("2025-01-10", 5000), ("2025-01-11", 8000), ...]
        # 이때 amt는 (주문 건수 합계) × 1000

        # 2) 최신 날짜(오늘)이 맨 뒤에 있으므로, reverse()로 인덱스 0이 오늘이 되도록
        rows.reverse()

        # 3) 최대 7일치 amounts (i=0 -> 오늘(D-Day), i=6 -> D-6)
        amounts = []
        for i in range(7):
            if i < len(rows):
                amt = rows[i][1] if rows[i][1] else 0
            else:
                amt = 0
            amounts.append(amt)

        # 4) x축 라벨
        x_labels = ["D-6","D-5","D-4","D-3","D-2","D-1","D-Day"]

        # 5) matplotlib로 그리기
        ax = canvas.axes
        ax.clear()

        # x좌표: range(7), y값: amounts
        ax.bar(range(7), amounts, color="skyblue")
        ax.set_xticks(range(7))
        ax.set_xticklabels(x_labels)
        ax.set_xlabel("기간")
        ax.set_ylabel("주문건수")

        canvas.draw()



    def draw_time_line(self, canvas: MplCanvas):
        """
        시간대별(0~23시) 주문 건수 -> 꺾은선 그래프
        """
        query = """
        SELECT strftime('%H', order_time) AS hh, COUNT(*) as cnt
        FROM orders
        GROUP BY strftime('%H', order_time)
        ORDER BY hh
        """
        self.cur.execute(query)
        rows = self.cur.fetchall()  # ex: [("00", 3), ("01", 5), ...]

        hour_counts = [0]*24
        for (h_str, cnt) in rows:
            h = int(h_str)
            hour_counts[h] = cnt

        x_vals = range(24)
        y_vals = hour_counts

        ax = canvas.axes
        ax.clear()
        ax.plot(x_vals, y_vals, marker='o', color='red')

        ax.set_xticks(range(24))
        ax.set_xticklabels([str(i) for i in range(24)],fontsize=6)
        
        ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))

        canvas.draw()

    def closeEvent(self, event):
        self.cur.close()
        self.conn.close()
        super().closeEvent(event)


###############################################################################
# 4) 메인 실행
###############################################################################
def main():
    app = QApplication(sys.argv)
    w = AnalyForm()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
