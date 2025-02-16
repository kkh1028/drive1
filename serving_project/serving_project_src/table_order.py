import sys
import os
import sqlite3
import queue
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from serving_project.weeklyfood import TodayWindow
from serving_project.weeklybread import TodayWindowb

class Ui_Dialog(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node

        # DB와 UI 관련 초기화
        self.package_name = 'serving_project'
        self.pkg_path = get_package_share_directory(self.package_name)
        self.db_path = os.path.join(self.pkg_path, 'db', 'restaurant.db')
        
        self.food_num_total = 0
        self.bread_num_total = 0

        self.setupUi()

    def setupUi(self):
        ## 중요 ##
        namespace = self.node.get_namespace() 
        self.setWindowTitle(f"{namespace}")
        self.resize(820, 485)

        # 전광판 효과 추가
        self.scrolling_label()

        self.weeklyf = QPushButton(self)
        self.weeklyf.setObjectName(u"weeklyf")
        self.weeklyf.setGeometry(QRect(680, 50, 100, 30))  # 위치와 크기 지정
        self.weeklyf.setText(QCoreApplication.translate("Dialog", u"주간 급식메뉴", None))
        self.weeklyf.clicked.connect(self.show_weekly_menu)
        
        self.weeklyb = QPushButton(self)
        self.weeklyb.setObjectName(u"weeklyb")
        self.weeklyb.setGeometry(QRect(560, 50, 100, 30))  # 위치와 크기 지정
        self.weeklyb.setText(QCoreApplication.translate("Dialog", u"주간 빵식메뉴", None))
        self.weeklyb.clicked.connect(self.show_weekly_menub)

        self.buttonBox = QDialogButtonBox(self)
        self.buttonBox.setGeometry(QRect(580, 400, 170, 32))
        self.buttonBox.setStandardButtons(QDialogButtonBox.Ok | QDialogButtonBox.Reset)
        
        self.pick_list = QPlainTextEdit(self)
        self.pick_list.setGeometry(QRect(590, 120, 171, 201))
        
        self.label_food_image = QLabel(self)
        self.label_food_image.setGeometry(QRect(40, 60, 161, 121))
        self.load_menuimage(self.label_food_image, 'food1.jpg')

        self.label_bread_image = QLabel(self)
        self.label_bread_image.setGeometry(QRect(40, 270, 161, 121))
        self.load_menuimage(self.label_bread_image, 'bread1.jpg')

        self.label_food_description = QLabel(self)
        self.label_food_description.setGeometry(QRect(230, 70, 130, 120))
        self.label_food_description.setStyleSheet("font: 11pt 'Ubuntu Mono';")
        self.label_food_description.setWordWrap(True)

        self.label_bread_description = QLabel(self)
        self.label_bread_description.setGeometry(QRect(230, 280, 130, 120))
        self.label_bread_description.setStyleSheet("font: 11pt 'Ubuntu Mono';")
        self.label_bread_description.setWordWrap(True)

        self.food_spinbox = QSpinBox(self)
        self.food_spinbox.setGeometry(QRect(390, 90, 81, 61))

        self.bread_spinbox = QSpinBox(self)
        self.bread_spinbox.setGeometry(QRect(390, 300, 81, 61))

        self.food_button = QPushButton("추가", self)
        self.food_button.setGeometry(QRect(500, 110, 41, 25))
        self.food_button.clicked.connect(self.add_food_to_list)

        self.bread_button = QPushButton("추가", self)
        self.bread_button.setGeometry(QRect(500, 310, 41, 25))
        self.bread_button.clicked.connect(self.add_bread_to_list)

        self.label_pick = QLabel(self)
        self.label_pick.setObjectName(u"label_pick")
        self.label_pick.setGeometry(QRect(590, 90, 171, 20))
        self.label_pick.setText(QCoreApplication.translate("Dialog", u"주문 확인", None))

        self.label_ok = QLabel(self)
        self.label_ok.setObjectName(u"label_ok")
        self.label_ok.setGeometry(QRect(590, 340, 171, 20))
        self.label_ok.setText(QCoreApplication.translate("Dialog", u"주문중", None))

        self.buttonBox.accepted.connect(self.send_order)
        self.buttonBox.button(QDialogButtonBox.Reset).clicked.connect(self.reset_pick_list)

        self.load_descriptions()

    def load_menuimage(self, label, image_name):
        image_path = os.path.join(self.pkg_path, 'menu_images', image_name)
        if os.path.exists(image_path):
            pixmap = QPixmap(image_path)
            label.setPixmap(pixmap)
            label.setScaledContents(True)
        else:
            label.setText("Image not found")

    def load_descriptions(self):
        food_price = """
            SELECT price FROM menu
            WHERE id = 1;
        """
        bread_price = """
            SELECT price FROM menu
            WHERE id = 2;
        """
        food_query = """
            SELECT comp_1, comp_2, comp_3, comp_4, comp_5 FROM daily_menu 
            WHERE DATE(day) = '2025-01-31' AND menu_id = 1;
        """
        bread_query = """
            SELECT comp_1, comp_2, comp_3, comp_4, comp_5 FROM daily_menu 
            WHERE DATE(day) = '2025-01-31' AND menu_id = 2;
        """
        self.label_food_description.setText('급식 : ' + self.get_db_data(food_price) + '원\n\n' + self.get_db_data(food_query))
        self.label_bread_description.setText('빵식 : ' + self.get_db_data(bread_price) + '원\n\n' + self.get_db_data(bread_query))

    def get_db_data(self, query, single_value=False):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute(query)
            result = cursor.fetchone()
        # 단일 값 (가격)일 경우
            if single_value:
                return str(result[0])  # 숫자를 문자열로 변환

            # 여러 개의 컬럼 값 (재료 리스트)
            return ', '.join(map(str, filter(None, result)))
        except sqlite3.Error as e:
            return f"DB Error: {e}"
        finally:
            if conn:
                conn.close()

    def scrolling_label(self):
        """전광판 효과 라벨 생성 및 설정"""
        self.scrolling_label = QLabel(self)
        self.scrolling_label.setText("👨‍🍳 : 천원의 아침밥 식당의 주간 메뉴를 확인하세요")
        self.scrolling_label.setStyleSheet("font: 14pt 'Arial';")
        self.scrolling_label.setAlignment(Qt.AlignCenter)
        self.scrolling_label.setGeometry(0, 10, 1200, 40)  # 라벨 위치 및 크기 설정

        # 타이머 설정
        self.scrolling_label_timer = QTimer(self)
        self.scrolling_label_timer.timeout.connect(self.move_scrolling_label)
        self.scrolling_label_timer.start(30)  # 30ms마다 호출

        # 초기 이동 변수 설정
        self.scrolling_label_x = self.width()  # 라벨 시작 위치는 창의 오른쪽 끝에서 시작

    def move_scrolling_label(self):
        """라벨을 왼쪽으로 이동시키는 함수"""
        self.scrolling_label_x -= 4  # 라벨을 왼쪽으로 4픽셀씩 이동

        if self.scrolling_label_x < -self.scrolling_label.width():
            # 라벨이 화면 왼쪽 끝으로 나가면 다시 오른쪽 끝으로 이동
            self.scrolling_label_x = self.width()

        # 라벨 위치 업데이트
        self.scrolling_label.move(self.scrolling_label_x, 10)

    def show_weekly_menu(self):
        self.today_window = TodayWindow()
        self.today_window.show()
    
    def show_weekly_menub(self):
        self.today_window = TodayWindowb()
        self.today_window.show()

    def add_food_to_list(self):
        quantity = self.food_spinbox.value()
        self.food_num_total += quantity
        self.update_pick_list()

    def add_bread_to_list(self):
        quantity = self.bread_spinbox.value()
        self.bread_num_total += quantity
        self.update_pick_list()

    def update_pick_list(self):
        self.pick_list.clear()
        if self.food_num_total > 0:
            self.pick_list.appendPlainText(f"Food: {self.food_num_total}")
        if self.bread_num_total > 0:
            self.pick_list.appendPlainText(f"Bread: {self.bread_num_total}")

    def reset_pick_list(self):
        self.pick_list.clear()
        self.food_num_total = 0
        self.bread_num_total = 0


    def send_order(self):
        """
        Ok 버튼을 눌렀을 때, 입력된 데이터를 ROS2로 퍼블리시하고 SQLite DB에 저장합니다.
        하지만 주문 확인 리스트는 비우지 않습니다.
        """
        order = self.pick_list.toPlainText()
        print(f"[DEBUG] Order content: {order}")  # pick_list 내용 출력
        if not order:
            print("[DEBUG] Order is empty, skipping send_order")
            return

        namespace = self.node.get_namespace().strip("/")
        table_num = int(''.join(filter(str.isdigit, namespace.split("_")[-1])))

        # 현재 시간 기반 ID와 order_time 생성
        current_time = QDateTime.currentDateTime()
        order_id = f"{current_time.toString('yyMMddhhmmss')}"  # ID 앞에 테이블 번호 추가
        order_time = current_time.toString("yyyy-MM-dd hh:mm:ss")  # DATETIME 형식

        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            if self.food_num_total > 0:
                food_order_id = f"{table_num}1{order_id}"  # 테이블 번호 + '1' + order_id
                cursor.execute(
                    "INSERT INTO orders (id, order_time, table_num, menu_id, num) VALUES (?, ?, ?, ?, ?)",
                    (food_order_id, order_time, table_num, 1, self.food_num_total)
                )
                print(f"[INFO] Food Order Inserted: ID={food_order_id}, Time={order_time}, num={self.food_num_total}")

                # ROS2 메시지 생성 및 발행
                msg = String()
                msg.data = f"{food_order_id}: Food: {self.food_num_total}"
                self.node.queue.put(msg)
                print(f"[INFO] Message sent to ROS2: {msg.data}")

            if self.bread_num_total > 0:
                bread_order_id = f"{table_num}2{order_id}"  # 테이블 번호 + '2' + order_id
                cursor.execute(
                    "INSERT INTO orders (id, order_time, table_num, menu_id, num) VALUES (?, ?, ?, ?, ?)",
                    (bread_order_id, order_time, table_num, 2, self.bread_num_total)
                )
                print(f"[INFO] Bread Order Inserted: ID={bread_order_id}, Time={order_time}, num={self.bread_num_total}")

                # ROS2 메시지 생성 및 발행
                msg = String()
                msg.data = f"{bread_order_id}: Bread: {self.bread_num_total}"
                self.node.queue.put(msg)
                print(f"[INFO] Message sent to ROS2: {msg.data}")

            conn.commit()

        except sqlite3.Error as e:
            print(f"[ERROR] Database error: {e}")

        finally:
            conn.close()

        # ✅ 주문 완료 후 `label_ok`의 텍스트 변경
        self.label_ok.setText(QCoreApplication.translate("Dialog", u"주문 완료", None))
        self.node.get_logger().info("[INFO] Order sent and label updated to '주문 완료'.")


class ROS2Node(Node):
    def __init__(self):
        super().__init__('order_publisher')
        self.queue = queue.Queue()
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 주문은 반드시 전달되어야 하므로 신뢰성 보장
            history=HistoryPolicy.KEEP_LAST,  # 최신 주문만 유지
            depth=1  # 최신 1개 주문만 유지
        )
        self.publisher = self.create_publisher(String, 'orders', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_order)
        self.get_logger().info("[INFO] ROS2Node initialized and publisher created.")

    def publish_order(self):
        while not self.queue.empty():
            msg = self.queue.get()
            self.publisher.publish(msg)
            self.get_logger().info(f"[INFO] Published order message: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ROS2Node()
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    window = Ui_Dialog(node)
    window.show()  # QMainWindow에 적합한 show() 사용

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
