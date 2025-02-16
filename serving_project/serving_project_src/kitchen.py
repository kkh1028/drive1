
import os
import re
import sys
import datetime
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from PySide2.QtWidgets import QApplication, QMainWindow
from functools import partial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
from ament_index_python.packages import get_package_share_directory
from threading import Thread
import sqlite3


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(788, 558)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.groupBox1 = QGroupBox(self.centralwidget)
        self.groupBox1.setObjectName(u"groupBox1")
        self.groupBox1.setGeometry(QRect(50, 80, 120, 80))
        self.label1 = QLabel(self.groupBox1)
        self.label1.setObjectName(u"label1")
        self.label1.setGeometry(QRect(10, 20, 111, 61))
        self.pushButton_1 = QPushButton(self.groupBox1)
        self.pushButton_1.setObjectName(u"pushButton_1")
        self.pushButton_1.setGeometry(QRect(0, 0, 121, 81))
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setObjectName(u"groupBox_2")
        self.groupBox_2.setGeometry(QRect(220, 80, 120, 80))
        self.label_2 = QLabel(self.groupBox_2)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(10, 20, 111, 61))
        self.pushButton_2 = QPushButton(self.groupBox_2)
        self.pushButton_2.setObjectName(u"pushButton_2")
        self.pushButton_2.setGeometry(QRect(0, 0, 121, 81))
        self.groupBox_3 = QGroupBox(self.centralwidget)
        self.groupBox_3.setObjectName(u"groupBox_3")
        self.groupBox_3.setGeometry(QRect(390, 80, 120, 80))
        self.label_3 = QLabel(self.groupBox_3)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(10, 20, 111, 61))
        self.pushButton_3 = QPushButton(self.groupBox_3)
        self.pushButton_3.setObjectName(u"pushButton_3")
        self.pushButton_3.setGeometry(QRect(0, 0, 121, 81))
        self.groupBox_5 = QGroupBox(self.centralwidget)
        self.groupBox_5.setObjectName(u"groupBox_5")
        self.groupBox_5.setGeometry(QRect(220, 200, 120, 80))
        self.label_5 = QLabel(self.groupBox_5)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(10, 20, 111, 61))
        self.pushButton_5 = QPushButton(self.groupBox_5)
        self.pushButton_5.setObjectName(u"pushButton_5")
        self.pushButton_5.setGeometry(QRect(0, 0, 121, 81))
        self.groupBox_4 = QGroupBox(self.centralwidget)
        self.groupBox_4.setObjectName(u"groupBox_4")
        self.groupBox_4.setGeometry(QRect(50, 200, 120, 80))
        self.label_4 = QLabel(self.groupBox_4)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(10, 20, 111, 61))
        self.pushButton_4 = QPushButton(self.groupBox_4)
        self.pushButton_4.setObjectName(u"pushButton_4")
        self.pushButton_4.setGeometry(QRect(0, 0, 121, 81))
        self.groupBox_6 = QGroupBox(self.centralwidget)
        self.groupBox_6.setObjectName(u"groupBox_6")
        self.groupBox_6.setGeometry(QRect(390, 200, 120, 80))
        self.label_6 = QLabel(self.groupBox_6)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(10, 20, 111, 61))
        self.pushButton_6 = QPushButton(self.groupBox_6)
        self.pushButton_6.setObjectName(u"pushButton_6")
        self.pushButton_6.setGeometry(QRect(0, 0, 121, 81))
        self.groupBox_7 = QGroupBox(self.centralwidget)
        self.groupBox_7.setObjectName(u"groupBox_7")
        self.groupBox_7.setGeometry(QRect(50, 320, 120, 80))
        self.label_7 = QLabel(self.groupBox_7)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(10, 20, 111, 61))
        self.pushButton_7 = QPushButton(self.groupBox_7)
        self.pushButton_7.setObjectName(u"pushButton_7")
        self.pushButton_7.setGeometry(QRect(0, 0, 121, 81))
        self.groupBox_9 = QGroupBox(self.centralwidget)
        self.groupBox_9.setObjectName(u"groupBox_9")
        self.groupBox_9.setGeometry(QRect(390, 320, 120, 80))
        self.label_9 = QLabel(self.groupBox_9)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setGeometry(QRect(10, 20, 111, 61))
        self.pushButton_9 = QPushButton(self.groupBox_9)
        self.pushButton_9.setObjectName(u"pushButton_9")
        self.pushButton_9.setGeometry(QRect(0, 0, 121, 81))
        self.groupBox_8 = QGroupBox(self.centralwidget)
        self.groupBox_8.setObjectName(u"groupBox_8")
        self.groupBox_8.setGeometry(QRect(220, 320, 120, 80))
        self.label_8 = QLabel(self.groupBox_8)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setGeometry(QRect(10, 20, 111, 61))
        self.pushButton_8 = QPushButton(self.groupBox_8)
        self.pushButton_8.setObjectName(u"pushButton_8")
        self.pushButton_8.setGeometry(QRect(0, 0, 121, 81))
        self.pushButton = QPushButton(self.centralwidget)
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.setGeometry(QRect(150, 420, 361, 41))

        self.pushButton_n = QPushButton(self.centralwidget)
        self.pushButton_n.setObjectName(u"pushButton")
        self.pushButton_n.setGeometry(QRect(50, 420, 80, 41))

        self.plainTextEdit = QPlainTextEdit(self.centralwidget)
        self.plainTextEdit.setObjectName(u"plainTextEdit")
        self.plainTextEdit.setGeometry(QRect(50, 470, 461, 41))

        self.groupBox_10 = QGroupBox(self.centralwidget)
        self.groupBox_10.setObjectName(u"groupBox_10")
        self.groupBox_10.setGeometry(QRect(550, 80, 181, 321))
        self.groupBox_13 = QGroupBox(self.groupBox_10)
        self.groupBox_13.setObjectName(u"groupBox_13")
        self.groupBox_13.setGeometry(QRect(20, 60, 141, 91))
        self.label = QLabel(self.groupBox_13)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 40, 61, 51))
        self.label_10 = QLabel(self.groupBox_13)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setGeometry(QRect(95, 70, 35, 16))
        self.groupBox_14 = QGroupBox(self.groupBox_10)
        self.groupBox_14.setObjectName(u"groupBox_14")
        self.groupBox_14.setGeometry(QRect(20, 200, 141, 91))
        self.label_11 = QLabel(self.groupBox_14)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setGeometry(QRect(95, 70, 35, 16))
        self.label_12 = QLabel(self.groupBox_14)
        self.label_12.setObjectName(u"label_12")
        self.label_12.setGeometry(QRect(20, 40, 61, 51))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 788, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.textEdit = QTextEdit(self.centralwidget)
        self.textEdit.setObjectName(u"textEdit")
        self.textEdit.setGeometry(QRect(50, 470, 461, 41))

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.groupBox1.setTitle(QCoreApplication.translate("MainWindow", u"table1", None))
        self.label1.setText(QCoreApplication.translate("MainWindow", u"", None))
        self.pushButton_1.setText("")
        self.groupBox_2.setTitle(QCoreApplication.translate("MainWindow", u"table2", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"", None))
        self.pushButton_2.setText("")
        self.groupBox_3.setTitle(QCoreApplication.translate("MainWindow", u"table3", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"", None))
        self.pushButton_3.setText("")
        self.groupBox_5.setTitle(QCoreApplication.translate("MainWindow", u"table5", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"", None))
        self.pushButton_5.setText("")
        self.groupBox_4.setTitle(QCoreApplication.translate("MainWindow", u"table4", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"", None))
        self.pushButton_4.setText("")
        self.groupBox_6.setTitle(QCoreApplication.translate("MainWindow", u"table6", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"", None))
        self.pushButton_6.setText("")
        self.groupBox_7.setTitle(QCoreApplication.translate("MainWindow", u"table7", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"", None))
        self.pushButton_7.setText("")
        self.groupBox_9.setTitle(QCoreApplication.translate("MainWindow", u"table9", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"", None))
        self.pushButton_9.setText("")
        self.groupBox_8.setTitle(QCoreApplication.translate("MainWindow", u"table8", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"", None))
        self.pushButton_8.setText("")
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"Serving", None))
        
        self.pushButton_n.setText(QCoreApplication.translate("MainWindow", u"거절", None))

        self.groupBox_10.setTitle(QCoreApplication.translate("MainWindow", u"LeftOver", None))
        self.groupBox_13.setTitle(QCoreApplication.translate("MainWindow", u"food", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.groupBox_14.setTitle(QCoreApplication.translate("MainWindow", u"bread", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.plainTextEdit.setPlainText(QCoreApplication.translate("MainWindow", u"", None))

    # retranslateUi


class DatabaseManager:
    def __init__(self, package_name='serving_project'):
        self.package_name = package_name
        self.pkg_path = get_package_share_directory(self.package_name)
        self.db_path = os.path.join(self.pkg_path, 'db', 'restaurant.db')
        self.conn = sqlite3.connect(self.db_path)
        self.conn.commit()

        # 초기값을 저장할 변수
        self.initial_food_remaining = 0
        self.initial_bread_remaining = 0

    def init_remaining_supply(self, day):
        """ 프로그램 실행 시 최초로 supply 테이블에서 초기값을 가져옴 """
        self.initial_food_remaining = self.get_remaining_supply(day, 1)
        self.initial_bread_remaining = self.get_remaining_supply(day, 2)

    def get_remaining_supply(self, day, menu_id):
        """ 특정 날짜와 menu_id에 해당하는 remaining 값을 조회 """
        cursor = self.conn.cursor()
        cursor.execute("""
            SELECT remaining FROM supply
            WHERE day = ? AND menu_id = ?
        """, (day, menu_id))
        result = cursor.fetchone()
        return result[0] if result else 0  # 값이 없으면 0 반환

    def update_remaining_count(self, menu_id, quantity):
        """ Serving 시 `label`(food)과 `label_12`(bread)의 값을 감소 """
        if menu_id == 1:
            self.initial_food_remaining = max(0, self.initial_food_remaining - quantity)
        elif menu_id == 2:
            self.initial_bread_remaining = max(0, self.initial_bread_remaining - quantity)


    def get_remaining_supply(self, day, menu_id):
        """ 특정 날짜와 menu_id에 해당하는 remaining 값을 조회 """
        cursor = self.conn.cursor()
        cursor.execute("""
            SELECT remaining FROM supply
            WHERE day = ? AND menu_id = ?
        """, (day, menu_id))
        result = cursor.fetchone()
        return result[0] if result else "N/A"  # 값이 없으면 "N/A" 반환


    def insert_order(self, order_id, time, processing, etc):
        cursor = self.conn.cursor()
        # 중복 체크
        cursor.execute("SELECT COUNT(*) FROM order_confirm WHERE id = ?", (order_id,))
        count = cursor.fetchone()[0]
        
        if count == 0:  # 중복이 아닐 때만 삽입
            cursor.execute('''
                INSERT INTO order_confirm (id, time, processing, etc)
                VALUES (?, ?, ?, ?)
            ''', (order_id, time, processing, etc))
            self.conn.commit()
        else:
            self.get_logger().warn(f"Order ID {order_id} already exists. Skipping insert.")


class ROS2Publisher(Node):
    def __init__(self, main_window):
        super().__init__('table_selection_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'selected_tables', 10)
        self.main_window = main_window
        self.db_manager = DatabaseManager()

        # ✅ 이미 처리한 주문 ID를 저장할 딕셔너리
        self.processed_orders = {}

        self.subscribers = []
        for i in range(1, 10):
            topic_name = f'/Table_order{i}/orders'
            subscriber = self.create_subscription(
                String,
                topic_name,
                partial(self.listener_callback, table_number=i),
                10
            )
            self.subscribers.append(subscriber)
        self.get_logger().info('ROS2 Publisher and Subscribers Initialized')

            
    def listener_callback(self, msg, table_number):
        self.get_logger().info(f'Received message for table {table_number}: {msg.data}')

        order_match = re.match(r'^(\d+):', msg.data)
        if order_match:
            order_id = order_match.group(1)
        else:
            order_id = "UNKNOWN"

        if order_id in self.processed_orders:
            self.get_logger().info(f"Skipping already processed order ID {order_id}")
            return

        self.processed_orders[order_id] = True

        if table_number not in self.main_window.table_orders:
            self.main_window.table_orders[table_number] = []

        # ✅ 주문 추가 여부 확인 로그 추가
        if msg.data not in self.main_window.table_orders[table_number]:
            self.get_logger().info(f"Adding order {msg.data} to table_orders[{table_number}]")
            self.main_window.table_orders[table_number].append(msg.data)

        # ✅ table_orders 확인 로그 추가
        self.get_logger().info(f"Updated table_orders[{table_number}]: {self.main_window.table_orders[table_number]}")

        food_match = re.search(r'Food:\s*(\d+)', msg.data)
        bread_match = re.search(r'Bread:\s*(\d+)', msg.data)

        existing_text = self.main_window.labels[table_number - 1].text()
        new_text = []
        if food_match:
            new_text.append(f"Food: {food_match.group(1)}")
        if bread_match:
            new_text.append(f"Bread: {bread_match.group(1)}")

        final_text = existing_text + "\n" + "\n".join(new_text) if existing_text else "\n".join(new_text)
        self.main_window.update_table_label(table_number, final_text)



    def publish_selected_buttons(self, selected_buttons):
        msg = Int32MultiArray()
        msg.data = selected_buttons
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
            
    def insert_order_to_db(self, table_number, processing, etc_text=""):
        order_ids = self.main_window.table_orders.get(table_number, [])

        if not order_ids:
            self.get_logger().warn(f"No orders found for table {table_number}")
            return

        current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        for order_id in order_ids[:]:
            try:
                order_id = int(order_id.split(":")[0])  # 🔥 `id`를 INTEGER로 변환
                processing_int = 1 if processing == 'y' else 0  # 🔥 `processing`을 INTEGER로 변환 (1: 'y', 0: 'n')

                cursor = self.db_manager.conn.cursor()
                cursor.execute("SELECT COUNT(*) FROM order_confirm WHERE id = ?", (order_id,))
                count = cursor.fetchone()[0]

                if count == 0:
                    cursor.execute(
                        "INSERT INTO order_confirm (id, time, processing, etc) VALUES (?, ?, ?, ?)",
                        (order_id, current_time, processing_int, etc_text)  # ✅ `processing_int` 사용
                    )
                    self.db_manager.conn.commit()
                    self.get_logger().info(f"Inserted into DB - Order ID: {order_id}, Time: {current_time}, Processing: {processing_int}, Etc: {etc_text}")

                    self.main_window.table_orders[table_number].remove(order_id)

                else:
                    self.get_logger().warn(f"Order ID {order_id} already exists. Skipping insert.")

            except ValueError as e:
                self.get_logger().warn(f"imsi_t_ValueError: Cannot convert order_id to int - {order_id}. Error: {e}")

            except sqlite3.Error as e:
                self.get_logger().error(f"Database error: {e}")

class MainWindow(QMainWindow):
    table_orders = {}
    def __init__(self, ros2_publisher):
        super().__init__()
        # UI 초기화
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # ROS2 퍼블리셔 연결
        self.ros2_publisher = ros2_publisher
        self.db_manager = DatabaseManager() 

        # 테이블 레이블 매핑
        self.labels = [
            self.ui.label1, self.ui.label_2, self.ui.label_3,
            self.ui.label_4, self.ui.label_5, self.ui.label_6,
            self.ui.label_7, self.ui.label_8, self.ui.label_9
        ]

        # 버튼 리스트 초기화
        self.buttons = [
            self.ui.pushButton_1, self.ui.pushButton_2, self.ui.pushButton_3,
            self.ui.pushButton_4, self.ui.pushButton_5, self.ui.pushButton_6,
            self.ui.pushButton_7, self.ui.pushButton_8, self.ui.pushButton_9
        ]

        # 각 버튼의 초기 상태 저장 (True: 투명, False: 반투명)
        self.button_states = [True] * len(self.buttons)
        # 선택된 버튼 번호를 저장할 리스트
        self.selected_buttons = []

        # 버튼 클릭 시 이벤트 연결
        for i, button in enumerate(self.buttons):
            button.clicked.connect(partial(self.toggle_button_style, i))

        # Serving 버튼 클릭 시 이벤트 연결
        self.ui.pushButton.clicked.connect(self.handle_serving_button)


        # 거절 버튼을 handle_reject_button()과 연결
        self.ui.pushButton_n.clicked.connect(self.handle_reject_button) 


        # 초기 스타일 설정
        self.set_initial_button_styles()

        # 창이 실행될 때 즉시 supply 데이터 조회 후 label 업데이트
        self.init_supply_labels()

    def set_initial_button_styles(self):
        """초기 버튼 스타일을 설정합니다 (투명 배경)."""
        for button in self.buttons:
            button.setStyleSheet("""
                QPushButton {
                    background-color: rgba(0, 0, 0, 0); /* 완전 투명 배경 */
                    border: none; /* 테두리 제거 */
                }
            """)

    def init_supply_labels(self):
        """ 프로그램 실행 시 `supply` 테이블에서 `label_10`, `label_11` 값을 가져오고 설정 """
        supply_date = "2025-01-31 00:00:00"
        self.db_manager.init_remaining_supply(supply_date)

        # label_10, label_11 초기화 (변경되지 않음)
        self.ui.label_10.setText('/ '+str(self.db_manager.initial_food_remaining))
        self.ui.label_11.setText('/ '+str(self.db_manager.initial_bread_remaining))

        # label, label_12 초기화 (Serving 시 줄어드는 값)
        self.ui.label.setText(str(self.db_manager.initial_food_remaining))
        self.ui.label_12.setText(str(self.db_manager.initial_bread_remaining))

        self.ui.label.setStyleSheet("font-size: 24pt; font-weight: bold;")
        self.ui.label_12.setStyleSheet("font-size: 24pt; font-weight: bold;")

    def toggle_button_style(self, index):
        """
        버튼 스타일을 토글하고 선택 상태를 업데이트합니다.
        index: 클릭된 버튼의 인덱스
        """
        button = self.buttons[index]

        # 상태에 따라 스타일 변경 및 선택 상태 업데이트
        if self.button_states[index]:
            # 반투명 배경으로 변경
            button.setStyleSheet("""
                QPushButton {
                    background-color: rgba(120, 10, 0, 50); /* 반투명 배경 */
                    border: none; /* 테두리 제거 */
                }
            """)
            self.selected_buttons.append(index + 1)  # 선택된 버튼 번호 추가
        else:
            # 투명 배경으로 변경
            button.setStyleSheet("""
                QPushButton {
                    background-color: rgba(0, 0, 0, 0); /* 완전 투명 배경 */
                    border: none; /* 테두리 제거 */
                }
            """)
            self.selected_buttons.remove(index + 1)  # 선택 해제된 버튼 번호 제거

        # 상태 토글
        self.button_states[index] = not self.button_states[index]


    def handle_serving_button(self):
        self.ros2_publisher.get_logger().info(f"서빙할 테이블 번호: {self.selected_buttons}")

        etc_text = self.ui.textEdit.toPlainText().strip() if len(self.selected_buttons) == 1 else ""

        # ✅ DB에 주문을 먼저 저장
        for table_number in self.selected_buttons:
            self.ros2_publisher.insert_order_to_db(table_number, 'y', etc_text)

        self.ros2_publisher.publish_selected_buttons(self.selected_buttons)

        food_total = 0
        bread_total = 0

        for table_number in self.selected_buttons:
            orders = self.ros2_publisher.main_window.table_orders.get(table_number, [])

            for msg_data in orders:
                food_match = re.search(r'Food:\s*(\d+)', msg_data)
                bread_match = re.search(r'Bread:\s*(\d+)', msg_data)

                food_total += int(food_match.group(1)) if food_match else 0
                bread_total += int(bread_match.group(1)) if bread_match else 0

            # ✅ 주문 완료 후 리스트 초기화 (이제 DB 저장 후 초기화)
            self.ros2_publisher.main_window.table_orders[table_number] = []

            # ✅ UI에서 주문 정보 제거
            self.update_table_label(table_number, "")

        # ✅ UI Label 값 한 번만 감소
        if food_total > 0:
            self.db_manager.update_remaining_count(1, food_total)
            self.ui.label.setText(str(self.db_manager.initial_food_remaining))

        if bread_total > 0:
            self.db_manager.update_remaining_count(2, bread_total)
            self.ui.label_12.setText(str(self.db_manager.initial_bread_remaining))

        for i, button in enumerate(self.buttons):
            if i + 1 in self.selected_buttons:
                button.setStyleSheet("""
                    QPushButton {
                        background-color: rgba(0, 0, 0, 0);
                        border: none;
                    }
                """)
                self.button_states[i] = True

        self.ui.textEdit.clear()
        self.selected_buttons.clear()


    def handle_reject_button(self):
        self.ros2_publisher.get_logger().info(f"거절할 테이블 번호: {self.selected_buttons}")
        
        # `textEdit`에 입력된 내용 가져오기
        etc_text = self.ui.textEdit.toPlainText().strip() if len(self.selected_buttons) == 1 else ""

        for table_number in self.selected_buttons:
            self.ros2_publisher.insert_order_to_db(table_number, 'n', etc_text)
            self.update_table_label(table_number, "")

        for i, button in enumerate(self.buttons):
            if i + 1 in self.selected_buttons:
                button.setStyleSheet("""
                    QPushButton {
                        background-color: rgba(0, 0, 0, 0);
                        border: none;
                    }
                """)
                self.button_states[i] = True

        self.selected_buttons.clear()

    def update_table_label(self, table_number, text):
        """
        특정 테이블의 레이블을 업데이트합니다.
        table_number: 테이블 번호 (1~9)
        text: 표시할 텍스트
        """
        if 1 <= table_number <= len(self.labels):
            self.labels[table_number - 1].setText(text)


def ros2_spin(ros2_publisher):
    """
    ROS2 노드를 백그라운드 스레드에서 실행합니다.
    """
    rclpy.spin(ros2_publisher)

def main():
    # ROS2 초기화
    rclpy.init()

    # MainWindow와 ROS2 노드 생성
    app = QApplication(sys.argv)
    ros2_publisher = ROS2Publisher(None)
    mainWindow = MainWindow(ros2_publisher)
    ros2_publisher.main_window = mainWindow  # MainWindow에 ROS2 노드 연결

    # ROS2를 별도의 스레드에서 실행
    ros2_thread = Thread(target=ros2_spin, args=(ros2_publisher,), daemon=True)
    ros2_thread.start()

    mainWindow.show()
    sys.exit(app.exec_())

    # ROS2 노드 종료
    ros2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()