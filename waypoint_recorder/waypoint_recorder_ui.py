import sys
import os
import yaml
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import FollowWaypoints
from action_msgs.msg import GoalStatus

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QTableWidget, QTableWidgetItem,
    QHeaderView, QFileDialog, QMessageBox, QGroupBox, QStatusBar,
    QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QFont, QColor, QPalette


# ── ROS2 spin thread ──────────────────────────────────────────────────────────
class RosThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)


# ── Signal bridge (ROS → Qt) ──────────────────────────────────────────────────
class Signals(QObject):
    waypoint_received = pyqtSignal(float, float, float, float)  # x, y, qz, qw
    nav2_feedback     = pyqtSignal(int, int)                    # current, total
    nav2_result       = pyqtSignal(bool, str)                   # success, message


# ── ROS2 Node ─────────────────────────────────────────────────────────────────
class WaypointNode(Node):
    def __init__(self, signals: Signals):
        super().__init__('waypoint_recorder_ui')
        self.signals = signals
        self.recording = False

        self.sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_cb, 10
        )
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def _goal_cb(self, msg: PoseStamped):
        if not self.recording:
            return
        x  = msg.pose.position.x
        y  = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.signals.waypoint_received.emit(x, y, qz, qw)

    def send_waypoints(self, waypoints: list, map_frame: str = 'map'):
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.signals.nav2_result.emit(False, 'follow_waypoints 액션 서버에 연결할 수 없습니다.')
            return

        poses = []
        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp['position']['x']
            pose.pose.position.y = wp['position']['y']
            pose.pose.position.z = wp['position'].get('z', 0.0)
            pose.pose.orientation.x = wp['orientation'].get('x', 0.0)
            pose.pose.orientation.y = wp['orientation'].get('y', 0.0)
            pose.pose.orientation.z = wp['orientation']['z']
            pose.pose.orientation.w = wp['orientation']['w']
            poses.append(pose)

        goal = FollowWaypoints.Goal()
        goal.poses = poses
        total = len(poses)

        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self.signals.nav2_feedback.emit(
                fb.feedback.current_waypoint, total
            )
        )
        future.add_done_callback(lambda f: self._goal_response_cb(f, total))

    def _goal_response_cb(self, future, total):
        handle = future.result()
        if not handle.accepted:
            self.signals.nav2_result.emit(False, 'Goal이 거부되었습니다.')
            return
        handle.get_result_async().add_done_callback(
            lambda f: self._result_cb(f, total)
        )

    def _result_cb(self, future, total):
        status = future.result().status
        missed = future.result().result.missed_waypoints
        if status == GoalStatus.STATUS_SUCCEEDED:
            if missed:
                self.signals.nav2_result.emit(
                    True, f'완료 ({total - len(missed)}/{total} 성공, {len(missed)}개 누락)'
                )
            else:
                self.signals.nav2_result.emit(True, f'전체 {total}개 웨이포인트 완료!')
        else:
            self.signals.nav2_result.emit(False, f'Goal 실패 (status={status})')


# ── Main Window ───────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self, node: WaypointNode):
        super().__init__()
        self.node = node
        self.waypoints = []
        self.recording = False

        self.setWindowTitle('Waypoint Recorder')
        self.setMinimumSize(720, 580)
        self._apply_dark_theme()
        self._build_ui()
        self._update_buttons()

    # ── Theme ────────────────────────────────────────────────────────────────
    def _apply_dark_theme(self):
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #0f1117;
                color: #c8d0e0;
                font-family: 'Noto Sans KR', 'Malgun Gothic', sans-serif;
            }
            QGroupBox {
                border: 1px solid #1f2433;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 8px;
                font-size: 11px;
                color: #5a6480;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
                letter-spacing: 2px;
                text-transform: uppercase;
            }
            QLineEdit {
                background: #13161e;
                border: 1px solid #1f2433;
                border-radius: 6px;
                padding: 7px 10px;
                color: #c8d0e0;
                font-size: 13px;
                font-family: 'Consolas', 'Courier New', monospace;
            }
            QLineEdit:focus { border-color: #00e5ff; }
            QTableWidget {
                background: #13161e;
                border: 1px solid #1f2433;
                border-radius: 6px;
                gridline-color: #1a1e2a;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 12px;
            }
            QTableWidget::item { padding: 5px 8px; }
            QTableWidget::item:selected { background: #1a2a3a; color: #00e5ff; }
            QHeaderView::section {
                background: #0f1117;
                border: none;
                border-bottom: 1px solid #1f2433;
                padding: 6px 8px;
                font-size: 10px;
                letter-spacing: 1.5px;
                color: #5a6480;
                text-transform: uppercase;
            }
            QStatusBar {
                background: #0a0c12;
                border-top: 1px solid #1f2433;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 11px;
                color: #5a6480;
            }
            QScrollBar:vertical {
                background: #0f1117;
                width: 6px;
                border-radius: 3px;
            }
            QScrollBar::handle:vertical {
                background: #1f2433;
                border-radius: 3px;
            }
        """)

    def _btn_style(self, color_hex, alpha=0.15):
        r = int(color_hex[1:3], 16)
        g = int(color_hex[3:5], 16)
        b = int(color_hex[5:7], 16)
        return f"""
            QPushButton {{
                background: rgba({r},{g},{b},{int(alpha*255)});
                color: {color_hex};
                border: 1px solid rgba({r},{g},{b},100);
                border-radius: 6px;
                padding: 9px 20px;
                font-size: 12px;
                font-weight: 600;
                letter-spacing: 1px;
                font-family: 'Consolas', 'Courier New', monospace;
            }}
            QPushButton:hover {{
                background: rgba({r},{g},{b},{int(alpha*255)+40});
            }}
            QPushButton:pressed {{
                background: rgba({r},{g},{b},{int(alpha*255)+70});
            }}
            QPushButton:disabled {{
                background: rgba(30,34,48,255);
                color: #3a4055;
                border-color: #1f2433;
            }}
        """

    # ── UI Build ─────────────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(20, 20, 20, 12)
        root.setSpacing(14)

        # ── Header
        header = QHBoxLayout()
        title = QLabel('WAYPOINT RECORDER')
        title.setStyleSheet("font-size:18px; font-weight:700; color:#eef2ff; letter-spacing:2px; font-family:'Consolas','Courier New',monospace;")
        self.state_label = QLabel('IDLE')
        self.state_label.setStyleSheet("font-size:11px; font-weight:600; padding:4px 12px; border-radius:4px; font-family:'Consolas','Courier New',monospace; background:rgba(90,100,128,40); color:#5a6480; border:1px solid #1f2433;")
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.state_label)
        root.addLayout(header)

        # ── Save config
        cfg_group = QGroupBox('SAVE CONFIG')
        cfg_layout = QVBoxLayout(cfg_group)
        cfg_layout.setSpacing(8)

        row1 = QHBoxLayout()
        self.path_edit = QLineEdit(os.path.expanduser('~/waypoints'))
        self.path_edit.setPlaceholderText('저장 경로')
        browse_btn = QPushButton('찾기')
        browse_btn.setStyleSheet(self._btn_style('#5a6480'))
        browse_btn.setFixedWidth(60)
        browse_btn.clicked.connect(self._browse_path)
        row1.addWidget(QLabel('경로'))
        row1.addWidget(self.path_edit)
        row1.addWidget(browse_btn)

        row2 = QHBoxLayout()
        self.filename_edit = QLineEdit()
        self.filename_edit.setPlaceholderText('비워두면 타임스탬프 자동 생성')
        row2.addWidget(QLabel('파일명'))
        row2.addWidget(self.filename_edit)
        lbl = QLabel('.yaml')
        lbl.setStyleSheet("color:#5a6480; font-family:'Consolas','Courier New',monospace;")
        row2.addWidget(lbl)

        for lbl_widget in cfg_layout.parentWidget().findChildren(QLabel):
            lbl_widget.setFixedWidth(40)
            lbl_widget.setStyleSheet("color:#5a6480; font-size:11px;")

        cfg_layout.addLayout(row1)
        cfg_layout.addLayout(row2)
        root.addWidget(cfg_group)

        # ── Control buttons
        ctrl_group = QGroupBox('CONTROL')
        ctrl_layout = QHBoxLayout(ctrl_group)
        ctrl_layout.setSpacing(10)

        self.btn_start = QPushButton('▶  저장 시작')
        self.btn_stop  = QPushButton('■  저장 완료')
        self.btn_undo  = QPushButton('↩  Undo')
        self.btn_clear = QPushButton('✕  초기화')
        self.btn_load  = QPushButton('📂  YAML 불러오기')   # ← NEW
        self.btn_send  = QPushButton('⬆  Nav2 전송')

        self.btn_start.setStyleSheet(self._btn_style('#39ff7e'))
        self.btn_stop.setStyleSheet(self._btn_style('#ffd166'))
        self.btn_undo.setStyleSheet(self._btn_style('#ff3b5c'))
        self.btn_clear.setStyleSheet(self._btn_style('#5a6480'))
        self.btn_load.setStyleSheet(self._btn_style('#a78bfa'))   # ← NEW  보라색
        self.btn_send.setStyleSheet(self._btn_style('#00e5ff'))

        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_undo.clicked.connect(self._on_undo)
        self.btn_clear.clicked.connect(self._on_clear)
        self.btn_load.clicked.connect(self._on_load_yaml)         # ← NEW
        self.btn_send.clicked.connect(self._on_send)

        for btn in [self.btn_start, self.btn_stop, self.btn_undo,
                    self.btn_clear, self.btn_load, self.btn_send]:
            ctrl_layout.addWidget(btn)

        root.addWidget(ctrl_group)

        # ── Counter
        count_row = QHBoxLayout()
        count_row.setContentsMargins(4, 0, 4, 0)
        self.count_label = QLabel('0')
        self.count_label.setStyleSheet("font-size:40px; font-weight:700; color:#eef2ff; font-family:'Consolas','Courier New',monospace;")
        unit = QLabel(' waypoints')
        unit.setStyleSheet("font-size:13px; color:#5a6480; padding-bottom:4px;")
        count_row.addWidget(self.count_label)
        count_row.addWidget(unit)
        count_row.addStretch()

        self.nav2_status = QLabel('')
        self.nav2_status.setStyleSheet("font-size:11px; font-family:'Consolas','Courier New',monospace; color:#5a6480;")
        count_row.addWidget(self.nav2_status)
        root.addLayout(count_row)

        # ── Table
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(['#', 'X', 'Y', 'Yaw (qz)', 'W'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Fixed)
        self.table.setColumnWidth(0, 40)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.setStyleSheet(self.table.styleSheet() + "QTableWidget { alternate-background-color: #111520; }")
        root.addWidget(self.table)

        # ── Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self._set_status('준비됨. RViz에서 2D Goal Pose를 눌러 웨이포인트를 기록하세요.')

    # ── State helpers ─────────────────────────────────────────────────────────
    def _set_state(self, state: str):
        styles = {
            'IDLE':      ('IDLE',      'background:rgba(90,100,128,40);  color:#5a6480;  border:1px solid #1f2433;'),
            'RECORDING': ('RECORDING', 'background:rgba(57,255,126,25);  color:#39ff7e;  border:1px solid rgba(57,255,126,80);'),
            'DONE':      ('DONE',      'background:rgba(0,229,255,20);   color:#00e5ff;  border:1px solid rgba(0,229,255,80);'),
            'SENDING':   ('SENDING',   'background:rgba(255,107,53,20);  color:#ff6b35;  border:1px solid rgba(255,107,53,80);'),
            'LOADED':    ('LOADED',    'background:rgba(167,139,250,20); color:#a78bfa;  border:1px solid rgba(167,139,250,80);'),  # ← NEW
        }
        text, style = styles.get(state, styles['IDLE'])
        base = "font-size:11px; font-weight:600; padding:4px 12px; border-radius:4px; font-family:'Consolas','Courier New',monospace; "
        self.state_label.setText(text)
        self.state_label.setStyleSheet(base + style)

    def _set_status(self, msg: str):
        self.status_bar.showMessage(f'  {msg}')

    def _update_buttons(self):
        has_wp = len(self.waypoints) > 0
        self.btn_start.setEnabled(not self.recording)
        self.btn_stop.setEnabled(self.recording)
        self.btn_undo.setEnabled(has_wp and self.recording)
        self.btn_clear.setEnabled(has_wp and not self.recording)
        self.btn_load.setEnabled(not self.recording)               # ← NEW
        self.btn_send.setEnabled(has_wp and not self.recording)

    # ── Button handlers ───────────────────────────────────────────────────────
    def _on_start(self):
        self.recording = True
        self.node.recording = True
        self._set_state('RECORDING')
        self._set_status('녹화 중... RViz에서 2D Goal Pose를 찍어 웨이포인트를 추가하세요.')
        self._update_buttons()

    def _on_stop(self):
        self.recording = False
        self.node.recording = False
        self._set_state('DONE')
        self._save_yaml()
        self._update_buttons()

    def _on_undo(self):
        if not self.waypoints:
            return
        self.waypoints.pop()
        self.table.removeRow(self.table.rowCount() - 1)
        self.count_label.setText(str(len(self.waypoints)))
        self._set_status(f'마지막 웨이포인트 제거됨. 총 {len(self.waypoints)}개.')

    def _on_clear(self):
        reply = QMessageBox.question(
            self, '확인', '모든 웨이포인트를 삭제하시겠습니까?',
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.waypoints.clear()
            self.table.setRowCount(0)
            self.count_label.setText('0')
            self._set_state('IDLE')
            self._set_status('초기화되었습니다.')
            self._update_buttons()

    # ── NEW: YAML 불러오기 ────────────────────────────────────────────────────
    def _on_load_yaml(self):
        """파일 다이얼로그로 YAML을 선택하고 웨이포인트를 테이블에 로드."""
        filepath, _ = QFileDialog.getOpenFileName(
            self, 'YAML 파일 불러오기',
            self.path_edit.text(),
            'YAML Files (*.yaml *.yml);;All Files (*)'
        )
        if not filepath:
            return

        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            QMessageBox.critical(self, 'YAML 읽기 오류', str(e))
            return

        # ── 포맷 검증
        if not isinstance(data, dict) or 'waypoints' not in data:
            QMessageBox.critical(
                self, '포맷 오류',
                "'waypoints' 키가 없는 파일입니다.\n"
                "이 앱으로 저장한 YAML 파일을 선택해 주세요."
            )
            return

        raw_wps = data['waypoints']
        if not isinstance(raw_wps, list) or len(raw_wps) == 0:
            QMessageBox.warning(self, '경고', '웨이포인트 목록이 비어 있습니다.')
            return

        # ── 기존 데이터가 있으면 덮어쓸지 확인
        if self.waypoints:
            reply = QMessageBox.question(
                self, '확인',
                f'현재 {len(self.waypoints)}개의 웨이포인트가 있습니다.\n'
                '불러온 데이터로 교체하시겠습니까?',
                QMessageBox.Yes | QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return

        # ── 테이블 & 내부 데이터 초기화 후 로드
        self.waypoints.clear()
        self.table.setRowCount(0)

        errors = []
        for idx, wp in enumerate(raw_wps):
            try:
                pos = wp['position']
                ori = wp['orientation']
                x   = float(pos['x'])
                y   = float(pos['y'])
                qz  = float(ori['z'])
                qw  = float(ori['w'])
            except (KeyError, TypeError, ValueError) as e:
                errors.append(f'  #{ idx + 1 }: {e}')
                continue

            self.waypoints.append({
                'position':    {'x': round(x,  4), 'y': round(y,  4), 'z': float(pos.get('z', 0.0))},
                'orientation': {
                    'x': float(ori.get('x', 0.0)),
                    'y': float(ori.get('y', 0.0)),
                    'z': round(qz, 6),
                    'w': round(qw, 6),
                }
            })

            row = self.table.rowCount()
            self.table.insertRow(row)
            for col, val in enumerate([str(row + 1), f'{x:.4f}', f'{y:.4f}', f'{qz:.4f}', f'{qw:.4f}']):
                item = QTableWidgetItem(val)
                item.setTextAlignment(Qt.AlignCenter)
                if col == 0:
                    item.setForeground(QColor('#5a6480'))
                self.table.setItem(row, col, item)

        self.count_label.setText(str(len(self.waypoints)))
        self._set_state('LOADED')

        fname = os.path.basename(filepath)
        if errors:
            QMessageBox.warning(
                self, '일부 웨이포인트 오류',
                f'{len(self.waypoints)}개 로드 성공, {len(errors)}개 건너뜀:\n' + '\n'.join(errors)
            )
            self._set_status(f'[{fname}] {len(self.waypoints)}개 로드 (일부 오류)')
        else:
            self._set_status(f'[{fname}] {len(self.waypoints)}개 웨이포인트 로드 완료 → Nav2 전송 버튼으로 실행하세요.')

        self._update_buttons()

    def _on_send(self):
        if not self.waypoints:
            return
        self._set_state('SENDING')
        self.nav2_status.setText('전송 중...')
        self.btn_send.setEnabled(False)
        self._set_status('Nav2로 웨이포인트 전송 중...')
        self.node.send_waypoints(self.waypoints)

    def _browse_path(self):
        path = QFileDialog.getExistingDirectory(self, '저장 경로 선택', self.path_edit.text())
        if path:
            self.path_edit.setText(path)

    # ── Waypoint received from ROS ────────────────────────────────────────────
    def on_waypoint_received(self, x: float, y: float, qz: float, qw: float):
        wp = {
            'position':    {'x': round(x,  4), 'y': round(y,  4), 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': round(qz, 6), 'w': round(qw, 6)}
        }
        self.waypoints.append(wp)

        row = self.table.rowCount()
        self.table.insertRow(row)
        items = [
            str(row + 1),
            f'{x:.4f}',
            f'{y:.4f}',
            f'{qz:.4f}',
            f'{qw:.4f}',
        ]
        for col, val in enumerate(items):
            item = QTableWidgetItem(val)
            item.setTextAlignment(Qt.AlignCenter)
            if col == 0:
                item.setForeground(QColor('#5a6480'))
            self.table.setItem(row, col, item)

        self.table.scrollToBottom()
        self.count_label.setText(str(len(self.waypoints)))
        self._set_status(f'웨이포인트 [{row + 1}] 추가됨: x={x:.3f}, y={y:.3f}')
        self._update_buttons()

    # ── Nav2 feedback/result ──────────────────────────────────────────────────
    def on_nav2_feedback(self, current: int, total: int):
        self.nav2_status.setText(f'[{current + 1}/{total}] 이동 중')
        self._set_status(f'Nav2 이동 중: [{current + 1}/{total}]')

    def on_nav2_result(self, success: bool, message: str):
        self._set_state('DONE')
        self.nav2_status.setText(message)
        self._set_status(message)
        self._update_buttons()
        if success:
            QMessageBox.information(self, 'Nav2 완료', message)
        else:
            QMessageBox.warning(self, 'Nav2 오류', message)

    # ── YAML save ────────────────────────────────────────────────────────────
    def _save_yaml(self):
        if not self.waypoints:
            self._set_status('저장할 웨이포인트가 없습니다.')
            return

        save_path = self.path_edit.text().strip()
        os.makedirs(save_path, exist_ok=True)

        fname = self.filename_edit.text().strip()
        if not fname:
            fname = f'waypoints_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        if not fname.endswith('.yaml'):
            fname += '.yaml'

        filepath = os.path.join(save_path, fname)
        data = {'map_frame': 'map', 'waypoints': self.waypoints}

        with open(filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

        self._set_status(f'저장 완료 → {filepath}')
        QMessageBox.information(self, '저장 완료', f'{len(self.waypoints)}개 웨이포인트 저장\n{filepath}')


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    signals = Signals()
    node = WaypointNode(signals)

    window = MainWindow(node)

    # Connect signals
    signals.waypoint_received.connect(window.on_waypoint_received)
    signals.nav2_feedback.connect(window.on_nav2_feedback)
    signals.nav2_result.connect(window.on_nav2_result)

    ros_thread = RosThread(node)
    ros_thread.start()

    window.show()
    ret = app.exec_()

    ros_thread.quit()
    ros_thread.wait()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()