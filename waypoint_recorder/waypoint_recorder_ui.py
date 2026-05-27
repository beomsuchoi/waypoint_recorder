import sys
import os
import yaml
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from nav2_msgs.action import FollowWaypoints
from action_msgs.msg import GoalStatus

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QTableWidget, QTableWidgetItem,
    QHeaderView, QFileDialog, QMessageBox, QGroupBox, QStatusBar,
    QSizePolicy
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QColor

# ── 끼임 감지 파라미터 ─────────────────────────────────────────────────────────
_STUCK_LIN_THRESH = 0.05   # m/s  이하  → 선속도 거의 없음
_STUCK_ANG_THRESH = 0.15   # rad/s 이상 → 회전 중
_STUCK_TIMEOUT    = 10.0   # 초  이 시간 이상 지속되면 끼임 판정


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
    robot_stuck       = pyqtSignal()                            # 끼임 감지


# ── ROS2 Node ─────────────────────────────────────────────────────────────────
class WaypointNode(Node):
    def __init__(self, signals: Signals):
        super().__init__('waypoint_recorder_ui')
        self.signals = signals
        self.recording = False

        # 웨이포인트 구독
        self.sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_cb, 10
        )
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # ── 끼임 감지 ─────────────────────────────────────────────────────────
        self._stuck_pub = self.create_publisher(Bool, '/robot_stuck', 10)
        self._odom_sub  = self.create_subscription(
            Odometry, '/fastlio2/lio_odom', self._odom_cb, 10
        )
        self._stuck_spin_start: float | None = None
        self._stuck_published: bool = False

        self.get_logger().info('[WaypointNode] 시작 — 끼임 감지(/robot_stuck) 활성화')

    # ── 웨이포인트 수신 ───────────────────────────────────────────────────────
    def _goal_cb(self, msg: PoseStamped):
        if not self.recording:
            return
        x  = msg.pose.position.x
        y  = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.signals.waypoint_received.emit(x, y, qz, qw)

    # ── 끼임 감지 ─────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        """선속도 ≈ 0 이고 각속도 ≠ 0 인 상태가 STUCK_TIMEOUT 초 지속되면 /robot_stuck 발행."""
        vx  = msg.twist.twist.linear.x
        wz  = msg.twist.twist.angular.z
        now = self.get_clock().now().nanoseconds * 1e-9

        is_spinning = abs(vx) < _STUCK_LIN_THRESH and abs(wz) > _STUCK_ANG_THRESH

        if is_spinning:
            if self._stuck_spin_start is None:
                self._stuck_spin_start = now
            elif (not self._stuck_published
                  and now - self._stuck_spin_start >= _STUCK_TIMEOUT):
                self.get_logger().warn(
                    f'[StuckDetector] {_STUCK_TIMEOUT:.0f}초 이상 제자리 회전 감지 '
                    f'→ /robot_stuck 발행')
                out = Bool()
                out.data = True
                self._stuck_pub.publish(out)
                self._stuck_published = True
                self.signals.robot_stuck.emit()   # UI 알림
        else:
            # 로봇이 다시 움직이기 시작하면 초기화
            if self._stuck_published:
                self.get_logger().info('[StuckDetector] 로봇 움직임 재개 → 초기화')
            self._stuck_spin_start = None
            self._stuck_published = False

    # ── Nav2 웨이포인트 전송 ──────────────────────────────────────────────────
    def send_waypoints(self, waypoints: list, map_frame: str = 'map'):
        # 새 목표 전송 시 끼임 상태 초기화
        self._stuck_spin_start = None
        self._stuck_published = False

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
        self.setMinimumSize(780, 620)
        self._apply_dark_theme()
        self._build_ui()
        self._update_buttons()

    # ── Theme ─────────────────────────────────────────────────────────────────
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
            QTableWidget QLineEdit {
                background: #1a2a3a;
                border: 1px solid #00e5ff;
                color: #00e5ff;
                padding: 2px 4px;
            }
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

    # ── UI Build ──────────────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(20, 20, 20, 12)
        root.setSpacing(14)

        # ── Header
        header = QHBoxLayout()
        title = QLabel('WAYPOINT RECORDER')
        title.setStyleSheet(
            "font-size:18px; font-weight:700; color:#eef2ff; letter-spacing:2px;"
            " font-family:'Consolas','Courier New',monospace;")
        self.state_label = QLabel('IDLE')
        self.state_label.setStyleSheet(
            "font-size:11px; font-weight:600; padding:4px 12px; border-radius:4px;"
            " font-family:'Consolas','Courier New',monospace;"
            " background:rgba(90,100,128,40); color:#5a6480; border:1px solid #1f2433;")
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
        lbl_path = QLabel('경로')
        lbl_path.setFixedWidth(40)
        lbl_path.setStyleSheet("color:#5a6480; font-size:11px;")
        row1.addWidget(lbl_path)
        row1.addWidget(self.path_edit)
        row1.addWidget(browse_btn)

        row2 = QHBoxLayout()
        self.filename_edit = QLineEdit()
        self.filename_edit.setPlaceholderText('비워두면 타임스탬프 자동 생성')
        lbl_file = QLabel('파일명')
        lbl_file.setFixedWidth(40)
        lbl_file.setStyleSheet("color:#5a6480; font-size:11px;")
        lbl_ext = QLabel('.yaml')
        lbl_ext.setStyleSheet("color:#5a6480; font-family:'Consolas','Courier New',monospace;")
        row2.addWidget(lbl_file)
        row2.addWidget(self.filename_edit)
        row2.addWidget(lbl_ext)

        cfg_layout.addLayout(row1)
        cfg_layout.addLayout(row2)
        root.addWidget(cfg_group)

        # ── Control buttons (2행 구조)
        ctrl_group = QGroupBox('CONTROL')
        ctrl_v = QVBoxLayout(ctrl_group)
        ctrl_v.setSpacing(8)

        # 1행: 녹화 제어
        row_rec = QHBoxLayout()
        row_rec.setSpacing(10)
        self.btn_start = QPushButton('▶  저장 시작')
        self.btn_stop  = QPushButton('■  저장 완료')
        self.btn_undo  = QPushButton('↩  Undo')
        self.btn_clear = QPushButton('✕  초기화')
        self.btn_start.setStyleSheet(self._btn_style('#39ff7e'))
        self.btn_stop.setStyleSheet(self._btn_style('#ffd166'))
        self.btn_undo.setStyleSheet(self._btn_style('#ff3b5c'))
        self.btn_clear.setStyleSheet(self._btn_style('#5a6480'))
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_undo.clicked.connect(self._on_undo)
        self.btn_clear.clicked.connect(self._on_clear)
        for btn in [self.btn_start, self.btn_stop, self.btn_undo, self.btn_clear]:
            row_rec.addWidget(btn)

        # 2행: 파일·편집·전송
        row_act = QHBoxLayout()
        row_act.setSpacing(10)
        self.btn_load   = QPushButton('📂  YAML 불러오기')
        self.btn_delete = QPushButton('🗑  선택 삭제')
        self.btn_save_edit = QPushButton('💾  편집 저장')
        self.btn_send   = QPushButton('⬆  Nav2 전송')
        self.btn_load.setStyleSheet(self._btn_style('#a78bfa'))
        self.btn_delete.setStyleSheet(self._btn_style('#ff6b35'))
        self.btn_save_edit.setStyleSheet(self._btn_style('#ffd166'))
        self.btn_send.setStyleSheet(self._btn_style('#00e5ff'))
        self.btn_load.clicked.connect(self._on_load_yaml)
        self.btn_delete.clicked.connect(self._on_delete_row)
        self.btn_save_edit.clicked.connect(self._on_save_edit)
        self.btn_send.clicked.connect(self._on_send)
        for btn in [self.btn_load, self.btn_delete, self.btn_save_edit, self.btn_send]:
            row_act.addWidget(btn)

        ctrl_v.addLayout(row_rec)
        ctrl_v.addLayout(row_act)
        root.addWidget(ctrl_group)

        # ── 카운터 + 끼임 경고
        count_row = QHBoxLayout()
        count_row.setContentsMargins(4, 0, 4, 0)
        self.count_label = QLabel('0')
        self.count_label.setStyleSheet(
            "font-size:40px; font-weight:700; color:#eef2ff;"
            " font-family:'Consolas','Courier New',monospace;")
        unit = QLabel(' waypoints')
        unit.setStyleSheet("font-size:13px; color:#5a6480; padding-bottom:4px;")
        count_row.addWidget(self.count_label)
        count_row.addWidget(unit)
        count_row.addStretch()

        # Nav2 상태 + 끼임 경고 (오른쪽)
        right_status = QVBoxLayout()
        right_status.setSpacing(2)
        self.nav2_status = QLabel('')
        self.nav2_status.setStyleSheet(
            "font-size:11px; font-family:'Consolas','Courier New',monospace; color:#5a6480;")
        self.stuck_label = QLabel('')
        self.stuck_label.setStyleSheet(
            "font-size:11px; font-weight:700; font-family:'Consolas','Courier New',monospace;"
            " color:#ff3b5c; background:rgba(255,59,92,15);"
            " border:1px solid rgba(255,59,92,60); border-radius:4px; padding:2px 8px;")
        self.stuck_label.setVisible(False)
        right_status.addWidget(self.nav2_status)
        right_status.addWidget(self.stuck_label)
        count_row.addLayout(right_status)
        root.addLayout(count_row)

        # ── 테이블 (더블클릭으로 X/Y/qz/qw 편집 가능)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(['#', 'X', 'Y', 'Yaw (qz)', 'W'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Fixed)
        self.table.setColumnWidth(0, 40)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setEditTriggers(QTableWidget.DoubleClicked)   # 더블클릭 편집
        self.table.setAlternatingRowColors(True)
        self.table.setStyleSheet(
            self.table.styleSheet() + "QTableWidget { alternate-background-color: #111520; }")
        self.table.itemChanged.connect(self._on_table_item_changed)
        root.addWidget(self.table)

        # 편집 안내
        hint = QLabel('  ✏  X · Y · qz · W 셀을 더블클릭하면 값을 직접 수정할 수 있습니다.')
        hint.setStyleSheet(
            "font-size:10px; color:#3a4055; font-family:'Consolas','Courier New',monospace;")
        root.addWidget(hint)

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
            'LOADED':    ('LOADED',    'background:rgba(167,139,250,20); color:#a78bfa;  border:1px solid rgba(167,139,250,80);'),
            'STUCK':     ('⚠ STUCK',  'background:rgba(255,59,92,25);   color:#ff3b5c;  border:1px solid rgba(255,59,92,80);'),
        }
        text, style = styles.get(state, styles['IDLE'])
        base = ("font-size:11px; font-weight:600; padding:4px 12px; border-radius:4px;"
                " font-family:'Consolas','Courier New',monospace; ")
        self.state_label.setText(text)
        self.state_label.setStyleSheet(base + style)

    def _set_status(self, msg: str):
        self.status_bar.showMessage(f'  {msg}')

    def _update_buttons(self):
        has_wp    = len(self.waypoints) > 0
        has_sel   = len(self.table.selectedItems()) > 0
        self.btn_start.setEnabled(not self.recording)
        self.btn_stop.setEnabled(self.recording)
        self.btn_undo.setEnabled(has_wp and self.recording)
        self.btn_clear.setEnabled(has_wp and not self.recording)
        self.btn_load.setEnabled(not self.recording)
        self.btn_delete.setEnabled(has_wp and not self.recording)
        self.btn_save_edit.setEnabled(has_wp and not self.recording)
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
        self.table.blockSignals(True)
        self.table.removeRow(self.table.rowCount() - 1)
        self.table.blockSignals(False)
        self.count_label.setText(str(len(self.waypoints)))
        self._set_status(f'마지막 웨이포인트 제거됨. 총 {len(self.waypoints)}개.')
        self._update_buttons()

    def _on_clear(self):
        reply = QMessageBox.question(
            self, '확인', '모든 웨이포인트를 삭제하시겠습니까?',
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.waypoints.clear()
            self.table.blockSignals(True)
            self.table.setRowCount(0)
            self.table.blockSignals(False)
            self.count_label.setText('0')
            self._set_state('IDLE')
            self._set_status('초기화되었습니다.')
            self._update_buttons()

    # ── 선택 행 삭제 ──────────────────────────────────────────────────────────
    def _on_delete_row(self):
        selected_rows = sorted(
            set(item.row() for item in self.table.selectedItems()),
            reverse=True  # 뒤에서부터 삭제해야 인덱스 안 밀림
        )
        if not selected_rows:
            self._set_status('삭제할 행을 먼저 선택하세요.')
            return

        self.table.blockSignals(True)
        for row in selected_rows:
            if row < len(self.waypoints):
                self.waypoints.pop(row)
                self.table.removeRow(row)

        # '#' 열 번호 재정렬
        for r in range(self.table.rowCount()):
            item = self.table.item(r, 0)
            if item:
                item.setText(str(r + 1))
        self.table.blockSignals(False)

        self.count_label.setText(str(len(self.waypoints)))
        self._set_status(f'{len(selected_rows)}개 삭제됨. 총 {len(self.waypoints)}개 남음.')
        self._update_buttons()

    # ── 편집 내용 저장 ────────────────────────────────────────────────────────
    def _on_save_edit(self):
        """테이블에서 편집된 내용을 YAML로 저장."""
        if not self.waypoints:
            self._set_status('저장할 웨이포인트가 없습니다.')
            return
        self._save_yaml()

    # ── 테이블 셀 편집 동기화 ─────────────────────────────────────────────────
    def _on_table_item_changed(self, item: QTableWidgetItem):
        """더블클릭으로 수정된 셀 값을 self.waypoints에 반영."""
        col = item.column()
        row = item.row()

        # '#' 열(0)은 수정 불가 / 범위 초과 방지
        if col == 0 or row >= len(self.waypoints):
            return

        try:
            val = float(item.text())
        except ValueError:
            # 숫자가 아닌 경우 → 원래 값으로 복원
            self.table.blockSignals(True)
            wp = self.waypoints[row]
            restore = {
                1: f"{wp['position']['x']:.4f}",
                2: f"{wp['position']['y']:.4f}",
                3: f"{wp['orientation']['z']:.6f}",
                4: f"{wp['orientation']['w']:.6f}",
            }.get(col, '')
            item.setText(restore)
            self.table.blockSignals(False)
            self._set_status(f'⚠ 잘못된 값입니다. 숫자를 입력해 주세요.')
            return

        wp = self.waypoints[row]
        if col == 1:
            wp['position']['x'] = round(val, 4)
            fmt = f'{val:.4f}'
        elif col == 2:
            wp['position']['y'] = round(val, 4)
            fmt = f'{val:.4f}'
        elif col == 3:
            wp['orientation']['z'] = round(val, 6)
            fmt = f'{val:.6f}'
        elif col == 4:
            wp['orientation']['w'] = round(val, 6)
            fmt = f'{val:.6f}'
        else:
            return

        # 포맷 맞춰 다시 표시 (시그널 차단)
        self.table.blockSignals(True)
        item.setText(fmt)
        self.table.blockSignals(False)

        col_names = {1: 'X', 2: 'Y', 3: 'qz', 4: 'qw'}
        self._set_status(f'[{row + 1}] {col_names[col]} 수정 → {fmt}')

    # ── YAML 불러오기 ─────────────────────────────────────────────────────────
    def _on_load_yaml(self):
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

        if self.waypoints:
            reply = QMessageBox.question(
                self, '확인',
                f'현재 {len(self.waypoints)}개의 웨이포인트가 있습니다.\n'
                '불러온 데이터로 교체하시겠습니까?',
                QMessageBox.Yes | QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return

        self.waypoints.clear()
        self.table.blockSignals(True)
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
                errors.append(f'  #{idx + 1}: {e}')
                continue

            self.waypoints.append({
                'position':    {'x': round(x,  4), 'y': round(y,  4),
                                'z': float(pos.get('z', 0.0))},
                'orientation': {
                    'x': float(ori.get('x', 0.0)),
                    'y': float(ori.get('y', 0.0)),
                    'z': round(qz, 6),
                    'w': round(qw, 6),
                }
            })
            self._insert_table_row(len(self.waypoints) - 1, x, y, qz, qw)

        self.table.blockSignals(False)
        self.count_label.setText(str(len(self.waypoints)))
        self._set_state('LOADED')

        fname = os.path.basename(filepath)
        if errors:
            QMessageBox.warning(
                self, '일부 웨이포인트 오류',
                f'{len(self.waypoints)}개 로드 성공, {len(errors)}개 건너뜀:\n'
                + '\n'.join(errors)
            )
            self._set_status(f'[{fname}] {len(self.waypoints)}개 로드 (일부 오류) — 셀 더블클릭으로 수정 가능')
        else:
            self._set_status(
                f'[{fname}] {len(self.waypoints)}개 로드 완료 — 셀 더블클릭으로 수정, Nav2 전송 버튼으로 실행')

        self._update_buttons()

    # ── Nav2 전송 ─────────────────────────────────────────────────────────────
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

    # ── 테이블 행 삽입 (공통 헬퍼) ───────────────────────────────────────────
    def _insert_table_row(self, row: int, x: float, y: float,
                          qz: float, qw: float):
        """row 위치에 행을 삽입. blockSignals는 호출자가 관리."""
        self.table.insertRow(row)
        # '#' 열: 읽기 전용
        num_item = QTableWidgetItem(str(row + 1))
        num_item.setTextAlignment(Qt.AlignCenter)
        num_item.setForeground(QColor('#5a6480'))
        num_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)  # 편집 불가
        self.table.setItem(row, 0, num_item)

        # X, Y, qz, qw: 편집 가능
        for col, val, fmt in [
            (1, x,  f'{x:.4f}'),
            (2, y,  f'{y:.4f}'),
            (3, qz, f'{qz:.6f}'),
            (4, qw, f'{qw:.6f}'),
        ]:
            cell = QTableWidgetItem(fmt)
            cell.setTextAlignment(Qt.AlignCenter)
            cell.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled | Qt.ItemIsEditable)
            self.table.setItem(row, col, cell)

    # ── Waypoint received from ROS ────────────────────────────────────────────
    def on_waypoint_received(self, x: float, y: float, qz: float, qw: float):
        wp = {
            'position':    {'x': round(x,  4), 'y': round(y,  4), 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': round(qz, 6), 'w': round(qw, 6)}
        }
        self.waypoints.append(wp)

        self.table.blockSignals(True)
        self._insert_table_row(self.table.rowCount(), x, y, qz, qw)
        self.table.blockSignals(False)

        self.table.scrollToBottom()
        self.count_label.setText(str(len(self.waypoints)))
        row = len(self.waypoints)
        self._set_status(f'웨이포인트 [{row}] 추가됨: x={x:.3f}, y={y:.3f}')
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

    # ── 끼임 감지 알림 ───────────────────────────────────────────────────────
    def on_robot_stuck(self):
        self._set_state('STUCK')
        self.stuck_label.setText(f'⚠ STUCK — 로봇이 {_STUCK_TIMEOUT:.0f}초 이상 제자리 회전 중')
        self.stuck_label.setVisible(True)
        self._set_status(f'⚠ 끼임 감지: /robot_stuck 토픽 발행됨')

        # 5초 후 경고 숨김
        from PyQt5.QtCore import QTimer
        QTimer.singleShot(5000, lambda: self.stuck_label.setVisible(False))

    # ── YAML save ─────────────────────────────────────────────────────────────
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
        QMessageBox.information(self, '저장 완료',
                                f'{len(self.waypoints)}개 웨이포인트 저장\n{filepath}')


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
    signals.robot_stuck.connect(window.on_robot_stuck)      # 끼임 감지

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
