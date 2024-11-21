#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from typing import Optional, List, Dict

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QComboBox, QGroupBox,
    QTabWidget, QListWidget, QSpinBox, QDoubleSpinBox,
    QCheckBox, QMessageBox, QFrame
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject

from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionFeedback
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler

# 服务消息类型的导入
from wp_nav_controller.srv import NavigationCommand, NavigationCommandResponse
from wp_nav_controller.srv import SequenceCommand, SequenceCommandResponse


# 创建一个信号发射器类
class SignalEmitter(QObject):
    status_signal = Signal(str)  # 导航状态信号
    position_signal = Signal(float, float)  # 位置信号
    message_signal = Signal(str, int)  # 状态栏消息信号


class NavigationControllerUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Navigation Controller")
        self.setMinimumSize(800, 600)

        # 创建信号发射器
        self.signal_emitter = SignalEmitter()

        # 连接信号到对应的槽
        self.signal_emitter.status_signal.connect(self._update_status_label)
        self.signal_emitter.position_signal.connect(self._update_position)
        self.signal_emitter.message_signal.connect(self._update_status_bar)

        # 添加成员变量
        self._navigator = None

        # 初始化ROS节点
        if not rospy.core.is_initialized():
            rospy.init_node('navigation_controller_ui', anonymous=True)

        # 创建中心部件和主布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # 创建标签页
        tab_widget = QTabWidget()
        main_layout.addWidget(tab_widget)

        # 添加导航控制和序列控制标签页
        tab_widget.addTab(self._create_navigation_tab(), "Navigation Control")
        tab_widget.addTab(self._create_sequence_tab(), "Sequence Control")

        # 状态栏显示最近的操作结果
        self.statusBar().showMessage("Ready")

        # 订阅move_base状态话题
        self.status_sub = rospy.Subscriber('/move_base/status',
                                           GoalStatusArray,
                                           self._status_callback)

        # 订阅导航结果和反馈（可选）
        self.result_sub = rospy.Subscriber('/move_base/result',
                                           MoveBaseActionResult,
                                           self._result_callback)
        self.feedback_sub = rospy.Subscriber('/move_base/feedback',
                                             MoveBaseActionFeedback,
                                             self._feedback_callback)

    def _status_callback(self, msg):
        """处理move_base状态回调"""
        try:
            if msg.status_list:
                status = msg.status_list[-1].status
                status_text = self._get_status_text(status)
                # 发射信号而不是直接更新UI
                self.signal_emitter.status_signal.emit(f"Navigation Status: {status_text}")
        except Exception as e:
            rospy.logwarn(f"Error in status callback: {e}")

    def _result_callback(self, msg):
        """处理导航结果回调"""
        try:
            status = msg.status.status
            if status == 3:  # SUCCEEDED
                self.signal_emitter.message_signal.emit("Navigation completed successfully", 3000)
            elif status == 4:  # ABORTED
                self.signal_emitter.message_signal.emit("Navigation aborted", 3000)
            elif status == 2:  # PREEMPTED
                self.signal_emitter.message_signal.emit("Navigation cancelled", 3000)
        except Exception as e:
            rospy.logwarn(f"Error in result callback: {e}")

    def _feedback_callback(self, msg):
        """处理导航反馈回调"""
        try:
            pos = msg.feedback.base_position.pose.position
            # 发射位置信号
            self.signal_emitter.position_signal.emit(pos.x, pos.y)
        except Exception as e:
            rospy.logwarn(f"Error in feedback callback: {e}")

    def _get_status_text(self, status):
        """获取状态文本描述"""
        status_dict = {
            0: "PENDING",
            1: "ACTIVE",
            2: "PREEMPTED",
            3: "SUCCEEDED",
            4: "ABORTED",
            5: "REJECTED",
            6: "PREEMPTING",
            7: "RECALLING",
            8: "RECALLED",
            9: "LOST"
        }
        return status_dict.get(status, "UNKNOWN")

    def _update_status_label(self, text):
        """更新状态标签"""
        self.nav_status_label.setText(text)

    def _update_position(self, x, y):
        """更新位置信息"""
        self.statusBar().showMessage(f"Current position - X: {x:.2f}, Y: {y:.2f}", 1000)

    def _update_status_bar(self, text, timeout):
        """更新状态栏"""
        self.statusBar().showMessage(text, timeout)

    def _show_error(self, title: str, message: str):
        """显示错误对话框"""
        # 错误对话框应该在主线程中显示
        QMessageBox.critical(self, title, message)
        self.signal_emitter.message_signal.emit(f"Error: {message}", 5000)

    def _create_navigation_tab(self) -> QWidget:
        """创建导航控制标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # 目标设置组
        target_group = QGroupBox("Navigation Target")
        target_layout = QVBoxLayout(target_group)

        # 目标类型选择
        target_type_layout = QHBoxLayout()
        target_type_layout.addWidget(QLabel("Target Type:"))
        self.target_type_combo = QComboBox()
        self.target_type_combo.addItems(["Waypoint", "Index", "Pose"])
        self.target_type_combo.currentTextChanged.connect(
            self._on_target_type_changed)
        target_type_layout.addWidget(self.target_type_combo)
        target_layout.addLayout(target_type_layout)

        # 目标参数堆叠布局
        self.waypoint_layout = QHBoxLayout()
        self.waypoint_layout.addWidget(QLabel("Waypoint Name:"))
        self.waypoint_input = QLineEdit()
        self.waypoint_layout.addWidget(self.waypoint_input)
        target_layout.addLayout(self.waypoint_layout)

        self.index_layout = QHBoxLayout()
        self.index_layout.addWidget(QLabel("Waypoint Index:"))
        self.index_spin = QSpinBox()
        self.index_spin.setRange(0, 99)
        self.index_layout.addWidget(self.index_spin)
        target_layout.addLayout(self.index_layout)
        self.index_layout.setEnabled(False)

        self.pose_layout = QHBoxLayout()
        self.pose_layout.addWidget(QLabel("X:"))
        self.pose_x = QDoubleSpinBox()
        self.pose_x.setRange(-999.99, 999.99)
        self.pose_layout.addWidget(self.pose_x)
        self.pose_layout.addWidget(QLabel("Y:"))
        self.pose_y = QDoubleSpinBox()
        self.pose_y.setRange(-999.99, 999.99)
        self.pose_layout.addWidget(self.pose_y)
        self.pose_layout.addWidget(QLabel("Yaw:"))
        self.pose_yaw = QDoubleSpinBox()
        self.pose_yaw.setRange(-3.14, 3.14)
        self.pose_layout.addWidget(self.pose_yaw)
        target_layout.addLayout(self.pose_layout)
        self.pose_layout.setEnabled(False)

        layout.addWidget(target_group)

        # 导航控制按钮组
        control_group = QGroupBox("Navigation Control")
        control_layout = QHBoxLayout(control_group)

        self.nav_to_btn = QPushButton("Navigate To")
        self.nav_to_btn.clicked.connect(self._on_navigate_to)
        control_layout.addWidget(self.nav_to_btn)

        self.pause_nav_btn = QPushButton("Pause")
        self.pause_nav_btn.clicked.connect(self._on_pause_navigation)
        control_layout.addWidget(self.pause_nav_btn)

        self.resume_nav_btn = QPushButton("Resume")
        self.resume_nav_btn.clicked.connect(self._on_resume_navigation)
        control_layout.addWidget(self.resume_nav_btn)

        self.stop_nav_btn = QPushButton("Stop")
        self.stop_nav_btn.clicked.connect(self._on_stop_navigation)
        control_layout.addWidget(self.stop_nav_btn)

        self.clear_nav_btn = QPushButton("Clear Costmaps")
        self.clear_nav_btn.clicked.connect(self._on_clear_costmaps)
        control_layout.addWidget(self.clear_nav_btn)

        layout.addWidget(control_group)

        # 导航状态显示
        status_group = QGroupBox("Navigation Status")
        status_layout = QVBoxLayout(status_group)
        self.nav_status_label = QLabel("Current Status: IDLE")
        status_layout.addWidget(self.nav_status_label)
        layout.addWidget(status_group)

        layout.addStretch()
        return tab

    def _create_sequence_tab(self) -> QWidget:
        """创建序列控制标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # 序列创建组
        sequence_group = QGroupBox("Sequence Creation")
        sequence_layout = QVBoxLayout(sequence_group)

        # 航点列表
        self.waypoint_list = QListWidget()
        sequence_layout.addWidget(self.waypoint_list)

        # 航点添加控件
        add_layout = QHBoxLayout()
        self.add_type_combo = QComboBox()
        self.add_type_combo.addItems(["Waypoint", "Index"])
        add_layout.addWidget(self.add_type_combo)

        self.add_waypoint_input = QLineEdit()
        add_layout.addWidget(self.add_waypoint_input)

        add_btn = QPushButton("Add")
        add_btn.clicked.connect(self._on_add_waypoint)
        add_layout.addWidget(add_btn)

        sequence_layout.addLayout(add_layout)

        # 序列操作按钮
        sequence_ops_layout = QHBoxLayout()
        remove_btn = QPushButton("Remove Selected")
        remove_btn.clicked.connect(self._on_remove_waypoint)
        sequence_ops_layout.addWidget(remove_btn)

        clear_btn = QPushButton("Clear All")
        clear_btn.clicked.connect(self._on_clear_sequence)
        sequence_ops_layout.addWidget(clear_btn)

        sequence_layout.addLayout(sequence_ops_layout)
        layout.addWidget(sequence_group)

        # 序列导航控制组
        nav_control_group = QGroupBox("Sequence Navigation Control")
        nav_control_layout = QVBoxLayout(nav_control_group)

        # 导航模式设置
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Single", "Loop", "Roundtrip"])
        mode_layout.addWidget(self.mode_combo)

        self.loop_check = QCheckBox("Enable Loop")
        mode_layout.addWidget(self.loop_check)

        self.reverse_check = QCheckBox("Reverse Direction")
        mode_layout.addWidget(self.reverse_check)

        nav_control_layout.addLayout(mode_layout)

        # 起始点设置
        start_layout = QHBoxLayout()
        start_layout.addWidget(QLabel("Start Point:"))
        self.start_point_input = QLineEdit()
        start_layout.addWidget(self.start_point_input)
        nav_control_layout.addLayout(start_layout)

        # 控制按钮
        control_layout = QHBoxLayout()
        start_seq_btn = QPushButton("Start")
        start_seq_btn.clicked.connect(self._on_start_sequence)
        control_layout.addWidget(start_seq_btn)

        pause_seq_btn = QPushButton("Pause")
        pause_seq_btn.clicked.connect(self._on_pause_sequence)
        control_layout.addWidget(pause_seq_btn)

        resume_seq_btn = QPushButton("Resume")
        resume_seq_btn.clicked.connect(self._on_resume_sequence)
        control_layout.addWidget(resume_seq_btn)

        stop_seq_btn = QPushButton("Stop")
        stop_seq_btn.clicked.connect(self._on_stop_sequence)
        control_layout.addWidget(stop_seq_btn)

        nav_control_layout.addLayout(control_layout)
        layout.addWidget(nav_control_group)

        # 序列状态显示
        status_group = QGroupBox("Sequence Status")
        status_layout = QVBoxLayout(status_group)
        self.seq_status_label = QLabel("Sequence Status: No sequence running")
        status_layout.addWidget(self.seq_status_label)
        layout.addWidget(status_group)

        layout.addStretch()
        return tab

    def _on_target_type_changed(self, target_type: str):
        """处理目标类型改变事件"""
        # 禁用所有输入布局
        self.waypoint_layout.setEnabled(False)
        self.index_layout.setEnabled(False)
        self.pose_layout.setEnabled(False)

        # 根据选择启用对应的输入布局
        if target_type == "Waypoint":
            self.waypoint_layout.setEnabled(True)
        elif target_type == "Index":
            self.index_layout.setEnabled(True)
        elif target_type == "Pose":
            self.pose_layout.setEnabled(True)

    def _on_navigate_to(self):
        """处理导航至目标的操作"""
        try:
            target_type = self.target_type_combo.currentText().lower()
            if target_type == "waypoint":
                waypoint = self.waypoint_input.text()
                if not waypoint:
                    raise ValueError("Waypoint name cannot be empty")
                self._call_navigation_service('nav_to', 'waypoint', [waypoint])

            elif target_type == "index":
                index = str(self.index_spin.value())
                self._call_navigation_service('nav_to', 'index', [index])

            elif target_type == "pose":
                x = str(self.pose_x.value())
                y = str(self.pose_y.value())
                yaw = str(self.pose_yaw.value())
                self._call_navigation_service('nav_to', 'pose', [x, y, yaw])

        except Exception as e:
            self._show_error("Navigation Error", str(e))

    def _on_pause_navigation(self):
        """暂停导航"""
        self._call_navigation_service('pause', '', [])

    def _on_resume_navigation(self):
        """继续导航"""
        self._call_navigation_service('resume', '', [])

    def _on_stop_navigation(self):
        """停止导航"""
        self._call_navigation_service('stop', '', [])

    def _on_clear_costmaps(self):
        """清除代价地图"""
        self._call_navigation_service('clear', '', [])

    def _on_add_waypoint(self):
        """添加航点到序列"""
        try:
            waypoint_type = self.add_type_combo.currentText()
            value = self.add_waypoint_input.text()

            if not value:
                raise ValueError("Waypoint value cannot be empty")

            if waypoint_type == "Index":
                # 确保是有效的索引
                try:
                    index = int(value)
                    if index < 0:
                        raise ValueError()
                    value = f"@{index}"
                except ValueError:
                    raise ValueError("Invalid index number")

            self.waypoint_list.addItem(value)
            self.add_waypoint_input.clear()

        except Exception as e:
            self._show_error("Add Waypoint Error", str(e))

    def _on_remove_waypoint(self):
        """从序列中移除选中的航点"""
        selected_items = self.waypoint_list.selectedItems()
        for item in selected_items:
            self.waypoint_list.takeItem(self.waypoint_list.row(item))

    def _on_clear_sequence(self):
        """清空序列"""
        self.waypoint_list.clear()
        self._call_sequence_service('clear', '', False, False, [])

    def _on_start_sequence(self):
        """启动序列导航"""
        try:
            # 获取序列中的所有航点
            waypoints = []
            for i in range(self.waypoint_list.count()):
                waypoints.append(self.waypoint_list.item(i).text())

            if not waypoints:
                raise ValueError("Sequence is empty")

            # 创建序列
            self._call_sequence_service('create', '', False, False, waypoints)

            # 启动序列导航
            mode = self.mode_combo.currentText().lower()
            loop = self.loop_check.isChecked()
            reverse = self.reverse_check.isChecked()
            start_point = self.start_point_input.text() or None

            if start_point:
                params = [start_point]
            else:
                params = []

            self._call_sequence_service('start', mode, loop, reverse, params)

        except Exception as e:
            self._show_error("Start Sequence Error", str(e))

    def _on_pause_sequence(self):
        """暂停序列导航"""
        self._call_sequence_service('pause', '', False, False, [])

    def _on_resume_sequence(self):
        """继续序列导航"""
        self._call_sequence_service('resume', '', False, False, [])

    def _on_stop_sequence(self):
        """停止序列导航"""
        self._call_sequence_service('stop', '', False, False, [])

    def _call_navigation_service(self, command: str, target_type: str, params: List[str]):
        """调用导航控制服务

        Args:
            command: 命令类型（nav_to, pause, resume, stop, clear）
            target_type: 目标类型（waypoint, index, pose）
            params: 参数列表
        """
        try:
            # 检查服务是否可用
            service_name = '/wp_nav_controller/navigation'
            try:
                rospy.wait_for_service(service_name, timeout=1.0)
            except rospy.ROSException:
                self._show_error(
                    "Service Error", f"Navigation service '{service_name}' is not available")
                return

            nav_service = rospy.ServiceProxy(service_name, NavigationCommand)

            # 打印调试信息
            rospy.loginfo(
                f"Calling navigation service with command: {command}, type: {target_type}, params: {params}")

            response = nav_service(
                command=command,
                target_type=target_type,
                params=params
            )

            if response.success:
                self.statusBar().showMessage(
                    f"Navigation command success: {response.message}", 3000)
            else:
                self._show_error("Navigation Error",
                                 f"Command failed: {response.message}\nCommand: {command}\nType: {target_type}\nParams: {params}")

        except rospy.ServiceException as e:
            self._show_error("Service Error", f"Service call failed: {str(e)}")
        except Exception as e:
            self._show_error("Error", f"Unexpected error: {str(e)}")

    def _call_sequence_service(self, command: str, mode: str, loop: bool, reverse: bool, params: List[str]):
        """调用序列导航服务

        Args:
            command: 命令类型（create, start, pause, resume, stop, etc.）
            mode: 导航模式（single, loop, roundtrip）
            loop: 是否循环执行
            reverse: 是否反向执行
            params: 参数列表
        """
        try:
            # 检查服务是否可用
            service_name = '/wp_nav_controller/sequence'
            try:
                rospy.wait_for_service(service_name, timeout=1.0)
            except rospy.ROSException:
                self._show_error(
                    "Service Error", f"Sequence service '{service_name}' is not available")
                return

            seq_service = rospy.ServiceProxy(service_name, SequenceCommand)

            # 打印调试信息
            rospy.loginfo(f"Calling sequence service with command: {command}, mode: {mode}, "
                          f"loop: {loop}, reverse: {reverse}, params: {params}")

            response = seq_service(
                command=command,
                mode=mode,
                loop=loop,
                reverse=reverse,
                params=params
            )

            if response.success:
                self.statusBar().showMessage(
                    f"Sequence command success: {response.message}", 3000)
                if command in ['info', 'state'] and response.data_keys:
                    self._update_sequence_status(
                        dict(zip(response.data_keys, response.data_values)))
            else:
                self._show_error("Sequence Error",
                                 f"Command failed: {response.message}\nCommand: {command}\n"
                                 f"Mode: {mode}\nParams: {params}")

        except rospy.ServiceException as e:
            self._show_error("Service Error", f"Service call failed: {str(e)}")
        except Exception as e:
            self._show_error("Error", f"Unexpected error: {str(e)}")

    def _update_sequence_status(self, status_data: Dict[str, str]):
        """更新序列状态显示

        Args:
            status_data: 状态数据字典
        """
        if not status_data:
            return

        status_text = []
        # 格式化重要的状态信息
        if 'total_points' in status_data:
            status_text.append(f"Total Points: {status_data['total_points']}")
        if 'current_index' in status_data:
            status_text.append(
                f"Current Index: {status_data['current_index']}")
        if 'mode' in status_data:
            status_text.append(f"Mode: {status_data['mode']}")
        if 'is_running' in status_data:
            status_text.append(f"Running: {status_data['is_running']}")
        if 'is_paused' in status_data:
            status_text.append(f"Paused: {status_data['is_paused']}")

        self.seq_status_label.setText("\n".join(status_text))

    def _show_error(self, title: str, message: str):
        """显示错误对话框

        Args:
            title: 错误标题
            message: 错误信息
        """
        QMessageBox.critical(self, title, message)
        self.statusBar().showMessage(f"Error: {message}", 5000)

    def closeEvent(self, event):
        """窗口关闭事件"""
        # 取消所有话题订阅
        self.status_sub.unregister()
        self.result_sub.unregister()
        self.feedback_sub.unregister()
        super().closeEvent(event)


def main():
    """主函数"""
    app = QApplication(sys.argv)

    # 设置应用程序样式
    app.setStyle('Fusion')

    window = NavigationControllerUI()
    window.show()

    try:
        sys.exit(app.exec())
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
