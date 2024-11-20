#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
from enum import Enum
from typing import List, Union, Tuple, Optional
from geometry_msgs.msg import Pose
from wp_nav_controller.navigation_control import NavigationControl, NavigationState


class SequenceMode(str, Enum):
    """导航序列模式枚举类

    定义了三种基本导航序列模式：
        - SINGLE: 单程导航，按顺序导航到所有航点后停止
        - LOOP: 循环导航，到达最后一个航点后回到第一个航点继续导航
        - ROUNDTRIP: 往返导航，到达最后一个航点后按相反顺序返回
    """
    SINGLE = "SINGLE"         # 单程导航模式
    LOOP = "LOOP"            # 循环导航模式
    ROUNDTRIP = "ROUNDTRIP"  # 往返导航模式


class NavigationPoint:
    """导航点类

    封装导航目标点的信息，支持通过位姿或航点名称/索引创建导航点

    Attributes:
        pose (Pose): 目标位姿
        name (str): 航点名称（可选）
        index (int): 航点索引（可选）
    """

    def __init__(self,
                 pose: Optional[Pose] = None,
                 name: Optional[str] = None,
                 index: Optional[int] = None):
        """初始化导航点

        Args:
            pose (Pose, optional): 目标位姿
            name (str, optional): 航点名称
            index (int, optional): 航点索引
        """
        self.pose = pose
        self.name = name
        self.index = index

    def __str__(self) -> str:
        """返回导航点的字符串表示

        Returns:
            str: 导航点描述（优先使用名称，其次是索引，最后是位姿坐标）
        """
        if self.name:
            return f"Waypoint(name={self.name})"
        elif self.index is not None:
            return f"Waypoint(index={self.index})"
        else:
            return f"Pose(x={self.pose.position.x:.2f}, y={self.pose.position.y:.2f})"


class NavigationSequence:
    """导航序列控制类

    提供航点序列导航的高级控制功能，包括：
    1. 序列管理：添加、移除、重排序导航点
    2. 导航模式：支持单程、循环、往返三种基本模式
    3. 序列控制：开始、暂停、继续、停止序列导航
    4. 状态反馈：提供序列执行状态和进度信息
    """

    class PauseState:
        """暂停状态记录类"""
        def __init__(self):
            # 序列配置
            self.sequence = []          # 当前执行的序列
            self.current_index = 0      # 当前执行到的索引

            # 导航模式
            self.mode = None           # 导航模式
            self.is_loop = False       # 是否循环执行
            self.is_roundtrip = False  # 是否往返执行
            self.is_reverse = False    # 是否反向导航

            # 目标信息
            self.current_goal = None   # 当前导航目标点

            # 统计信息
            self.roundtrip_count = 0   # 往返计数

    def __init__(self, navigator: NavigationControl):
        """初始化导航序列控制器

        Args:
            navigator (NavigationControl): 导航控制器实例
        """
        self._navigator = navigator
        self._sequence: List[NavigationPoint] = []
        self._current_index = 0  # 当前执行的航点索引
        self._roundtrip_count = 0  # 完整往返循环次数（前进+返回）
        self._is_running = False
        self._is_paused = False
        self._is_loop = False
        self._is_roundtrip = False
        self._sequence_thread = None
        self._sequence_lock = threading.Lock()
        self._mode = SequenceMode.SINGLE
        self._reverse_direction = False  # 控制导航方向
        self._feedback_callback = None
        self._reverse_enabled = False    # 是否启用反向导航
        self._paused_state = None  # 存储暂停状态
        self._just_resumed = False  # 标记是否刚从暂停恢复

    def set_feedback_callback(self, callback):
        """设置序列导航的反馈回调函数

        Args:
            callback (callable): 回调函数，接收参数：(current_point, sequence_info)
                current_point: 当前导航点
                sequence_info: 序列信息字典
        """
        self._feedback_callback = callback

    def clear_sequence(self):
        """清空导航序列"""
        with self._sequence_lock:
            self._sequence.clear()
            self._current_index = 0
        rospy.loginfo("Navigation sequence cleared")

    def add_pose(self, pose: Pose) -> bool:
        """添加位姿到导航序列

        Args:
            pose (Pose): 目标位姿

        Returns:
            bool: 是否成功添加位姿到序列
        """
        with self._sequence_lock:
            try:
                self._sequence.append(NavigationPoint(pose=pose))
                rospy.loginfo(
                    f"Added pose to sequence at index {len(self._sequence)-1}")
                return True
            except Exception as e:
                rospy.logerr(f"Failed to add pose to sequence: {e}")
                return False

    def add_waypoint_by_name(self, name: str) -> bool:
        """通过名称添加航点到导航序列

        Args:
            name (str): 航点名称

        Returns:
            bool: 是否成功添加航点到序列
        """
        with self._sequence_lock:
            try:
                pose = self._navigator._waypoint_manager.get_waypoint_by_name(
                    name)
                if pose:
                    self._sequence.append(
                        NavigationPoint(pose=pose, name=name))
                    rospy.loginfo(
                        f"Added waypoint '{name}' to sequence at index {len(self._sequence)-1}")
                    return True
                else:
                    rospy.logerr(f"Waypoint '{name}' not found")
                    return False
            except Exception as e:
                rospy.logerr(
                    f"Failed to add waypoint '{name}' to sequence: {e}")
                return False

    def add_waypoint_by_index(self, index: int) -> bool:
        """通过索引添加航点到导航序列

        Args:
            index (int): 航点索引

        Returns:
            bool: 是否成功添加航点到序列
        """
        with self._sequence_lock:
            try:
                waypoint = self._navigator._waypoint_manager.get_waypoint_by_index(
                    index)
                if waypoint:
                    name, pose = waypoint
                    self._sequence.append(NavigationPoint(
                        pose=pose, name=name, index=index))
                    rospy.loginfo(
                        f"Added waypoint {index} ('{name}') to sequence at index {len(self._sequence)-1}")
                    return True
                else:
                    rospy.logerr(f"Waypoint at index {index} not found")
                    return False
            except Exception as e:
                rospy.logerr(
                    f"Failed to add waypoint {index} to sequence: {e}")
                return False

    def set_sequence(self, points: List[Union[Pose, Tuple[str, None], Tuple[None, int]]]) -> bool:
        """设置完整的导航序列

        Args:
            points: 导航点列表，每个元素可以是：
                   - Pose: 直接的位姿
                   - (str, None): 航点名称
                   - (None, int): 航点索引

        Returns:
            bool: 是否成功设置序列
        """
        try:
            self.clear_sequence()
            for point in points:
                if isinstance(point, Pose):
                    self.add_pose(point)
                elif isinstance(point, tuple):
                    if isinstance(point[0], str):
                        self.add_waypoint_by_name(point[0])
                    elif isinstance(point[1], int):
                        self.add_waypoint_by_index(point[1])
            return True
        except Exception as e:
            rospy.logerr(f"Failed to set sequence: {e}")
            return False

    def reorder_sequence(self, new_indices: List[int], reverse: bool = False) -> bool:
        """重新排序导航序列

        Args:
            new_indices (List[int]): 新的索引顺序列表
            reverse (bool): 是否反转排序结果

        Returns:
            bool: 是否成功重排序
        """
        if self._is_running:
            rospy.logwarn("Cannot reorder sequence while it is running")
            return False

        with self._sequence_lock:
            try:
                # 检查索引有效性
                if not all(0 <= i < len(self._sequence) for i in new_indices):
                    rospy.logerr("Invalid indices for reordering")
                    return False

                # 按指定顺序重排序列
                new_sequence = [self._sequence[i] for i in new_indices]

                # 如果需要反转
                if reverse:
                    new_sequence.reverse()

                self._sequence = new_sequence
                rospy.loginfo("Sequence reordered" +
                              (" (reversed)" if reverse else ""))
                return True

            except Exception as e:
                rospy.logerr(f"Failed to set sequence order: {e}")
                return False

    def reverse_sequence(self) -> bool:
        """反转当前导航序列的顺序

        Returns:
            bool: 是否成功反转序列
        """
        if self._is_running:
            rospy.logwarn("Cannot reverse sequence while it is running")
            return False

        with self._sequence_lock:
            try:
                self._sequence.reverse()
                rospy.loginfo("Navigation sequence reversed")
                return True
            except Exception as e:
                rospy.logerr(f"Failed to reverse sequence: {e}")
                return False

    def remove_point(self, index: int) -> bool:
        """从序列中移除指定索引的导航点

        Args:
            index (int): 要移除的导航点索引

        Returns:
            bool: 是否成功移除
        """
        if self._is_running:
            rospy.logwarn("Cannot remove point while sequence is running")
            return False

        with self._sequence_lock:
            try:
                if 0 <= index < len(self._sequence):
                    removed_point = self._sequence.pop(index)
                    rospy.loginfo(
                        f"Removed point {removed_point} at index {index}")
                    return True
                else:
                    rospy.logerr(f"Invalid index {index}")
                    return False
            except Exception as e:
                rospy.logerr(f"Failed to remove point: {e}")
                return False

    def insert_point(self, index: int, point: NavigationPoint) -> bool:
        """在指定索引位置插入导航点

        Args:
            index (int): 插入位置的索引
            point (NavigationPoint): 要插入的导航点

        Returns:
            bool: 是否成功插入
        """
        if self._is_running:
            rospy.logwarn("Cannot insert point while sequence is running")
            return False

        with self._sequence_lock:
            try:
                if 0 <= index <= len(self._sequence):
                    self._sequence.insert(index, point)
                    rospy.loginfo(f"Inserted {point} at index {index}")
                    return True
                else:
                    rospy.logerr(f"Invalid insert index {index}")
                    return False
            except Exception as e:
                rospy.logerr(f"Failed to insert point: {e}")
                return False

    def get_sequence_info(self) -> dict:
        """获取当前导航序列的信息

        Returns:
            dict: 包含序列信息的字典
                {
                    'total_points': int,       # 序列中的总点数
                    'current_index': int,      # 当前导航点索引
                    'mode': SequenceMode,      # 当前导航模式
                    'is_running': bool,        # 是否正在运行
                    'is_paused': bool,         # 是否暂停
                    'is_loop': bool,          # 是否循环模式
                    'is_roundtrip': bool,      # 是否往返模式
                    'points': list             # 导航点列表
                }
        """
        with self._sequence_lock:
            return {
                'total_points': len(self._sequence),
                'current_index': self._current_index,
                'mode': self._mode,
                'is_running': self._is_running,
                'is_paused': self._is_paused,
                'is_loop': self._is_loop,
                'is_roundtrip': self._is_roundtrip,
                'points': [str(point) for point in self._sequence]
            }

    def start_sequence(self, mode: SequenceMode = SequenceMode.SINGLE,
                      is_loop: bool = False, is_roundtrip: bool = False, reverse: bool = False,
                      start_point: Union[str, int] = None) -> bool:
        """启动序列导航

        Args:
            mode (SequenceMode): 导航模式，默认为单程模式
            is_loop (bool): 是否循环执行序列（对LOOP和ROUNDTRIP模式有效）
            is_roundtrip (bool): 是否为往返模式（对LOOP和ROUNDTRIP模式有效）
            reverse (bool): 是否启用反向导航（对所有模式有效）
            start_point: 起始点，可以是航点名称或索引。None表示从头开始

        Returns:
            bool: 是否成功启动序列导航
        """
        if self._is_running:
            rospy.logwarn("Navigation sequence is already running")
            return False

        if not self._sequence:
            rospy.logerr("Navigation sequence is empty")
            return False

        try:
            # 设置起始索引
            if start_point is not None:
                start_index = self._find_start_index(start_point)
                if start_index is None:
                    return False
                self._current_index = start_index
            else:
                self._current_index = len(self._sequence) - 1 if reverse else 0

            self._mode = mode
            self._is_loop = is_loop or mode == SequenceMode.LOOP
            self._is_roundtrip = is_roundtrip or mode == SequenceMode.ROUNDTRIP
            self._is_running = True
            self._is_paused = False
            self._reverse_direction = reverse
            self._reverse_enabled = reverse

            self._roundtrip_count = 0  # 重置完整往返计数

            # 如果是反向启动，翻转序列
            if reverse:
                with self._sequence_lock:
                    self._sequence.reverse()
                    # 更新起始索引位置
                    if start_point is not None:
                        self._current_index = len(self._sequence) - 1 - self._current_index

            # 启动序列导航线程
            self._sequence_thread = threading.Thread(
                target=self._sequence_execution_thread
            )
            self._sequence_thread.start()

            mode_str = f"{mode}"
            config_str = []
            if is_loop:
                config_str.append("looped")
            if reverse:
                config_str.append("reverse")
            if start_point is not None:
                config_str.append(f"start from {start_point}")

            status_str = f" ({', '.join(config_str)})" if config_str else ""
            rospy.loginfo(f"Started {mode_str} navigation{status_str}")
            return True

        except Exception as e:
            rospy.logerr(f"Failed to start sequence: {e}")
            self._is_running = False
            return False

    def _find_start_index(self, start_point) -> Optional[int]:
        """查找起始点在序列中的索引

        Args:
            start_point: 航点名称或索引

        Returns:
            int: 找到的索引，未找到返回None
        """
        try:
            if isinstance(start_point, int):
                if 0 <= start_point < len(self._sequence):
                    return start_point
                rospy.logerr(f"Invalid start index: {start_point}")
            else:
                for i, point in enumerate(self._sequence):
                    if point.name == start_point:
                        return i
                rospy.logerr(f"Start point not found in sequence: {start_point}")
            return None
        except Exception as e:
            rospy.logerr(f"Error finding start point: {e}")
            return None

    def pause_sequence(self) -> bool:
        """暂停序列导航，并保存必要的恢复信息

        Returns:
            bool: 是否成功暂停
        """
        if not self._is_running or self._is_paused:
            rospy.logwarn("Cannot pause: sequence not running or already paused")
            return False

        try:
            # 暂停当前导航
            if not self._navigator.pause_navigation():
                return False

            # 保存恢复导航必需的状态
            pause_state = self.PauseState()

            with self._sequence_lock:
                # 保存序列配置、索引和目标
                pause_state.sequence = self._sequence.copy()
                pause_state.current_index = self._current_index
                pause_state.current_goal = self._sequence[self._current_index]

                # 保存导航模式
                pause_state.mode = self._mode
                pause_state.is_loop = self._is_loop
                pause_state.is_roundtrip = self._is_roundtrip
                pause_state.is_reverse = self._reverse_direction

                # 保存往返计数
                pause_state.roundtrip_count = self._roundtrip_count

                self._paused_state = pause_state
                self._is_paused = True

            rospy.loginfo(
                f"Sequence paused at index {self._current_index}, "
                f"target: {pause_state.current_goal},"
                f"roundtrip count: {pause_state.roundtrip_count}"
            )
            return True

        except Exception as e:
            rospy.logerr(f"Failed to pause sequence: {e}")
            return False

    def get_pause_state(self) -> dict:
        """获取暂停状态信息"""
        if not self._is_paused or not self._paused_state:
            return None

        return {
            'current_index': self._paused_state.current_index,
            'total_points': len(self._paused_state.sequence),
            'mode': self._paused_state.mode,
            'is_loop': self._paused_state.is_loop,
            'is_reverse': self._paused_state.is_reverse,
            'current_goal': str(self._paused_state.current_goal)
        }

    def resume_sequence(self) -> bool:
        """加载暂停时保存的导航任务，恢复序列导航

        Returns:
            bool: 是否成功恢复
        """
        # 仅检查运行和暂停状态
        if not self._is_running or not self._is_paused:
            rospy.logwarn("Cannot resume: sequence not running or not paused")
            return False

        try:
            with self._sequence_lock:
                # 恢复序列配置
                self._sequence = self._paused_state.sequence
                self._current_index = self._paused_state.current_index

                # 恢复导航模式
                self._mode = self._paused_state.mode
                self._is_loop = self._paused_state.is_loop
                self._is_roundtrip = self._paused_state.is_roundtrip
                self._reverse_direction = self._paused_state.is_reverse

                # 恢复往返计数
                self._roundtrip_count = self._paused_state.roundtrip_count

                # 如果索引超出序列范围，则直接停止
                if self._current_index >= len(self._sequence):
                    rospy.logwarn(
                        f"Cannot resume: sequence completed at index {self._current_index} "
                        f"with total points {len(self._sequence)}"
                    )
                    self._is_running = False
                    self._paused_state = None
                    return False

                # 标记为刚恢复状态
                self._just_resumed = True

                # 恢复导航
                if self._navigator.resume_navigation():
                    self._is_paused = False
                    self._paused_state = None

                    # 重启序列执行线程
                    self._sequence_thread = threading.Thread(
                        target=self._sequence_execution_thread
                    )
                    self._sequence_thread.start()

                    rospy.loginfo(
                        f"Sequence resumed from index {self._current_index}, "
                        f"continuing to target: {self._sequence[self._current_index]}"
                    )
                    return True

                rospy.logerr(f"Failed to resume navigation to {self._sequence[self._current_index]}")
                return False

        except Exception as e:
            rospy.logerr(f"Failed to resume sequence: {e}")
            return False

    def stop_sequence(self) -> bool:
        """停止序列导航

        Returns:
            bool: 是否成功停止
        """
        if not self._is_running:
            rospy.logwarn("No running sequence to stop")
            return False

        try:
            self._is_running = False
            self._is_paused = False
            return self._navigator.stop_navigation()
        except Exception as e:
            rospy.logerr(f"Failed to stop sequence: {e}")
            return False

    def _sequence_execution_thread(self):
        """序列导航执行线程

        负责：
        1. 按照指定模式执行导航序列
        2. 处理导航结果
        3. 响应暂停/继续/停止命令
        4. 管理序列状态和进度
        5. 提供导航反馈
        """
        while self._is_running and not rospy.is_shutdown():
            if self._is_paused:
                rospy.sleep(0.1)  # 暂停时等待
                continue

            try:
                # 如果当前是IDLE状态，开始导航到当前点
                if self._navigator.get_state() == NavigationState.IDLE:
                    if not self._navigate_to_current_point():
                        if not self._is_paused:
                            break
                        continue

                # 等待导航完成或暂停
                while self._navigator.get_state() == NavigationState.NAVIGATING:
                    if not self._is_running or self._is_paused:
                        break
                    rospy.sleep(0.1)

                # 如果暂停,继续等待
                if self._is_paused:
                    continue

                # 导航完成时更新序列
                if self._navigator.get_state() == NavigationState.IDLE:
                    if not self._update_sequence_index():
                        if not self._is_loop or not self._is_roundtrip or not self._is_running:
                            break
                        continue

            except Exception as e:
                rospy.logerr(f"Error in sequence execution: {e}")
                if not self._is_paused:  # 只有非暂停状态才退出
                    self._is_running = False
                    break
                continue  # 暂停状态下继续等待

        # 仅在非暂停状态下清理
        if not self._is_paused:
            self._cleanup_sequence()

    def _navigate_to_current_point(self) -> bool:
        """导航到当前点"""
        try:
            current_point = self._sequence[self._current_index]

            # 提供导航反馈
            self._provide_feedback(current_point)

            # 开始导航
            success = self._navigator.navigate_to_pose(current_point.pose)
            if not success:
                rospy.logerr(f"Failed to navigate to {current_point}")
                self._is_running = False
                return False

            # 等待导航完成
            while self._navigator.get_state() == NavigationState.NAVIGATING:
                if not self._is_running:
                    return False
                rospy.sleep(0.1)

            return True

        except Exception as e:
            rospy.logerr(f"Error during navigation: {e}")
            self._is_running = False
            return False

    def _update_sequence_index(self) -> bool:
        """更新导航序列的索引"""
        with self._sequence_lock:
            # 先检查是否需要更新
            if self._navigator.get_state() not in [NavigationState.IDLE, NavigationState.NAVIGATING]:
                return True  # 继续执行序列

            if self._mode == SequenceMode.ROUNDTRIP:
                return self._handle_roundtrip_mode()
            return self._handle_single_or_loop_mode()

    def _handle_roundtrip_mode(self) -> bool:
        """处理往返模式的索引更新"""
        if not self._reverse_direction:
            # 正向导航逻辑
            if self._current_index == len(self._sequence) - 1:
                # 到达终点，准备返程
                self._reverse_direction = True
                self._current_index -= 1  # 从倒数第二个点开始返程
                rospy.loginfo("Reached end point, starting return trip")
                # 继续执行序列
                return True
            else:
                self._current_index += 1
                # 继续执行序列
                return True
        else:
            # 反向导航逻辑
            if self._current_index == 0:
                # 完成返程
                self._roundtrip_count += 1  # 完整往返次数+1
                rospy.loginfo(f"Roundtrip loop completed (count: {self._roundtrip_count})")

                # 已完成往返，判断是否继续
                if not self._is_loop and not self._is_roundtrip and not self._is_paused:
                    self._is_running = False
                    return False

                # 重置为正向导航，从第二个点开始新的往返
                self._reverse_direction = False
                self._current_index = 1
                return True  # 继续执行
            else:
                self._current_index -= 1
                return True  # 继续执行

    def _handle_single_or_loop_mode(self) -> bool:
        """处理单程或循环模式的索引更新"""
        step = -1 if self._reverse_direction else 1
        self._current_index += step

        if self._reverse_direction and self._current_index < 0:
            if self._mode == SequenceMode.LOOP:
                self._current_index = len(self._sequence) - 1
                rospy.loginfo("Reached start point, continuing reverse loop")
            else:
                rospy.loginfo("Single trip reverse navigation completed")
                self._is_running = False
                return False

        elif not self._reverse_direction and self._current_index >= len(self._sequence):
            if self._mode == SequenceMode.LOOP:
                self._current_index = 0
                rospy.loginfo("Reached end point, continuing forward loop")
            else:
                rospy.loginfo("Single trip forward navigation completed")
                self._is_running = False
                return False

        return True

    def _provide_feedback(self, current_point):
        """提供导航反馈"""
        if self._feedback_callback:
            try:
                self._feedback_callback(current_point, self.get_sequence_info())
            except Exception as e:
                rospy.logerr(f"Error in feedback callback: {e}")

    def _cleanup_sequence(self):
        """清理序列状态"""
        # 如果是反向模式且不是暂停状态，恢复序列原始顺序
        if self._reverse_enabled and not self._is_paused:
            with self._sequence_lock:
                self._sequence.reverse()

        # 仅在非暂停状态下重置状态
        if not self._is_paused:
            self._current_index = 0
            self._reverse_direction = False
            self._reverse_enabled = False