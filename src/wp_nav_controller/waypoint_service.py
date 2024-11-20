#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from amr_map_tools.srv import *


class WaypointException(Exception):
    """航点服务异常类，用于处理航点服务相关的特定异常。

    Attributes:
        message (str): 异常信息。
        service_name (str): 发生异常的服务名称。
    """

    def __init__(self, message, service_name=None):
        """初始化航点服务异常。

        Args:
            message (str): 异常信息。
            service_name (str, optional): 发生异常的服务名称。默认为 None。
        """
        super().__init__(message)
        self.service_name = service_name


class WaypointInfoService:
    """航点信息服务类，提供航点信息的查询功能。

    该类负责与ROS服务进行通信，获取航点相关的信息，包括航点总数、
    指定索引的航点信息以及根据名称查询航点信息。

    Attributes:
        _service_initialized (bool): 服务是否已初始化的标志。
        timeout (float): 等待服务的超时时间。
        _get_num_client: 获取航点数量的服务客户端。
        _get_wp_index_client: 根据索引获取航点的服务客户端。
        _get_wp_name_client: 根据名称获取航点的服务客户端。

    Methods:
        get_waypoint_count(): 获取航点总数。
        get_waypoint_by_index(index): 根据索引获取航点信息。
        get_waypoint_by_name(name): 根据名称获取航点信息。
    """

    def __init__(self, timeout=None):
        """初始化航点信息服务。

        Args:
            timeout (float, optional): 等待服务的超时时间（秒），None 表示无限等待。
        """
        self._service_initialized = False
        self.timeout = timeout

    def _ensure_initialized(self):
        """确保服务已经初始化。

        如果服务尚未初始化，则会进行初始化过程，包括等待服务可用和创建服务客户端。
        如果初始化失败，将抛出 WaypointException 异常。

        Raises:
            WaypointException: 当服务初始化失败时抛出。
        """
        if self._service_initialized:
            rospy.logdebug("Waypoint info service already initialized, skipping initialization")
            return

        try:
            rospy.loginfo("Initializing waypoint info service...")

            # 等待服务可用
            rospy.wait_for_service('waypoint/get_num_waypoint', timeout=self.timeout)
            rospy.wait_for_service('waypoint/get_waypoint_index', timeout=self.timeout)
            rospy.wait_for_service('waypoint/get_waypoint_name', timeout=self.timeout)

            # 创建服务客户端
            self._get_num_client = rospy.ServiceProxy('waypoint/get_num_waypoint', GetNumOfWaypoints)
            self._get_wp_index_client = rospy.ServiceProxy('waypoint/get_waypoint_index', GetWaypointByIndex)
            self._get_wp_name_client = rospy.ServiceProxy('waypoint/get_waypoint_name', GetWaypointByName)

            # 初始化标志
            self._service_initialized = True

            rospy.logdebug("WaypointInfoService initialized")
            rospy.loginfo("Waypoint info service initialized successfully")

        except rospy.ROSException as e:
            raise WaypointException("Failed to initialize waypoint info service: %s" % str(e), 'info')

    def get_waypoint_count(self):
        """获取系统中的航点总数。

        Returns:
            int: 航点总数。如果获取失败则返回 None。
        """
        self._ensure_initialized()
        try:
            response = self._get_num_client()
            return response.num
        except rospy.ServiceException as e:
            rospy.logerr("Failed to get waypoint count: %s" % str(e))
            return None

    def get_waypoint_by_index(self, index):
        """根据索引获取航点信息。

        Args:
            index (int): 航点的索引值。

        Returns:
            tuple: 包含航点名称和位姿的元组 (name, pose)，获取失败时返回 None。
        """
        self._ensure_initialized()
        try:
            response = self._get_wp_index_client(index)
            return response.name, response.pose
        except rospy.ServiceException as e:
            rospy.logerr("Failed to get waypoint by index %d: %s" %
                         (index, str(e)))
            return None

    def get_waypoint_by_name(self, name):
        """根据名称获取航点信息。

        Args:
            name (str): 航点的名称。

        Returns:
            geometry_msgs/Pose: 航点的位姿信息，获取失败时返回 None。
        """
        self._ensure_initialized()
        try:
            response = self._get_wp_name_client(name)
            return response.pose
        except rospy.ServiceException as e:
            rospy.logerr("Failed to get waypoint by name %s: %s" %
                         (name, str(e)))
            return None


class WaypointSaveService:
    """航点保存服务类，提供将航点信息保存到文件的功能。

    该类负责与ROS服务进行通信，将当前系统中的航点信息保存到指定的文件中。

    Attributes:
        _service_initialized (bool): 服务是否已初始化的标志。
        timeout (float): 等待服务的超时时间。
        _save_client: 保存航点的服务客户端。

    Methods:
        save_waypoints(filename): 将航点信息保存到指定文件。
    """

    def __init__(self, timeout=None):
        """初始化航点保存服务。

        Args:
            timeout (float, optional): 等待服务的超时时间（秒），None 表示无限等待。
        """
        self._service_initialized = False
        self.timeout = timeout

    def _ensure_initialized(self):
        """确保服务已经初始化。

        如果服务尚未初始化，则会进行初始化过程，包括等待服务可用和创建服务客户端。
        如果初始化失败，将抛出 WaypointException 异常。

        Raises:
            WaypointException: 当服务初始化失败时抛出。
        """
        if self._service_initialized:
            rospy.logdebug("Waypoint save service already initialized, skipping initialization")
            return

        try:
            rospy.loginfo("Initializing waypoint save service...")

            # 等待服务可用
            rospy.wait_for_service('waypoint/save_waypoints', timeout=self.timeout)

            # 创建服务客户端
            self._save_client = rospy.ServiceProxy('waypoint/save_waypoints', SaveWaypoints)

            # 初始化标志
            self._service_initialized = True

            rospy.logdebug("WaypointSaveService initialized")
            rospy.loginfo("Waypoint save service initialized successfully")
        except rospy.ROSException as e:
            raise WaypointException("Failed to initialize waypoint save service: %s" % str(e), 'save')

    def save_waypoints(self, filename):
        """将航点信息保存到指定的文件中。

        Args:
            filename (str): 保存航点信息的文件路径。

        Returns:
            bool: 保存成功返回 True，失败返回 False。
        """
        self._ensure_initialized()
        try:
            self._save_client(filename)
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Failed to save waypoints: %s" % str(e))
            return False


class WaypointNavigationService:
    """航点导航服务类，提供机器人导航控制功能。

    该类负责发布导航目标和接收导航结果的反馈，支持导航到指定名称的航点
    或指定的位姿。

    Attributes:
        _service_initialized (bool): 服务是否已初始化的标志。
        _navi_result_callback: 导航结果的回调函数。
        _navi_waypoint_pub: 发布航点名称的发布者。
        _navi_pose_pub: 发布位姿的发布者。

    Methods:
        set_navi_result_callback(callback): 设置导航结果的回调函数。
        navigate_to_waypoint(waypoint_name): 导航到指定名称的航点。
        navigate_to_pose(pose): 导航到指定的位姿。
    """

    def __init__(self):
        """初始化航点导航服务"""
        self._service_initialized = False
        self._navi_result_callback = None

    def _ensure_initialized(self):
        """确保服务已经初始化。

        如果服务尚未初始化，则会进行初始化过程，包括创建发布者和订阅者。
        如果初始化失败，将抛出 WaypointException 异常。

        Raises:
            WaypointException: 当服务初始化失败时抛出。
        """
        if self._service_initialized:
            rospy.logdebug("Waypoint navigation service already initialized, skipping initialization")
            return

        try:
            rospy.loginfo("Initializing waypoint navigation service...")

            # 发布导航目标
            self._navi_waypoint_pub = rospy.Publisher(
                'waypoint/navi_waypoint',
                String,
                queue_size=10
            )
            self._navi_pose_pub = rospy.Publisher(
                'waypoint/navi_pose',
                Pose,
                queue_size=10
            )

            # 订阅导航结果
            rospy.Subscriber(
                'waypoint/navi_result',
                String,
                self._on_navi_result
            )

            # 初始化标志
            self._service_initialized = True

            rospy.logdebug("WaypointNavigationService initialized")
            rospy.loginfo("Waypoint navigation service initialized successfully")
        except Exception as e:
            raise WaypointException("Failed to initialize navigation service: %s" % str(e), 'navigation')

    def set_navi_result_callback(self, callback):
        """设置导航结果的回调函数。

        Args:
            callback (callable): 处理导航结果的回调函数，接收一个字符串参数表示导航结果。
        """
        self._ensure_initialized()
        self._navi_result_callback = callback

    def _on_navi_result(self, msg):
        """处理接收到的导航结果。

        Args:
            msg (std_msgs/String): 导航结果消息。
        """
        if self._navi_result_callback:
            self._navi_result_callback(msg.data)

    def navigate_to_waypoint(self, waypoint_name):
        """导航到指定名称的航点。

        Args:
            waypoint_name (str): 目标航点的名称。

        Returns:
            bool: 发送导航命令成功返回 True，失败返回 False。
        """
        self._ensure_initialized()
        try:
            msg = String(waypoint_name)
            self._navi_waypoint_pub.publish(msg)
            return True
        except Exception as e:
            rospy.logerr("Failed to navigate to waypoint: %s" % str(e))
            return False

    def navigate_to_pose(self, pose):
        """导航到指定的位姿。

        Args:
            pose (geometry_msgs/Pose): 目标位姿。

        Returns:
            bool: 发送导航命令成功返回 True，失败返回 False。
        """
        self._ensure_initialized()
        try:
            self._navi_pose_pub.publish(pose)
            return True
        except Exception as e:
            rospy.logerr("Failed to navigate to pose: %s" % str(e))
            return False


class Singleton(type):
    """单例模式元类，用于确保类只有一个实例。

    该元类通过维护一个实例字典来实现单例模式，确保每个使用此元类的类
    在整个程序运行期间只创建一个实例。

    Attributes:
        _instances (dict): 存储类及其唯一实例的字典。
    """
    _instances = {}

    def __call__(cls, *args, **kwargs):
        """创建或返回类的唯一实例。

        当尝试创建类的新实例时，首先检查该类的实例是否已存在。如果存在，
        则返回现有实例；如果不存在，则创建新实例并存储。

        Args:
            *args: 位置参数。
            **kwargs: 关键字参数。

        Returns:
            object: 类的唯一实例。
        """
        if cls not in cls._instances:
            cls._instances[cls] = super(
                Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class WaypointServiceManager(metaclass=Singleton):
    """航点服务管理器，统一管理所有航点相关服务的单例类。

    该类使用单例模式，确保在整个系统中只有一个服务管理器实例。它封装了
    航点信息查询、保存和导航等所有功能，提供统一的接口进行访问。

    Attributes:
        _initialized (bool): 管理器是否已初始化的标志。
        _info_service (WaypointInfoService): 航点信息服务实例。
        _save_service (WaypointSaveService): 航点保存服务实例。
        _navigation_service (WaypointNavigationService): 航点导航服务实例。

    Properties:
        node_name (str): 获取当前ROS节点名称。
    """

    def __init__(self, node_name='waypoint_service_node', services=None, timeout=None):
        """初始化航点服务管理器。

        Args:
            node_name (str, optional): ROS节点名称。默认为'waypoint_service_node'。
            services (list, optional): 需要初始化的服务列表，可选值：['info', 'save', 'navigation']。
                                     默认为None，表示初始化所有服务。
            timeout (float, optional): 等待服务的超时时间（秒），None表示无限等待。

        Raises:
            Exception: 初始化失败时抛出的异常。
        """
        # 确保只初始化一次
        if hasattr(self, '_initialized'):
            return
        self._initialized = True

        try:
            # 初始化ROS节点
            if not rospy.get_node_uri():
                rospy.init_node(node_name, anonymous=True)
                rospy.loginfo("Initialized ROS node: %s" % rospy.get_name())

            # 初始化请求的服务
            self._init_services(services, timeout)

            rospy.loginfo("Waypoint service manager initialized successfully")

        except Exception as e:
            rospy.logerr("Failed to initialize service manager: %s" % str(e))
            raise

    def _init_services(self, services=None, timeout=None):
        """初始化指定的服务。

        Args:
            services (list, optional): 需要初始化的服务列表。默认为None，表示初始化所有服务。
            timeout (float, optional): 等待服务的超时时间。
        """
        # 默认初始化所有服务
        if services is None:
            services = ['info', 'save', 'navigation']

        # 创建服务实例（但不立即初始化）
        if 'info' in services:
            self._info_service = WaypointInfoService(timeout)
        if 'save' in services:
            self._save_service = WaypointSaveService(timeout)
        if 'navigation' in services:
            self._navigation_service = WaypointNavigationService()

    def _ensure_info_service(self):
        """确保航点信息服务可用。

        如果服务尚未创建，则创建新的服务实例。
        """
        if not hasattr(self, '_info_service'):
            self._info_service = WaypointInfoService()

    def _ensure_save_service(self):
        """确保航点保存服务可用。

        如果服务尚未创建，则创建新的服务实例。
        """
        if not hasattr(self, '_save_service'):
            self._save_service = WaypointSaveService()

    def _ensure_navigation_service(self):
        """确保航点导航服务可用。

        如果服务尚未创建，则创建新的服务实例。
        """
        if not hasattr(self, '_navigation_service'):
            self._navigation_service = WaypointNavigationService()

    # 航点信息服务接口
    def get_waypoint_count(self):
        """获取系统中的航点总数。

        Returns:
            int: 航点总数。如果获取失败则返回 None。
        """
        self._ensure_info_service()
        return self._info_service.get_waypoint_count()

    def get_waypoint_by_index(self, index):
        """根据索引获取航点信息。

        Args:
            index (int): 航点的索引值。

        Returns:
            tuple: 包含航点名称和位姿的元组 (name, pose)，获取失败时返回 None。
        """
        self._ensure_info_service()
        return self._info_service.get_waypoint_by_index(index)

    def get_waypoint_by_name(self, name):
        """根据名称获取航点信息。

        Args:
            name (str): 航点的名称。

        Returns:
            geometry_msgs/Pose: 航点的位姿信息，获取失败时返回 None。
        """
        self._ensure_info_service()
        return self._info_service.get_waypoint_by_name(name)

    # 航点保存服务接口
    def save_waypoints(self, filename):
        """将航点信息保存到指定的文件中。

        Args:
            filename (str): 保存航点信息的文件路径。

        Returns:
            bool: 保存成功返回 True，失败返回 False。
        """
        self._ensure_save_service()
        return self._save_service.save_waypoints(filename)

    # 航点导航服务接口
    def set_navi_result_callback(self, callback):
        """设置导航结果的回调函数。

        Args:
            callback (callable): 处理导航结果的回调函数，接收一个字符串参数表示导航结果。
        """
        self._ensure_navigation_service()
        self._navigation_service.set_navi_result_callback(callback)

    def navigate_to_waypoint(self, waypoint_name):
        """导航到指定名称的航点。

        Args:
            waypoint_name (str): 目标航点的名称。

        Returns:
            bool: 发送导航命令成功返回 True，失败返回 False。
        """
        self._ensure_navigation_service()
        return self._navigation_service.navigate_to_waypoint(waypoint_name)

    def navigate_to_pose(self, pose):
        """导航到指定的位姿。

        Args:
            pose (geometry_msgs/Pose): 目标位姿。

        Returns:
            bool: 发送导航命令成功返回 True，失败返回 False。
        """
        self._ensure_navigation_service()
        return self._navigation_service.navigate_to_pose(pose)

    @property
    def node_name(self):
        """获取当前ROS节点名称。

        Returns:
            str: 当前ROS节点的名称。
        """
        return rospy.get_name()


# 使用示例
if __name__ == '__main__':
    try:
        # 创建服务管理器实例，只初始化导航服务，设置5秒超时
        manager = WaypointServiceManager(
            node_name='waypoint_test',
            services=['navigation'],
            timeout=5.0
        )
        rospy.loginfo("Using node: %s" % manager.node_name)

        # 设置导航结果回调
        def on_navi_result(result):
            rospy.loginfo("Navigation result: %s" % result)
        manager.set_navi_result_callback(on_navi_result)

        # 稍后使用信息服务（将自动初始化）
        wp_count = manager.get_waypoint_count()
        rospy.loginfo("Waypoint count: %s" % wp_count)

        if wp_count and wp_count > 0:
            # 获取第一个航点
            wp = manager.get_waypoint_by_index(0)
            if wp:
                name, pose = wp
                rospy.loginfo("First waypoint: %s" % name)

                # 导航到该航点
                manager.navigate_to_waypoint(name)

                # 使用保存服务（将自动初始化）
                manager.save_waypoints("/tmp/waypoints.xml")

        # 验证单例模式
        same_manager = WaypointServiceManager()
        rospy.loginfo("Singleton verified: %s" % (manager is same_manager))

        rospy.spin()

    except Exception as e:
        rospy.logerr("Test failed: %s" % str(e))
