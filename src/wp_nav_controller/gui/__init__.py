# -*- coding: utf-8 -*-

"""
Navigation Controller GUI package.

This package provides a PySide6-based graphical user interface
for the waypoint navigation controller.
"""

from .navigation_ui import NavigationControllerUI

# 导出主要的类和函数，这样可以直接从gui包导入
__all__ = ['NavigationControllerUI']
