#!/usr/bin/env python3

import os
import subprocess
import configparser
from ament_index_python.packages import get_package_share_directory
import configparser


def get_base_path():
    """根据是否具有root权限来确定日志路径"""
    response = subprocess.run(
        "touch" + " /root/RCC_WS/privilege_test", stdout=subprocess.PIPE, shell=True)
    """ 没有root权限 """
    if (response.returncode != 0):
        print("no root privilege, use install directory")
        base_path = os.path.join(
            get_package_share_directory('jaka_driver'),
            'soft_inserter_interface'
        )
    else:
        subprocess.run("rm" + " /root/RCC_WS/privilege_test",
                       stdout=subprocess.PIPE, shell=True)
        base_path = os.path.join(
            '/root/RCC_WS',
            'log',
            'ROBOT_ARM'
        )

    if not os.path.exists(base_path):
        os.makedirs(base_path)
    return base_path
