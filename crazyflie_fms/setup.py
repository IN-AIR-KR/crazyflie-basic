import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'crazyflie_fms'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        # 빌드 단계가 없는 프런트엔드 — 정적 파일 그대로 install 된다.
        # (--symlink-install 로 빌드하면 web/ 수정이 재빌드 없이 바로 반영)
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Crazyflie Fleet Management System — 웹 UI 기반 편대 모니터링/임무 제어',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fms_node = crazyflie_fms.fms_node:main',
            'web_server = crazyflie_fms.web_server:main',
        ],
    },
)
