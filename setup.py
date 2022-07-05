from setuptools import setup
import sys
from glob import glob
import os

package_name = 'delivery_api_server'
py_version = ".".join(map(str, sys.version_info[:2]))
site_pkgs_path = 'lib/python' + py_version + '/site-packages/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kai',
    maintainer_email='lkw303@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delivery_api_server=delivery_api_server.delivery_api_server:main',
            'dispatch_internal_hub_collect=delivery_api_server.dispatch_internal_hub_collect:main',
            'dispatch_internal_hub_deposit=delivery_api_server.dispatch_internal_hub_deposit:main',
            'dispatch_external_hub_collect=delivery_api_server.dispatch_external_hub_collect:main',
            'dispatch_external_hub_deposit=delivery_api_server.dispatch_external_hub_deposit:main',
            'post_order_test=delivery_api_server.post_order_test:test_post',
        ],
    },
)
