from setuptools import setup

package_name = 'kinemic_ros_bridge'

setup(
 name=package_name,
 version='0.1.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools', 'kserviceconnect'],
 zip_safe=True,
 tests_require=[],
 entry_points={
     'console_scripts': [
             'kinemic_ros_node = kinemic_ros_bridge.kinemic_ros_node:main'
     ],
   },
 dependency_links=['https://repo.kinemic.com/repository/pypi/simple/kserviceconnect/',
                   'https://repo.kinemic.com/repository/pypi/simple/sensorproto/']
)
