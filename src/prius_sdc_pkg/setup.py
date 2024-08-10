from setuptools import setup
import os
from glob import glob

package_name = 'prius_sdc_pkg'
config_module = 'prius_sdc_pkg/config'
det_module = 'prius_sdc_pkg/Detection'
det_lane_module = 'prius_sdc_pkg/Detection/Lanes'
det_sign_module = 'prius_sdc_pkg/Detection/Signs'
det_TL_module = 'prius_sdc_pkg/Detection/TrafficLights'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,config_module,det_module,det_lane_module,det_sign_module,det_TL_module],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('lib', package_name), glob('scripts/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='louwee',
    maintainer_email='adulouis980@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder_node = prius_sdc_pkg.video_recorder:main',
            'driver_node = prius_sdc_pkg.driving_node:main',
            'spawner_node = prius_sdc_pkg.sdf_spawner:main'
        ],
    },
)
