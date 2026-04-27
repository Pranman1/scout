import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'scout_system'


def walk(subdir):
    """Return (install_path, [files]) tuples for every file under `subdir/` recursively.

    Keeps the directory structure intact inside share/<package>/<subdir>/..., which
    matters for Gazebo model folders (each model lives in its own subdir with
    model.config + model.sdf)."""
    entries = []
    for root, _dirs, files in os.walk(subdir):
        if not files:
            continue
        rel = os.path.relpath(root, subdir)
        install_dir = os.path.join('share', package_name, subdir)
        if rel != '.':
            install_dir = os.path.join(install_dir, rel)
        entries.append((install_dir, [os.path.join(root, f) for f in files]))
    return entries


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    (os.path.join('share', package_name, 'maps'), glob('maps/*')),
]
data_files += walk('models')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranav',
    maintainer_email='pranav@todo.todo',
    description='Scout hazard-detection TurtleBot3 stack (EECS106A).',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'manual_mapper   = scout_system.manual_mapper:main',
            'scout_teleop    = scout_system.scout_teleop:main',
            'auto_mapper     = scout_system.auto_mapper:main',
            'scan_resampler  = scout_system.scan_resampler:main',
            'mission_manager = scout_system.mission_manager:main',
            'hazard_detector = scout_system.hazard_detector:main',
            'hazard_tracker  = scout_system.hazard_tracker:main',
            'ur7_stub        = scout_system.ur7_client_stub:main',
        ],
    },
)
