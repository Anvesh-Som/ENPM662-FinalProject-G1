from setuptools import find_packages, setup

package_name = 'medibot'
data_files=[]
data_files.append(('share/ament_index/resource_index/packages',
            ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/bringup_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/rsp_only_launch.py']))
data_files.append(('share/' + package_name + '/urdf', ['urdf/medibot.urdf']))
data_files.append(('share/' + package_name + '/urdf', ['urdf/medibot.csv']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/hospital.sdf']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/base_link.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/forearm_link.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/left_finger_link.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/left_wheel_link.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/right_finger_link.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/right_wheel_link.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/shoulder_link.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/upper_arm_link.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/wrist_link.STL']))
data_files.append(('share/' + package_name + '/config', ['config/medibot_items.yaml']))
data_files.append(('share/' + package_name + '/config', ['config/medibot_controllers.yaml']))
# data_files.append(('share/' + package_name + '/config', ['config/joint_names_medibot.yaml']))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anvesh',
    maintainer_email='anvesh.s.som@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'medibot_arm_controller = medibot.arm_controller:main',
            'medibot_base_controller = medibot.base_controller:main',
            'medibot_base_odometry = medibot.base_odometry:main',
            'medibot_forward_kinematics = medibot.forward_kinematics:main',
            'medibot_item_dispatcher = medibot.item_dispatcher:main',
            "medibot_joint_state_retimer = medibot.joint_state_retimer:main",
            "medibot_arm_goal_ik = medibot.arm_goal_ik_node:main",
        ],
    },
)
