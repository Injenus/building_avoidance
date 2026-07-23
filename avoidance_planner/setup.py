from setuptools import setup
pkg = 'avoidance_planner'
setup(
    name=pkg, version='0.1.0', packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        ('share/' + pkg + '/config', ['config/mission.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='dev', maintainer_email='dev@example.com',
    description='Bug2-like obstacle avoidance planner', license='MIT',
    entry_points={'console_scripts': ['planner = avoidance_planner.planner_node:main']},
)
