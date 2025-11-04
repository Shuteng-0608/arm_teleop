# from distutils.core import setup
# from catkin_pkg.python_setup import generate_distutils_setup

# # fetch values from package.xml
# setup_args = generate_distutils_setup(
#     packages=['vptele',
#               'vptele.arm_control', 'vptele.arm_control.Robotic_Arm', 
#               'vptele.core', 
#               'vptele.end_effectors', 'vptele.end_effectors.gripper', 'vptele.end_effectors.hand',
#               'vptele.teleop.image_server', 'vptele.teleop.open_television', 'vptele.teleop.robot_control', 'vptele.teleop.robot_control.dex_retargeting', 'vptele.teleop.utils',
#               'vptele.trajectory',
#               'vptele.utils'], # 你要安装的Python包名，必须和目录名一致
#     package_dir={'': 'src'}, # 告诉setup.py你的Python包在`src`目录下
#     # 如果你有可执行脚本，也要在这里声明，这样catkin会为它们生成入口点，方便rosrun
#     # scripts=['scripts/my_script'], # 如果脚本在scripts目录下
# )

# setup(**setup_args)

# ...existing code...
from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

# 自动发现 src 下的包，避免手动列出
packages = find_packages('vptele')

# 从 package.xml 获取默认的安装参数
setup_args = generate_distutils_setup(
    packages=packages,
    package_dir={'': 'vptele'},
)

# 补充/覆盖项
setup_args.setdefault('include_package_data', True)
# setup_args.setdefault('python_requires', '>=3.6')
# 如果想让 rosrun 可直接调用脚本，保留 scripts 列表（可按需修改）
# setup_args.setdefault('scripts', ['scripts/main_ros.py', 'scripts/test_ik.py'])

setup(**setup_args)
# ...existing code...