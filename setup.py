from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    # packages=['processStrategy', 'sentenceBuffer'],  # 所有包名放在一个列表中
    # package_dir={'': 'lib'},  # 指定包所在的目录
)
# 'sr_modbus_model', 'sr_modbus_sdk'
setup(**setup_args)