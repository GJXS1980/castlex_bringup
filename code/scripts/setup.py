#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from distutils.core import setup
from Cython.Build import cythonize
import os

'''
TODO:
    使用Cpython 编译python文件，将.py文件编译成.c文件和.so文件

USAGE:
    python3 setup.py build_ext --inplace 

'''
# 在列表中输入需要加密的py文件
key_funs = ['castlex_stm32_bringup_python3.py']

setup(
    name="castlex_stm32_bringup_python3 app", 
    ext_modules = cythonize(key_funs),
)

print('Done!')
