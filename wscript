#!/usr/bin/env python
# encoding: utf-8
import sys
import os
import fnmatch
import glob
sys.path.insert(0, sys.path[0]+'/waf_tools')

VERSION = '0.0.1'
APPNAME = 'cartesian_franka'

srcdir = '.'
blddir = 'build'

from waflib.Build import BuildContext
from waflib import Logs
from waflib.Tools import waf_unit_test

import eigen
import generic

def options(opt):
    opt.load('compiler_cxx')
    opt.load('compiler_c')
    opt.load('eigen')
    opt.load('pybind')

    opt.add_option('--python', action='store_true', help='compile python bindings', dest='pybind')
    generic.add_options(opt, 'libfranka')


def configure(conf):
    conf.get_env()['BUILD_GRAPHIC'] = False

    conf.load('compiler_cxx')
    conf.load('compiler_c')
    conf.load('waf_unit_test')
    conf.load('eigen')
    conf.load('generic')

    if conf.options.pybind:
        conf.load('python')
        conf.load('pybind')
    
    conf.env['py_flags'] = ''
    conf.env['BUILD_PYTHON'] = False
    if conf.options.pybind:
        conf.check_python_version((2, 7))
        conf.check_python_headers(features='pyext')
        conf.check_python_module('numpy')
        conf.check_pybind11(required=True)
        conf.env['BUILD_PYTHON'] = True
        if conf.env.CXX_NAME in ["gcc", "g++"]:
            conf.env['py_flags'] = ' -fPIC' # we need -fPIC in some Linux/gcc combinations

    # libfranka
    conf.check_generic(lib='libfranka', include_check='franka/robot.h', lib_check=['franka'], required=True)

    # eigen 3.x
    conf.check_eigen(required=True, min_version=(3,0,0))

    native = '' # future AVX if needed

    if conf.env.CXX_NAME in ["clang"]:
        common_flags = "-Wall -std=c++11"
        # no-stack-check required for Catalina
        opt_flags = " -O3 -g -faligned-new  -fno-stack-check" + native
    else:
        gcc_version = int(conf.env['CC_VERSION'][0]+conf.env['CC_VERSION'][1])
        if gcc_version < 47:
            common_flags = "-Wall -std=c++0x"
        else:
            common_flags = "-Wall -std=c++11"
        opt_flags = " -O3 -g" + native
        if gcc_version >= 71:
            opt_flags = opt_flags + " -faligned-new"

    all_flags = common_flags + conf.env['py_flags'] + opt_flags

    conf.env['CXXFLAGS'] = conf.env['CXXFLAGS'] + all_flags.split(' ')

    print(conf.env['CXXFLAGS'])

def build(bld):
    
    libs = 'LIBFRANKA EIGEN'
    defines = ['VERSION=\"0.1\"']
    
    bld.program(features = 'cxx cxxstlib',#static lib
                source = "src/cartesian_franka/joint_motion_generator.cpp \
                         src/cartesian_franka/cartesian_motion_generator.cpp \
                         src/cartesian_franka/robot.cpp",
                includes = './src',
                uselib = libs,
                defines = defines,
                target = 'cartesian_franka')
    

    bld.program(features = 'cxx',
                source = "src/example.cpp",
                includes = './src',
                uselib =  libs,
                use='cartesian_franka',
                defines = defines,
                target = 'example')

    bld.program(features = 'c cshlib pyext',
                source = './src/python.cpp',
                includes = './src',
                uselib  = 'PYBIND11 ' + libs,
                use = 'cartesian_franka',
                defines = defines,
                target = 'pycartesian_franka')

