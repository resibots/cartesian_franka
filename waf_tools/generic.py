#! /usr/bin/env python
# encoding: utf-8
# JB Mouret - 2020
"""
Generic library configuration for waf
"""

import os, glob, types
from waflib.Configure import conf

def add_options(opt, lib):
    opt.add_option('--' + lib, type='string', help='path to ' + lib, dest=lib)
    
@conf
def check_generic(conf, *k, **kw):
    def get_directory(filename, dirs):
        res = conf.find_file(filename, dirs)
        return res[:-len(filename)-1]
    include_dirs = ['/usr/include/', '/usr/local/include/', os.environ['HOME'] + '/include']
    lib_dirs = ['/usr/lib/', '/usr/local/lib/', os.environ['HOME'] + '/lib', '/usr/lib/x86_64-linux-gnu/']

    # keywords
    lib = kw.get('lib', '')
    include_check = kw.get('include_check', '')
    lib_check = kw.get('lib_check', [])
    required = kw.get('required', False)
  

    ####### includes
    if include_check != '':
        conf.start_msg('Checking includes for ' + lib + ' [generic]')
        if getattr(conf.options, lib):
            inc = getattr(conf.options, lib)
            include_dirs = [inc, inc + '/include']
        try:
            incl = get_directory(include_check, include_dirs)
            conf.env['INCLUDES_' + lib.upper()] = [incl]
            conf.end_msg(incl)

        except:
            if required:
                conf.fatal(include_check + ' not found in %s' % str(include_dirs))
            conf.end_msg(include_check + ' not found in %s' % str(include_dirs), 'RED')
            return 1

    ######## lib (optional since there are header-only libs)
    # OSX/Mac uses .dylib and GNU/Linux .so
    lib_suffix = 'dylib' if conf.env['DEST_OS'] == 'darwin' else 'so'
    if lib_check != []:
        conf.start_msg('Checking libraries for ' + lib + ' [generic]')
        if getattr(conf.options, lib):
            libpath = getattr(conf.options, lib)
            lib_dirs = [libpath, libpath + '/lib', libpath + '/build']
        #conf.env.LIBPATH_DART = dart_lib

        conf.env['LIB_' + lib.upper()] = []
        found = False
        lib_dir = ''
        for i in lib_check:
            found_a = True
            found_so = True
            try: #check for .a
                lib_dir = get_directory('lib' + i + '.' + 'a', lib_dirs)
            except:
                found = False
            try: #check for .suffix
                lib_dir = get_directory('lib' + i + '.' + lib_suffix, lib_dirs)
            except:
                found_so = False
            if (found_a) or (found_so):
                found = True
                conf.env['LIB_' + lib.upper()] += [i]
                conf.env['LIBPATH_' + lib.upper()] += [lib_dir]
                conf.end_msg(lib_dir)

        if  not found:
            if required:# will not work if many libraries
                conf.fatal(str(lib_check) + ' not found in %s' % str(lib_dirs))
            conf.end_msg(str(lib_check) + ' not found in %s' % str(lib_dirs), 'RED')

    return 1
