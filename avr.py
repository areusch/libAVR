#!/usr/bin/python
#
# libAVR project management script.

from optparse import OptionParser
from os.path import exists
import os
import shutil
import subprocess
import sys

kConfigFileName = 'libavr-config.h'
kGitDir = '.git'
kLibAVRGitRepo = 'git://github.com/areusch/libAVR.git'
kLibAVRSrc = 'src'
kLibAVRPath = 'libavr'
kLibAVRGitPath = 'libavr-support'
kSConstructFileName = 'template-SConstruct'

class Resolution:
    def __init__(self, func, suggestion):
        self.func = func
        self.suggestion = suggestion

    def __call__(self, *args, **kwargs):
        d = self.func(*args, **kwargs)
        if d:
            return self.suggestion % d
        return None

# Error Resolutions:
kNoResolutions = Resolution(lambda: {}, 'No suggestions are available to resolve the problem.')
kResolutionNetwork = Resolution(lambda: {}, 'Check your network interface.')
kResolutionGit = Resolution(lambda: {}, 'Ensure git is installed.')

class UserResolvableError(Exception):
    def __init__(self, error, resolution=None):
        Error.__init__(self, error)
        self.error = error
        if resolution is None:
            self.resolution = kNoResolutions
        elif type(resolution) == 'list':
            self.resolution = resolution
        else:
            self.resolution = [resolution]

    def build_resolution_string(self):
        res = []
        for x in self.resolution:
            r = x()
            if r:
                res.append(' - %s' % r)
        if len(res) == 0:
            res.append(' - %s' % kNoResolutions())

    def __str__(self):
        return '%s\nTry one of the following:\n%s' % (self.error,
                                                      self.build_resolution_string())

class SubprocessError(UserResolvableError):
    def __init__(self, action, resolution, cmdline, exit_code, log=None):
        UserResolvableError.__init__(self, action, resolution)
        self.action = action
        self.resolution = resolution
        self.cmdline = cmdline
        self.exit_code = exit_code
        self.execution_log = log

    def __str__(self):
        logstr = 'A log of stdout/stderr is as follows:\n%s\n\n### --- END OF LOG --- ###\n' % \
                 self.execution_log if self.execution_log else ''
        return 'While %s with command %s: %s exited with code %d.\n%sTry one of the following:\n%s' % \
               (self.action,
                self.cmdline,
                self.cmdline[0],
                self.exit_code,
                logstr,
                self.build_resolution_string())

def try_run(action, resolution, *args, **kwargs):
    subprocess_kwargs = kwargs['subprocess'] if 'subprocess' in kwargs else {}
    subprocess_kwargs['stdout'] = subprocess.PIPE
    subprocess_kwargs['stderr'] = subprocess.STDOUT
    print action
    process = subprocess.Popen(args, **subprocess_kwargs)
    stdout, stderr = process.communicate()
    if process.returncode != 0:
        raise SubprocessError(action, resolution, args, process.exit_code, log=stdout)

def do_init_opts(p):
    pass

def do_init_project(opts, args):
    if exists(kGitDir) or exists(kConfigFileName) or exists(kLibAVRPath) or exists(kLibAVRGitPath):
        raise Exception('Project already exists!')

    # 1) git init
    try_run('initializing the git repo',
            [kResolutionGit],
            'git', 'init')

    # 2) git submodule init
    try_run('initializing submodules',
            [kResolutionGit],
            'git', 'submodule', 'init')

    # 3) git submodule add ...
    try_run('adding the libAVR git repository as a remote',
            [kResolutionNetwork, kResolutionGit],
            'git', 'submodule', 'add', kLibAVRGitRepo, kLibAVRGitPath)

    # 4) git submodule update
    try_run('pulling from the libAVR git repository',
            [kResolutionNetwork, kResolutionGit],
            'git', 'submodule', 'update')

    # 5) Update submodule
    os.symlink(kLibAVRGitPath + os.path.sep + kLibAVRSrc, kLibAVRPath)

    # 6) Touch the config file
    open(kConfigFileName, 'w').close()

    # 7) Copy SConstruct
    shutil.copy(kLibAVRGitPath + os.path.sep + kSConstructFileName,
                kSConstructFileName)

    # 8) Git add/commit
    try_run('staging files',
            [kResolutionGit],
            'git', 'add', 'libavr', 'SConstruct', 'libavr-config.h')
    try_run('git commit',
            [kResolutionGit],
            'git', 'commit', '-m', 'Add libAVR to project.')


def do_configure_opts(p):
    pass

def do_configure(opts, args):
    pass

def do_pull_opts(p):
    pass

def do_pull_git(opts, args):
    try_run('pulling from the libAVR git repository',
            [kResolutionNetwork, kResolutionGit],
            'git', 'submodule', 'foreach', 'git', 'pull')

def do_build_opts(p):
    pass

def do_build(opts, args):
    try_run('',
            None,
            'scons')

def do_burn_opts(p):
    pass

def do_burn(opts, args):
    try_run('building and burning the AVR project',
            None,
            'scons', '--burn')

command_map = {'init': {'func': do_init_project,
                        'opt': do_init_opts,
                        'doc': 'Initialize a libAVR-backed project in the current directory.'},
               'configure': {'func': do_configure,
                             'opt': do_configure_opts,
                             'doc': 'Configure the compile-time options for a libAVR project.'},
               'pull': {'func': do_pull_git,
                        'opt': do_pull_opts,
                        'doc': 'Update libAVR to the latest version by pulling.'},
               'build': {'func': do_build,
                         'opt': do_build_opts,
                         'doc': 'Build the libAVR project in the current directory.'},
               'burn': {'func': do_burn,
                        'opt': do_burn_opts,
                        'doc': 'Build and burn the libAVR project in the current directory.'}}

def main(argv):
    if (len(argv) < 2):
        print 'Usage: %s <command>' % argv[0]
        print
        print 'Specify --help after <command> for command-specific options.'
        print 'Available commands are:'
        for cmd in command_map.keys():
            print '  %-10s %s' % (cmd, command_map[cmd]['doc'])
        sys.exit(1)
    elif not argv[1] in command_map:
        print 'Invalid command: %s' % argv[1]
        sys.exit(1)

    cmd = argv[1]
    del argv[1]
    p = OptionParser('%%s %s [options]' % cmd)
    command_map[cmd]['opt'](p)

    opts, args = p.parse_args()
    command_map[cmd]['func'](opts, args)


if __name__ == '__main__':
    main(sys.argv)
