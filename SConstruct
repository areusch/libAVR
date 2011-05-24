import os
import pickle
import sys

configuration_file_name = '.configure'

def is_valid_avr_libc_base(x):
    return os.path.exists(x + os.path.sep + 'avr' + os.path.sep + 'io.h')

def validate_config(c):
    for x in c['includes']:
        if is_valid_avr_libc_base(x):
            break
    else:
        return False

    return True

config = None
if os.path.exists(configuration_file_name):
    try:
        config = pickle.load(open(configuration_file_name))
        if not validate_config(config):
            os.unlink(configuration_file_name)
    except:
        os.unlink(configuration_file_name)

platform_configs = {'Darwin': {'include_search_path': ['/usr/local/CrossPack-AVR/avr/include']}}

if not config:
    platform_config = platform_configs[os.uname()[0]]

    config = {'includes': []}
    if 'includes' in platform_config:
        config['includes'] += platform_config['includes']

    for x in platform_config['include_search_path']:
        if is_valid_avr_libc_base(x):
            config['includes'].append(x)
            break
    else:
        print "Configure error: Cannot find avr-libc!"
        sys.exit(1)

    pickle.dump(config, open(configuration_file_name, "w"))

AddOption('--chip',
          dest='chip',
          type='string',
          nargs=1,
          action='store',
          help='part # of the chip to compile to')

AddOption('--frequency',
          dest='frequency',
          type='int',
          nargs=1,
          action='store',
          help='operating frequency of the mega, in mhz')

env = Environment(CC = 'avr-gcc',
                  CFLAGS = ['-I%s' % x for x in config['includes'] + ['src']] + ['-mmcu=%s' % GetOption('chip'), '-std=gnu99', '-DF_CPU=%d000000L' % GetOption('frequency'), '-O3'],
                  ENV = {'PATH': os.environ['PATH']},
                  LDFLAGS = ['-mmcu=%s' % GetOption('chip')])

libavr = env.Library('src/libavr', Glob('src/*.c'))
