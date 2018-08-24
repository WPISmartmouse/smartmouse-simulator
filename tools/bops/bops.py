import configparser
import argparse
import colorama
import json
import os
import subprocess
import sys

ALL = 'all'
ERROR = colorama.Fore.RED + "ERROR: " + colorama.Fore.RESET

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("conf_name", nargs='?', default=ALL, help="A name of one of the build confs in the .build folder")
    parser.add_argument("--verbose", '-v', help="print more stuff")
    parser.add_argument("--root", '-r', help="specify a custom root build directory")
    parser.add_argument("--continue-on-failure", '-c', action="store_true", help="Keep going if one build conf has an error")

    subparsers = parser.add_subparsers(dest='cmd')
    subparsers.required = True

    build_parser = subparsers.add_parser('build')
    build_parser.set_defaults(func=build)

    test_parser = subparsers.add_parser('test')
    test_parser.set_defaults(func=test)

    args = parser.parse_args()

    args.func(args)

class BuildConf:

    def __init__(self, root, name, cmake_flags):
        self.name = name
        self.dir = os.path.join(root, name)
        self.cmake_flags = cmake_flags.split(' ')

def cmake_succeeded(cwd):
    return os.path.exists(os.path.join(cwd, 'Makefile'))

def common(args):
    if args.root:
        root = args.root
    else:
        root = os.path.join(os.getcwd(), '.build')

    if not os.path.isdir(root):
        if not os.path.exists(root):
            print(colorama.Fore.YELLOW, end='')
            print("Making diretory {}".format(root))
            print(colorama.Fore.RESET, end='')
            os.mkdir(root)
        else:
            print(colorama.Fore.RED, end='')
            print("Root {} is not a directory".format(root))
            print(colorama.Fore.RESET, end='')

    config = configparser.ConfigParser()
    config_path = os.path.expanduser("~/.config/smartmouse-simulator.ini")
    if not os.path.exists(config_path):
        print(colorama.Fore.YELLOW, end='')
        print("Configuration file {} does not exist. Generating default...".format(config_path))
        print(colorama.Fore.RESET, end='')

        config['build'] = {
                'arm': '-DREAL=ON',
                'debug': '-DCMAKE_BUILD_TYPE=DEBUG',
                'asan': '-DADDRESS_SANITIZER=ON',
                'ubsan': '-DUNDEFINED_SANITIZER=ON',
                'tsan': '-DTHREAD_SANITIZER=ON',
                'lsan': '-DLEAK_SANITIZER=ON',
                'release': '-DCMAKE_BUILD_TYPE=RELEASE',
        }
        config.write(open(config_path, 'w'))
    else:
        config.read(config_path)

    confs_dict = config['build']

    if args.conf_name != ALL:
        if args.conf_name not in confs_dict:
            print(ERROR + "No configuration {}. Valid options are: {}".format(args.conf_name, list(confs_dict.keys())))
            return root, None, True
        else:
            confs_dict = {args.conf_name: confs_dict[args.conf_name]}

    confs = []
    for conf_name, cmake_flags in confs_dict.items():
        confs.append(BuildConf(root, conf_name, cmake_flags))

    for conf in confs:
        if not os.path.isdir(conf.dir):
            print(colorama.Fore.YELLOW, end='')
            print("Making new build conf {}".format(conf.dir))
            print(colorama.Fore.RESET, end='')
            os.mkdir(conf.dir)

    return root, confs, False


def build(args):
    root, confs, error = common(args)
    if error:
        return
    for conf in confs:
        success = build_conf(args, root, conf)
        if not success and not args.continue_on_failure:
            break


def build_conf(args, root, conf, targets=[]):
    cwd = os.path.join(root, conf.name)

    if not cmake_succeeded(cwd):
        cmd = ['cmake', '../..'] + conf.cmake_flags
        if args.verbose:
            print(cmd)
        result = subprocess.run(cmd, cwd=cwd)
        if result.returncode:
            print(colorama.Fore.RED, end='')
            print("CMake failed {} in directory {}".format(cmd, cwd))
            print(colorama.Fore.RESET, end='')
            return False

    cmd = ["make"] + targets
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode:
        print(colorama.Fore.RED, end='')
        print("Make failed: {} in directory {}".format(cmd, cwd))
        print(colorama.Fore.RESET, end='')
        return False

    return True


def test(args):
    root, confs, error = common(args)
    if error:
        return
    for conf in confs:
        success = test_conf(root, conf)
        if not success and not args.continue_on_failure:
            break


def test_conf(root, conf):
    cwd = os.path.join(root, conf.name)

    # Check which targets we need to recompile
    compile_commands = json.load(open(os.path.join(cwd, 'compile_commands.json'), 'r'))
    tests = []
    for cmd in compile_commands:
        if '_TEST' in cmd['file']:
            filename = cmd['file']
            test_name = os.path.splitext(os.path.basename(filename))[0]
            tests.append(test_name)

    # Recompile if necessary
    if len(tests) >= 0:
        success = build_conf(args, root, conf, targets=tests)
        if not success:
            return False


    cmd = ["ctest", "--output-on-failure"]
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode:
        print(colorama.Fore.RED, end='')
        print("Command failed: {} in directory {}".format(cmd, cwd))
        print(colorama.Fore.RESET, end='')
        return False

    return True

