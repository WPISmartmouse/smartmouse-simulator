import argparse
import configparser
import json
import os
import subprocess

try:
    import colorama
except ModuleNotFoundError as e:
    print(e)
    print("Did you forget to source your virtualenv?")



ALL = 'all'
ERROR = colorama.Fore.RED + "ERROR: " + colorama.Fore.RESET


def main():
    if not os.environ["SSIM"]:
        print(colorama.Fore.RED, end='')
        print("$SSIM environment variable was not set")
        print(colorama.Fore.RESET, end='')

    parser = argparse.ArgumentParser()
    parser.add_argument("--verbose", '-v', help="print more stuff")
    parser.add_argument("--root", '-r', help="specify a custom root build directory")
    parser.add_argument("--continue-on-failure", '-c', action="store_true",
                        help="Keep going if one build conf has an error")

    subparsers = parser.add_subparsers(dest='cmd')
    subparsers.required = True

    clean_parser = subparsers.add_parser('clean')
    clean_parser.set_defaults(func=clean)
    clean_parser.add_argument("conf_names", nargs='*', default=ALL, help="name(s) of build conf(s)")

    cmake_parser = subparsers.add_parser('cmake')
    cmake_parser.set_defaults(func=cmake)
    cmake_parser.add_argument("conf_names", nargs='*', default=ALL, help="name(s) of cmake conf(s)")

    build_parser = subparsers.add_parser('build')
    build_parser.set_defaults(func=build)
    build_parser.add_argument("conf_names", nargs='*', default=ALL, help="name(s) of build conf(s)")
    build_parser.add_argument("--targets", nargs='*', default=[ALL], help="name(s) of target(s)")

    test_parser = subparsers.add_parser('test')
    test_parser.set_defaults(func=test)
    test_parser.add_argument("conf_names", nargs='*', default=ALL, help="name(s) of build conf(s)")

    args = parser.parse_args()

    args.func(args)


class CompleteConf:

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
        root = os.path.join(os.environ["SSIM"], '.build')

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

    cmake_config = configparser.ConfigParser()
    config_path = os.path.expanduser("~/.config/smartmouse-simulator.ini")
    if not os.path.exists(config_path):
        print(colorama.Fore.YELLOW, end='')
        print("Configuration file {} does not exist. Generating default...".format(config_path))
        print(colorama.Fore.RESET, end='\n')

        cmake_config['cmake_args'] = {
            'arm': '-DREAL=ON',
            'debug': '-DCMAKE_BUILD_TYPE=DEBUG',
            'asan': '-DADDRESS_SANITIZER=ON',
            'ubsan': '-DUNDEFINED_SANITIZER=ON',
            'tsan': '-DTHREAD_SANITIZER=ON',
            'lsan': '-DLEAK_SANITIZER=ON',
            'release': '-DCMAKE_BUILD_TYPE=RELEASE',
        }
        cmake_config.write(open(config_path, 'w'))
    else:
        cmake_config.read(config_path)

    cmake_confs = cmake_config['cmake_args']

    complete_confs = []
    if args.conf_names == ALL:
        print(colorama.Fore.GREEN, end='')
        print("Working on ALL confs")
        print(colorama.Fore.RESET, end='\n')
        for conf_name, cmake_flags in cmake_confs.items():
            complete_confs.append(CompleteConf(root, conf_name, cmake_flags))
    else:
        for conf_name in args.conf_names:
            if conf_name not in cmake_confs:
                print(ERROR + "No configuration {}. Valid options are: {}".format(conf_name, list(cmake_confs.keys())))
                return root, None, True
            else:
                cmake_flags = cmake_confs[conf_name]
                complete_confs.append(CompleteConf(root, conf_name, cmake_flags))

    for conf in complete_confs:
        if not os.path.isdir(conf.dir):
            print(colorama.Fore.YELLOW, end='')
            print("Making new build conf {}".format(conf.dir))
            print(colorama.Fore.RESET, end='')
            os.mkdir(conf.dir)

    return root, complete_confs, False


def cmake(args):
    root, confs, error = common(args)
    if error:
        return
    for conf in confs:
        success = cmake_conf(args, root, conf)
        if not success and not args.continue_on_failure:
            break


def cmake_conf(args, root, conf):
    cwd = os.path.join(root, conf.name)
    cmd = ['cmake', '../..'] + conf.cmake_flags
    if args.verbose:
        print(cmd)
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode:
        print(colorama.Fore.RED, end='')
        print("CMake failed {} in directory {}".format(cmd, cwd))
        print(colorama.Fore.RESET, end='')
        return False


def clean(args):
    root, confs, error = common(args)
    if error:
        return
    for conf in confs:
        success = clean_conf(args, root, conf)
        if not success and not args.continue_on_failure:
            break


def clean_conf(args, root, conf, targets=[]):
    full_conf_path = os.path.join(root, conf.name, "*")
    if full_conf_path[:5] != '/home':
        print(colorama.Fore.RED, end='')
        print("you probably don't want to delete {}".format(full_conf_path))
        print(colorama.Fore.RESET, end='')
        return False
    cmd = ['rm', '-rf', full_conf_path]
    result = subprocess.run(cmd)
    if result.returncode:
        print(colorama.Fore.RED, end='')
        print("rm command {} failed".format(full_conf_path))
        print(colorama.Fore.RESET, end='')
        return False


def build(args):
    root, confs, error = common(args)
    if error:
        return
    for conf in confs:
        success = build_conf(args, root, conf, args.targets)
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
        success = test_conf(args, root, conf)
        if not success and not args.continue_on_failure:
            break


def test_conf(args, root, conf):
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
