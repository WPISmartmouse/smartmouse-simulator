import argparse
import colorama
import json
import os
import subprocess
import sys

ALL = 'all'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("build_conf", nargs='?', default=ALL, help="A name of one of the build confs in the .build folder")
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


def common(args):
    if args.root:
        root = args.root
    else:
        root = os.path.join(os.getcwd(), '.build')

    confs = os.listdir(root)

    if not os.path.isdir(root):
        print(colorama.Fore.RED, end='')
        print("Root {} is not a directory".format(root))
        print(colorama.Fore.RESET, end='')

    # yea it's annoying, but precheck all the confs
    if args.build_conf != ALL:
        confs = [args.build_conf]

    for conf in confs:
        conf_dir = os.path.join(root, conf)
        if not os.path.isdir(conf_dir):
            print(colorama.Fore.RED, end='')
            print("Conf {} is not a directory".format(conf_dir))
            print(colorama.Fore.RESET, end='')
            sys.exit(1)

    return root, confs


def build(args):
    root, confs = common(args)
    for conf in confs:
        success = build_conf(root, conf)
        if not success and not args.continue_on_failure:
            break


def build_conf(root, conf, targets=[]):
    cwd = os.path.join(root, conf)
    cmd = ["make"] + targets
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode:
        print(colorama.Fore.RED, end='')
        print("Command failed: {} in directory {}".format(cmd, cwd))
        print(colorama.Fore.RESET, end='')
        return False

    return True


def test(args):
    root, confs = common(args)
    for conf in confs:
        success = test_conf(root, conf)
        if not success and not args.continue_on_failure:
            break


def test_conf(root, conf):
    cwd = os.path.join(root, conf)
    print(cwd)

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
        build_conf(root, conf, targets=tests)

    cmd = ["ctest", "--output-on-failure"]
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode:
        print(colorama.Fore.RED, end='')
        print("Command failed: {} in directory {}".format(cmd, cwd))
        print(colorama.Fore.RESET, end='')
        return False

    return True

