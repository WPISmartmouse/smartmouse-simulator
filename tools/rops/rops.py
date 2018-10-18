import argparse
import json
import os

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
    parser.add_argument("ms", help="input *.ms file describing the robot", type=argparse.FileType('r'))
    parser.add_argument("--cpp-output-dir", type=argparse.FileType)
    parser.add_argument("--h-output-dir", type=argparse.FileType)

    args = parser.parse_args()

    mouse = json.load(args.ms)


