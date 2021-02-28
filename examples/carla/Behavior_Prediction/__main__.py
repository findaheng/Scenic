import argparse
import subprocess

parser = argparse.ArgumentParser()
parser.addArgument('--type', '-t', type=str, default='intersection', help='Specify traffic type')
parser.addArgument('--number', '-n', type=str, default='01', help='Specify scenario number in form ##')
args = parser.parse_arg()

path = f'{args.type}/{args.type}_{args.number}.scenic'

subprocess.run(['python', '-m', 'scenic', path, '--simulate'])
