'''
Created on March, 2015

'''

from pathlib import Path
from time import sleep
import sys
import argparse

from transformer.radlm import infos, language
from transformer.radlm.parser import Semantics

class Exit(Exception):
    def __init__(self, error_code, msg):
        self.error_code = error_code
        self.msg = msg

def prepare_radlm_source(filename):
    source = Path(filename)
    if source.suffix != '.radlm':
        raise Exit(-3, "RADLM file need to have .radlm suffix, {} given."
                       "".format(str(filename)))
    if not source.is_file():
        raise Exit(-3, str(source) + "isn't a valid file.")
    try: #We ensure the file is readable.
        source.open('r').close()
    except:
        raise Exit(-3, "The source file isn't readable.")
    infos.source_file = source

def transform_spec(radl_files=None, radlm_file=None, **_):
    ########
    # Bootstrap the semantics from the language definition.
    ########
    infos.semantics = Semantics(language)

    #processing the radlm_file first
    prepare_radlm_source(radlm_file)
     

if __name__ == "__main__":

    infos.calling_dir = Path.cwd()

    ########
    # Parse arguments
    ########
    p = argparse.ArgumentParser(prog='radlm')
    p.add_argument('--version_lang', action='version', version='RADLM language ' + language.version)

    subs_p = p.add_subparsers(dest='cmd', title='subcommands')
    
    transformp = subs_p.add_parser('transform', help='transform radl files')
    transformp.set_defaults(func=transform_spec)

    transformp.add_argument('radl_files', nargs='+', help='the RADL source files to be transformed')
    transformp.add_argument('-M','--radlm_file', help='the RADLM file used to transform the RADL source file')
    args = p.parse_args()

    try:
        cmd = args.func
    except AttributeError:
        print("You need to specify a subcommand.\n")
        p.print_help()
        exit(1)
    try:
        exit(cmd(**args.__dict__))
    except Exit as e:
        print(e.msg)
        exit(e.error_code)
