'''
Created on March, 2015

'''

from pathlib import Path
import shutil, errno
import sys
import argparse

from transformer.astutils.names import NonExistingIdent, ExistingIdent, RootNamespace
from transformer.radlm import infos, language, gather, strip, transformer, generator
from transformer.radlm.parser import Semantics
from transformer.radlm.utils import pretty_print, write_file

class Exit(Exception):
    def __init__(self, error_code, msg):
        self.error_code = error_code
        self.msg = msg

def prepare_workspace(ws_dir):
    ws_dir = Path(ws_dir)
    if not ws_dir.is_dir():
        raise Exit(-2, "The workspace directory {} doesn't exist.\n"
                "You can change it using the --ws_dir option."
                "".format(ws_dir))
    infos.ws_dir = ws_dir

def prepare_source(filename, suffix):
    source = Path(filename)
    if source.suffix != suffix:
        raise Exit(-3, "RADLM file need to have {} suffix, {} given."
                       "".format(suffix, str(filename)))
    if not source.is_file():
        raise Exit(-3, str(source) + "isn't a valid file.")
    try: #We ensure the file is readable.
        source.open('r').close()
    except:
        raise Exit(-3, "The source file isn't readable.")
    infos.source_file = source

def transform_radl(ws_dir=None, radl_files=None, radlm_file=None, **_):

    prepare_workspace(ws_dir)

    ########
    # Bootstrap the semantics from the language definition.
    ########
    infos.semantics = Semantics(language)

    #processing the radlm file first
    prepare_source(radlm_file, '.radlm')
    ##############
    # Parse RADLM
    ##############
    
    qname = infos.root_namespace.qualify(infos.source_file.stem)

    with infos.source_file.open() as f:
        infos.radlm_ast = infos.semantics(f.read(), qname, infos.root_namespace)
        
    gather.do_pass(infos.radlm_ast)
    strip.do_pass(infos.radlm_ast)
    content = pretty_print(infos.radlm_ast)
    radl_name = infos.source_file.stem + ".radl"
    write_file(infos.ws_dir / radl_name, content)

    #processing each radl file
    for rf in radl_files:
        prepare_source(rf, '.radl')
        infos.root_namespace = RootNamespace()
        qname = infos.root_namespace.qualify(infos.source_file.stem)
        with infos.source_file.open() as f:
            infos.radl_ast = infos.semantics(f.read(), qname, infos.root_namespace)
        transformer.do_pass(infos.radl_ast)
        content = pretty_print(infos.radl_ast)
        write_file(infos.ws_dir / infos.source_file.name, content)

def generate_files(ws_dir=None, spec_file=None, **_):

    prepare_workspace(ws_dir)

    prepare_source(spec_file, '.spec')

    with infos.source_file.open() as f:
        generator.process(f.read())


if __name__ == "__main__":

    infos.calling_dir = Path.cwd()

    ########
    # Parse arguments
    ########
    p = argparse.ArgumentParser(prog='radlm')
    p.add_argument('--version_lang', action='version', version='RADLM language ' + language.version)
    p.add_argument('--ws_dir', default='./radlm/', metavar='DIR', help='generate files in the DIR')

    subs_p = p.add_subparsers(dest='cmd', title='subcommands')
    
    #transformation option
    transformp = subs_p.add_parser('trans', help='transform radl files')
    transformp.set_defaults(func=transform_radl)

    transformp.add_argument('radl_files', nargs='+', help='the RADL source files to be transformed')
    transformp.add_argument('-M','--radlm_file', help='the RADLM file used to transform the RADL source file')
 
    #generation option
    genp = subs_p.add_parser('gen', help='generate files from specificaiton')
    genp.set_defaults(func=generate_files)

    genp.add_argument('spec_file', help='the specification from which files are generated')

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
