'''
Created on March, 2015

'''

from pathlib import Path
import distutils.dir_util
import sys
import argparse

from framework import infos
from framework.astutils.names import NonExistingIdent, ExistingIdent, RootNamespace
from framework.radlm import radlmLang
from framework.radlm.parser import Semantics
from framework.radlm.utils import pretty_print, write_file, ensure_dir

from framework.compiler import specLang
from framework.compiler.parser import Compiler

from framework.weaver import gather, weaver



class Exit(Exception):
    def __init__(self, error_code, msg):
        self.error_code = error_code
        self.msg = msg

def prepare_workspace(project_dir, option):
    if not project_dir.is_dir():
        raise Exit(-2, "The project directory {} doesn't exist.\n"
                       .format(project_dir))
    ws_name = project_dir.name + '_' + option + '/'
    parent = project_dir.parent
    ws_dir = parent / ws_name
    ensure_dir(ws_dir)
    infos.ws_dir = ws_dir

def check_source(source):
    if not source.is_file():
        raise Exit(-3, str(source) + " isn't a valid file.")
    try: #We ensure the file is readable.
        source.open('r').close()
    except:
        raise Exit(-3, "The source file isn't readable.")
    infos.source_file = source

def weave_radl(project_dir=None, radlm_file=None, **_):
    
    project_dir = Path(project_dir) 
    prepare_workspace(project_dir, 'trans')
    distutils.dir_util.copy_tree(str(project_dir.resolve()), str(infos.ws_dir.resolve()))

    ########
    # Bootstrap the semantics from the language definition.
    ########
    infos.semantics = Semantics(radlmLang)

    #processing the radlm file first
    source = Path(radlm_file)
    if source.suffix != '.radlm':
        raise Exit(-3, "RADLM file need to have {} suffix, {} given."
                       "".format('.radlm', str(radlm_file)))
    check_source(source)

    ##############
    # Parse RADLM
    ##############
    
    qname = infos.root_namespace.qualify(infos.source_file.stem)

    with infos.source_file.open() as f:
        infos.radlm_ast = infos.semantics(f.read(), qname, infos.root_namespace)
        
    gather.collect_interceptors(infos.radlm_ast)
    gather.collect_implants(infos.radlm_ast)

    for child in infos.ws_dir.iterdir():
        if child.suffix == '.radl':
            check_source(child)
            infos.root_namespace = RootNamespace()
            qname = infos.root_namespace.qualify(infos.source_file.stem)
            with infos.source_file.open() as f:
                infos.radl_ast = infos.semantics(f.read(), qname, infos.root_namespace)
            weaver.do_pass(infos.radl_ast)
            content = pretty_print(infos.radl_ast)
            write_file(infos.ws_dir / infos.source_file.name, content)
    
    for key in infos.weaved:
        if infos.weaved[key] is False:
            print("{} is not weaved".format(key))


def compile_spec(project_dir=None, spec_file=None, **_):
    compiler = Compiler(specLang)
    source = Path(spec_file)
    with source.open() as f:
        compiler.parse(f.read())



if __name__ == "__main__":

    infos.calling_dir = Path.cwd()

    ########
    # Parse arguments
    ########
    p = argparse.ArgumentParser(prog='radlm')
    p.add_argument('--version_radlm', action='version', version='RADLM language ' + radlmLang.version)

    subs_p = p.add_subparsers(dest='cmd', title='subcommands')
    
    #weaver option
    weavep = subs_p.add_parser('weave', help='weave radl files')
    weavep.set_defaults(func=weave_radl)

    weavep.add_argument('project_dir', help='the project directory containing radl files and user source code')
    weavep.add_argument('radlm_file', help='the RADLM file used to transform the RADL source file')
 
    #compiler option
    compilep = subs_p.add_parser('compile', help='generate files from specificaiton')
    compilep.set_defaults(func=compile_spec)

    compilep.add_argument('project_dir', help='the project directory containing radl files and user source code')
    compilep.add_argument('spec_file', help='the specification used to generate radlm files and step functions')
    

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
