'''
Created on Oct, 2014

@author: Léonard Gérard leonard.gerard@sri.com


Check and get the module setting object.

'''
from pathlib import Path

from radler.radlr import infos
from radler.radlr.errors import error
from radler.radlr.rast import AstVisitor


def module_settings(visitor, node, seen):
    if seen:
        error("Only one module_settings object is allowed per module.",
              node._location)
    mbp = node['MODULE_BASE_PATH']
    if mbp:
        infos.module_base_path /= Path(mbp._val)

def do_pass(ast):
    visitor = AstVisitor({'module_settings' : module_settings}, kind='red')
    visitor.visit(ast, False)
