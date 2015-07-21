'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Working Directory module.

The pass adds a working directory attribute '_wd' to every nodes,
storing to the user path associated. All paths are Path from pathlib.

Once the pass is done, the main function is 'of'.
- To get the working directory, use wd.of(node). The returned path is absolute
only if the user specified it so.
- To get an absolute path, do infos.module_base_path / wd.of(node).
'''

from pathlib import Path

from radlm.weaver.rast import AstVisitor


def _wd(visitor, node, wd):
    p = node.get('PATH', None)
    subdir = p._val if p else ''
    wd = wd / subdir
    node._wd = wd
    return visitor.node_bf(node, wd)

def do_pass(ast):
    """ Add a _wd attribute to nodes indicating current user path."""
    visitor = AstVisitor(default=_wd, kind='bf')
    visitor.visit(ast, Path())


def of(node):
    """This function return the user path of node.
    Note that the pass needs to have been done before any call to this fun.
    """
    return node._wd
