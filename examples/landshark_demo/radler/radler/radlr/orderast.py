'''
Created on Nov, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Check circles
'''
from radler.radlr import infos
from radler.radlr.errors import error
from radler.radlr.rast import Alias, AstVisitor, Ident


def _is_current_module(node):
    return node._qname.has_root(infos.ast._qname)  # @UndefinedVariable

def mark_term(term, deps):
    qn = term._qname
    if qn in deps:
        error("The term {} depends on itself.".format(qn),
              term._location)
    else:
        return deps + [qn]

def onnode(visitor, node, deps):
    deps = mark_term(node, deps)
    visitor.node_bf(node, deps)

def onleaf(visitor, leaf, deps):
    if isinstance(leaf, Alias):
        deps = mark_term(leaf, deps)
        target = leaf._is_alias_of
        if _is_current_module(target):
            visitor.visit(target, deps)
    elif isinstance(leaf, Ident):
        mark_term(leaf, deps)
    else:
        pass

def do_pass(ast):
    v = AstVisitor(default=onnode, onleaf=onleaf, kind='bf')
    v.visit(ast, [])


# '''
# tarjan the definitions
# '''
# from tarjan import tarjan
# 
# from radler.radlr.errors import error
# from radler.radlr.rast import Ident, AstVisitor, Alias
# 
# 
# def collect_node_deps(visitor, node, deps):
#     nq = node._qname
#     deps[nq] = deps.get(nq, list())
#     for c in node._children:
#         deps[nq].append(c._qname)
#     visitor.node_visit(node, deps)
# 
# def collect_leaf_deps(visitor, leaf, deps):
#     if isinstance(leaf, Alias):
#         deps[leaf._qname] = leaf._is_alias_of._qname
# 
# def do_pass(ast):
#     deps = dict()
#     visitor = AstVisitor(default=collect_node_deps,
#                          onleaf=collect_leaf_deps, kind='bf')
#     visitor.visit(ast, deps)

