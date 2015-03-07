'''
Created on March, 2015

1.Gather interceptor information and build a map
  node.qname -> interceptor

'''
from transformer.astutils.nodetrees import fun_dict_of
from transformer.radlm import infos
from transformer.radlm.rast import AstVisitor


def interceptor(visitor, node, _):
    node_name = str(node['NODE']._qname)
    infos.interceptors[node_name] = node
  

def do_pass(ast):
    v = AstVisitor(fun_dict_of((interceptor,)), kind='bf')
    v.visit(ast, ())
