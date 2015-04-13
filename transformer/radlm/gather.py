'''
Created on March, 2015

  Gather interceptor information and build a map
  node.qname -> interceptor

'''
from transformer.astutils.nodetrees import fun_dict_of
from transformer.radlm import infos
from transformer.radlm.rast import AstVisitor

_module_settings = None

def interceptor(visitor, node, _):
    node_name = str(node['NODE']._qname)
    infos.interceptors[node_name] = node
  

def collect_interceptors(ast):
    v = AstVisitor(fun_dict_of((interceptor,)), kind='bf')
    v.visit(ast, ())

def node(visitor, node, _):
    node_name = str(node._qname)
    infos.cxx[node_name] = {'CXX': node['CXX'],
                            'MODULE': _module_settings
                           }

def collect_cxx(ast):
    v = AstVisitor(fun_dict_of((node,)), kind='bf')
    v.visit(ast, ())
    # clear module settings for other radl file
    global _module_settings
    _module_settings = None

def module_settings(visitor, node, _):
    global _module_settings 
    _module_settings = node

def collect_module_settings(ast):
    v = AstVisitor(fun_dict_of((module_settings,)), kind='bf')
    v.visit(ast,())
