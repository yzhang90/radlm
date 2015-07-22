'''
Created on March, 2015

  Gather interceptor information and build a map
  node.qname -> interceptor

'''
from radlm.astutils.nodetrees import fun_dict_of
from radlm.weaver import infos
from radlm.weaver.rast import AstVisitor

_module_settings = None

def interceptor(visitor, node, _):
    node_name = str(node['NODE']._qname)
    infos.interceptors[node_name] = node
    infos.weaved[str(node._qname)] = False 
  

def collect_interceptors(ast):
    v = AstVisitor(fun_dict_of((interceptor,)), kind='bf')
    v.visit(ast, ())

def implant(visitor, node, _):
    location = str(node['LOCATION']._qname)
    infos.implants[location] = node
    infos.weaved[str(node._qname)] = False

def collect_implants(ast):
    v = AstVisitor(fun_dict_of((implant,)), kind='bf')
    v.visit(ast, ())

def node(visitor, node, _):
    node_name = str(node._qname)
    infos.cxx[node_name] = {'NODE': node,
                            'MODULE': _module_settings}

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
