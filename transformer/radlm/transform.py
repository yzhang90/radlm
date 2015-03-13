'''
Created on March, 2015

Transform RADL file based on interceptors information

'''

from transformer.radlm.rast import AstVisitor
from transformer.radlm import infos


def transform_node(visitor, node, _):
    interceptor = infos.interceptors.get(str(node._qname))
    if interceptor:
       if interceptor['PUBLISHES']:
           for pub in interceptor['PUBLISHES']:
               node['PUBLISHES'].append(pub)
       if interceptor['SUBSCRIBES']:
           for sub in interceptor['SUBSCRIBES']:
               node['SUBSCRIBES'].append(sub)
       if interceptor['CXX']:
           node._children['CXX'] = interceptor['CXX']

def do_pass(ast):
    v = AstVisitor({'node' : transform_node}, kind='bf')
    v.visit(ast, ())
