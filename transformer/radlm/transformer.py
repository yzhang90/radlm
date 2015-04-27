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
           cxx = interceptor['CXX']
           if cxx['PATH']:
               node['CXX']['PATH'] = cxx['PATH']
           if cxx['HEADER']:
               node['CXX']['HEADER'] = cxx['HEADER']
           if cxx['FILENAME']:
               node['CXX']['FILENAME'].extend(cxx['FILENAME'])
           if cxx['LIB']:
               node['CXX']['LIB'].extend(cxx['LIB'])
           if cxx['CLASS']:
               node['CXX']['CLASS'] = cxx['CLASS']

def transform_linux(visitor, node, _):
    qname = str(node._qname)
    if node._name.startswith('_'):
       qname = qname.replace('.'+node._name, '')
    implant = infos.implants.get(qname)
    if implant:
       for n in implant:
           node['NODES'].append(n)

def do_pass(ast):
    v = AstVisitor({'node' : transform_node,
                    'linux': transform_linux}, kind='bf')
    v.visit(ast, ())
