'''
Created on March, 2015

Transform RADL file based on interceptors information

'''

from radlm.weaver.rast import AstVisitor
from radlm.weaver import infos


def weave_node(visitor, node, _):
    interceptor = infos.interceptors.get(str(node._qname))
    if interceptor:
       infos.weaved[str(interceptor._qname)] = True
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


def weave_linux(visitor, node, _):
    qname = str(node._qname)
    if node._name.startswith('_'):
       qname = qname.replace('.'+node._name, '')
    implant = infos.implants.get(qname)
    if implant:
       infos.weaved[str(implant._qname)] = True
       for n in implant['NODES']:
           node['NODES'].append(n)

def do_pass(ast):
    v = AstVisitor({'node' : weave_node,
                    'linux': weave_linux}, kind='bf')
    v.visit(ast, ())
