'''
Created on March, 2015

Strip RADLM file by removing interceptors definition

'''

from transformer.radlm.rast import AstVisitor


def strip_interceptor(visitor, node, _):
    for n in node._children:
       if n._kind == 'interceptor' :
           node._children.remove(n)
 
def do_pass(ast):
    v = AstVisitor({'_ast' : strip_interceptor}, kind='bf')
    v.visit(ast, ())
