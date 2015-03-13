'''
Created on March, 2015

'''

from transformer.radlm.rast import AstNode, AstVisitor
from transformer.astutils.nodetrees import fun_dict_of


def pretty_print(node, indchar='  ', indent=0):
    """ pretty print a node
    @Return a string
    """
    def _ast(visitor, node, indent):
        return visitor.visit(node._children, indent)

    def print_type(visitor, node, indent):
        if node._get_internal('kind_annot', None):
            s = indchar*indent + node._typed_name + ' '
        else:
            s = indchar*indent
        if node._kind == 'string':
            s += '"' + node._val + '"'
        else:
            s += node._val
        return s, indent

    def print_node(visitor, node, indent):
        s = indchar*indent + node._typed_name + ' {\n';
        sc, _ = visitor.visit(node._children, indent+1)
        s += sc
        s += indchar*indent + '}\n'
        return s, indent

    def print_list(visitor, l, indent):
        s = ''
        for item in l:
            sl, _ = visitor.visit(item, indent) 
            s += sl
        return s, indent

    def print_dict(visitor, d, indent):
        s = ''
        for key in d:
            if d[key] :
                s += indchar*indent + key + '\n'
                sd, _ = visitor.visit(d[key], indent+1)
                s += sd + '\n'
        return s, indent

    def print_leaf(visitor, n, indent):
        s = indchar*indent + n._name
        return s, indent

    visitor_dict = fun_dict_of((_ast,))

    types = ['int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64',
             'float32', 'float64', 'bool', 'string', 'duration', 'time', 'ip']

    for ty in types: visitor_dict[ty] = print_type

    visitor = AstVisitor(visitor_dict,
                         default=print_node, 
                         onlist=print_list,
                         ondict=print_dict,
                         onleaf=print_leaf,
                         kind='mapacc')
    s, _ = visitor.visit(node, indent)
    return s
