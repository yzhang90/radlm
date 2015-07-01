'''
Created on March, 2015

'''

from transformer.radlm.rast import AstNode, AstVisitor
from transformer.astutils.nodetrees import fun_dict_of
from transformer.astutils.names import NonExistingIdent
from transformer.radlm import infos

def pretty_print(node, indchar='  ', indent=0):
    """ pretty print a node
    @Return a string
    """
    def _ast(visitor, node, indent):
        return visitor.visit(node._children, indent)

    def _alias(visitor, node, indent):
        s = indchar*indent + node._name + " = " + str(node._val) + '\n'
        return s, indent

    def print_type(visitor, node, indent):
        if node._get_internal('kind_annot', None):
            s = indchar*indent + node._typed_name + ' '
        else:
            s = indchar*indent
        if node._kind == 'string':
            s += '"' + node._val + '"'
        else:
            s += node._val
        if indent == 0:
            s += '\n'
        return s, indent

    def print_node(visitor, node, indent):
        name = node._typed_name
        if name.startswith('_'):
            s = indchar*indent
        else:
            s = indchar*indent + node._typed_name + ' '
        s += '{\n'
        sc, _ = visitor.visit(node._children, indent+1)
        s += sc
        s += indchar*indent + '}'
        if indent == 0:
            s += '\n'
        return s, indent

    def print_list(visitor, l, indent):
        s = ''
        for item in l:
            sl, _ = visitor.visit(item, indent) 
            s += sl + '\n'
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
        node_name = n._name
        try:
            infos.root_namespace.lookup_node(n._qname)
        except NonExistingIdent:
           node_name = str(n._qname)
        s = indchar*indent + node_name
        if indent == 0:
            s += '\n'
        return s, indent

    visitor_dict = fun_dict_of((_ast, _alias,))

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


def ensure_dir(path):
    """{path} is expected to be a libpath.Path object"""
    if path.exists():
        if not path.is_dir():
            raise Exception("{} should be a directory".format(dir))
    else:
        ensure_dir(path.parent)
        path.mkdir()


def write_file(filepath, filecontent):
    d = filepath.parent
    ensure_dir(d)
    with filepath.open('w', encoding='UTF-8') as f:
        f.write(filecontent)

