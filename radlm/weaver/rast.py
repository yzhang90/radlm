'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com

'''
from collections import Mapping

from radlm.astutils.names import NonExistingIdent
from radlm.astutils.nodetrees import Functor
from radlm.weaver import infos
from radlm.weaver.errors import internal_error, noloc


class AlreadyAttached(Exception): pass


class Ident(Mapping):
    """" An ident is a reference to an actual node, it bears the same name
    but has its own location"""
    __slots__ = ['_location', '_node']
    def __init__(self, node, location):
        self._location = location
        self._node = node
    def __getstate__(self):
        """ Allows pickling: necessary because of __getattr__"""
        return tuple(getattr(self, a) for a in Ident.__slots__)
    def __setstate__(self, state):
        """ Allows pickling: necessary because of __getattr__"""
        for a,v in zip(Ident.__slots__, state): setattr(self, a, v)

    @classmethod
    def of(cls, node_or_ident):
        if isinstance(node_or_ident, Ident):
            return Ident(node_or_ident._node, noloc())
        else:
            return Ident(node_or_ident, noloc())

    def _attach(self, node):
        if self._node:
            raise AlreadyAttached()
        self._node = node
    def _reattach(self, node):
        self._node = node

    def __repr__(self):
        return str(self._qname)
    #container convention, behave like node
    def __len__(self):
        return max(len(self._node),1)
    def __getitem__(self, key):
        return self._node[key]
    def __iter__(self):
        return iter(self._node)
    def __setattr__(self, attr, value):
        if attr in self.__slots__:
            object.__setattr__(self, attr, value)
        else:
            setattr(self._node, attr, value)
    def __getattr__(self, attr):
        return getattr(self._node, attr)
    def __copy__(self):
        internal_error("Trying to copy an Ident.")
    def __deepcopy__(self, d):
        internal_error("Trying to deepcopy an Ident.")

    def __eq__(self, other):
        return self._qname == getattr(other, '_qname' , None)


_unique_val = object()

class Alias(Mapping):
    """ Note that Alias have their own fields when edited.
    Some reserved fields are _qname, _location, _is_alias_of.
    """
    def __init__(self, qname, location, target_ident):
        self._qname = qname
        self._location = location
        self._is_alias_of = target_ident
        self._internals = dict()

    @property
    def _name(self):
        return self._qname.name()

    def __getattr__(self, attr):
        """ Fall back to the node """
        return getattr(self._node, attr)

    def _get_internal(self, attr, default=_unique_val):
        if default is _unique_val:
            return self._internals[attr]
        else:
            return self._internals.get(attr, default)
    def _set_internal(self, attr, value):
        self._internals[attr] = value
    def _append_internal(self, attr, value):
        i = self._internals.get(attr, False)
        if i is False:
            i = list()
            self._internals[attr] = i
        i.append(value)

    def __getstate__(self):
        """ Allows pickling: necessary because of __getattr__"""
        return self.__dict__
    def __setstate__(self, state):
        """ Allows pickling: necessary because of __getattr__"""
        self.__dict__.update(state)

    @property
    def _node(self):
        return self._is_alias_of._node
    #container convention, behave like node
    def __len__(self):
        return max(len(self._node),1)
    def __getitem__(self, key):
        return self._node[key]
    def __iter__(self):
        return iter(self._node)
    def __eq__(self, other):
        return self._qname == getattr(other, '_qname' , None)

class AstNode(Mapping):
    """ Basically a named kind with children.
    Inherit its container behavior from its children container.
    """
    def __init__(self, kind, qname, children, namespace, location):
        self._kind = kind
        self._qname = qname
        self._children = children
        self._namespace = namespace
        self._location = location
        self._internals = dict()

    @property
    def _name(self):
        return self._qname.name()

    @property
    def _val(self):
        """ Used for nodes holding one value as their unique child."""
        if len(self._children) != 1:
            internal_error("Tried to get _val of a node with {} childs"
                           "".format(len(self._children)))
        return self._children[0]
    #container convention, behave like _children
    def __len__(self):
        return len(self._children)
    def __getitem__(self, key):
        try:
            return self._children[key]
        except (KeyError, TypeError):
            pass #TypeError is useful in case _children is a list and key a str
        pa = list(iter(self._children))
        raise KeyError("{a} is not among the possibilities:"
                       "{pa}".format(a=key, pa=pa))
    def __setitem__(self, key, value):
        self._children[key] = value
    def __iter__(self):
        return iter(self._children)
    def __getattr__(self, attr):
        """Attributes are namespace lookup.
        """
        #ensure we have a _namespace attribute in case __getattr__ is called
        #before init (for example when using copy).
        object.__getattribute__(self, '_namespace')
        try:
            return self._namespace[attr]
        except NonExistingIdent: pass
        raise AttributeError(attr)

    def _get_internal(self, attr, default=_unique_val):
        if default is _unique_val:
            return self._internals[attr]
        else:
            return self._internals.get(attr, default)
    def _set_internal(self, attr, value):
        self._internals[attr] = value
    def _append_internal(self, attr, value):
        i = self._internals.get(attr, False)
        if i is False:
            i = list()
            self._internals[attr] = i
        i.append(value)

    @property
    def _typed_name(self):
        return "{n} : {k}".format(n=self._name, k=self._kind)

    def __str__(self): pass # This is replaced bellow since we need the class
    #to be defined before we can define a visitor which will give us a nice
    #printing function. 

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        return self._qname == getattr(other, '_qname' , None)

nodetree_kind = Functor(AstNode, '_children', '_kind')
#The visitor for Ast and AstNodes
AstVisitor = nodetree_kind.Visitor

def follow_links(on_other_leaf):
    def _follow_links(visitor, leaf, acc):
        if isinstance(leaf, Ident):
            return visitor.visit(leaf._node, acc)
        elif isinstance(leaf, Alias):
            return visitor.visit(leaf._is_alias_of, acc)
        else:
            return on_other_leaf(visitor, leaf, acc)
    return _follow_links

def follow_modlocal_links(on_other_leaf):
    def _is_current_module(node):
        return node._qname.has_root(infos.radl_ast._qname)  # @UndefinedVariable
    def _follow_links(visitor, leaf, acc):
        if isinstance(leaf, Ident) and _is_current_module(leaf._node):
            return visitor.visit(leaf._node, acc)
        elif isinstance(leaf, Alias) and _is_current_module(leaf._is_alias_of):
            return visitor.visit(leaf._is_alias_of)
        else:
            return on_other_leaf(visitor, leaf, acc)
    return _follow_links

#getting children of a node (flattened, by passing the containers).
flatten_children = nodetree_kind.flatten_children

#Pretty printer for AstNodes register in the class
AstNode.__str__ = Functor(AstNode, '_children', '_typed_name').spprint_node


