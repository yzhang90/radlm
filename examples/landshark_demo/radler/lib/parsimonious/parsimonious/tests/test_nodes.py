
from nose.tools import eq_, assert_raises  # @UnresolvedImport

from parsimonious.nodes import Node, NodeVisitor, VisitationError


class HtmlFormatter(NodeVisitor):
    """Visitor that turns a parse tree into HTML fragments"""

    def visit_bold_open(self, node, visited_children):
        return '<b>'

    def visit_bold_close(self, node, visited_children):
        return '</b>'

    def visit_text(self, node, visited_children):
        """Return the text verbatim."""
        return node.text

    def visit_bold_text(self, node, visited_children):
        return ''.join(visited_children)


class ExplosiveFormatter(NodeVisitor):
    """Visitor which raises exceptions"""

    def visit_boom(self, node, visited_children):
        raise ValueError


def test_visitor():
    """Assert a tree gets visited correctly.

    We start with a tree from applying this grammar... ::

        bold_text  = bold_open text bold_close
        text       = ~'[a-zA-Z 0-9]*'
        bold_open  = '(('
        bold_close = '))'

    ...to this text::

        ((o hai))

    """
    text = '((o hai))'
    tree = Node('bold_text', text, 0, 9,
                [Node('bold_open', text, 0, 2),
                 Node('text', text, 2, 7),
                 Node('bold_close', text, 7, 9)])
    result = HtmlFormatter().visit(tree)
    eq_(result, '<b>o hai</b>')


def test_visitation_exception():
    assert_raises(VisitationError,
                  ExplosiveFormatter().visit,
                  Node('boom', '', 0, 0))


def test_str():
    """Test str and unicode of ``Node``."""
    n = Node('text', 'o hai', 0, 5)
    good = '<Node called "text" matching "o hai">'
    eq_(str(n), good)
    eq_(str(n), good)


def test_repr():
    """Test repr of ``Node``."""
    s = 'hai ö'
    boogie = 'böogie'
    n = Node(boogie, s, 0, 3, children=[
            Node('', s, 3, 4), Node('', s, 4, 5)])
    eq_(repr(n), """s = {hai_o}\nNode({boogie}, s, 0, 3, children=[Node('', s, 3, 4), Node('', s, 4, 5)])""".format(hai_o=repr(s), boogie=repr(boogie)))
