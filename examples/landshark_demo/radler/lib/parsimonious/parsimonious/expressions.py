"""Subexpressions that make up a parsed grammar

These do the parsing.

"""
# TODO: Make sure all symbol refs are local--not class lookups or
# anything--for speed. And kill all the dots.

import re

from parsimonious.exceptions import ParseError, IncompleteParseError
from parsimonious.nodes import Node, RegexNode
from parsimonious.utils import StrAndRepr


__all__ = ['Expression', 'Literal', 'Regex', 'Sequence', 'OneOf', 'Lookahead',
           'Not', 'Optional', 'ZeroOrMore', 'OneOrMore']

def is_internal(name):
    """ By convention, expressions are considered internal if their name begins
    with two underscore.
    """
    return len(name)>1 and name[0] == name[1] == '_'

def as_rhs(expr):
    """ Display expr as it would be written in a rhs of a grammar rule.
    If its name is internal, decompose the expression otherwise use it.
    """
    return expr._as_rhs() if is_internal(expr.name) else expr.name

class Expression(StrAndRepr):
    """A thing that can be matched against a piece of text"""

    # Slots are about twice as fast as __dict__-based attributes:
    # http://stackoverflow.com/questions/1336791/dictionary-vs-object-which-is-more-efficient-and-why

    # By convention kinds beginning with __ are treated like subexpressions
    __slots__ = ['name']

    def __init__(self, name=''):
        if name is '':
            name = "__" + self.__class__.__name__ + "__"
        self.name = name

    def parse(self, text, pos=0):
        """Return a parse tree of ``text``.

        Raise ``ParseError`` if the expression wasn't satisfied. Raise
        ``IncompleteParseError`` if the expression was satisfied but didn't
        consume the full string.

        """
        node = self.match(text, pos=pos)
        if node.end < len(text):
            e = IncompleteParseError(text, node.end, self)
            e.node = node
            raise e
        return node

    def match(self, text, pos=0):
        """Return the parse tree matching this expression at the given
        position, not necessarily extending all the way to the end of ``text``.

        Raise ``ParseError`` if there is no match there.

        :arg pos: The index at which to start matching

        """
        error = ParseError(text)
        node = self._match(text, pos, {}, error)
        if node is None:
            raise error
        return node

    def _match(self, text, pos, cache, error):
        """Internal-only guts of ``match()``

        :arg cache: The packrat cache::

            {(oid, pos): Node tree matched by object `oid` at index `pos` ...}

        :arg error: A ParseError instance with ``text`` already filled in but
            otherwise blank. We update the error reporting info on this object
            as we go. (Sticking references on an existing instance is faster
            than allocating a new one for each expression that fails.) We
            return None rather than raising and catching ParseErrors because
            catching is slow.
        """
        # TODO: Optimize. Probably a hot spot.
        #
        # Is there a way of looking up cached stuff that's faster than hashing
        # this id-pos pair?
        #
        # If this is slow, think about the array module. It might (or might
        # not!) use more RAM, but it'll likely be faster than hashing things
        # all the time. Also, can we move all the allocs up front?
        #
        # To save space, we have lots of choices: (0) Quit caching whole Node
        # objects. Cache just what you need to reconstitute them. (1) Cache
        # only the results of entire rules, not subexpressions (probably a
        # horrible idea for rules that need to backtrack internally a lot). (2)
        # Age stuff out of the cache somehow. LRU? (3) Cuts.
        expr_id = id(self)
        node = cache.get((expr_id, pos), ())  # TODO: Change to setdefault to prevent infinite recursion in left-recursive rules.
        if node is ():
            node = self._uncached_match(text, pos, cache, error)
            cache[(expr_id, pos)] = node

        # Record progress for error reporting:
        # Strict pos > implies getting the deepest, loose >= get the topmost
        if node is None and pos >= error.pos: #We have an error and we progress
            # Don't record __ rules unless that's all we've seen so far.
            notworth = (is_internal(self.name) and hasattr(error.expr, 'name'))
            if not notworth or self.name == '__OneOf__':
                error.expr = self
                error.pos = pos

        return node

    def __unicode__(self):
        return '<%s %s at 0x%s>' % (
            self.__class__.__name__,
            self.as_rule(),
            id(self))

    def as_rule(self):
        """Return the left- and right-hand sides of a rule that represents me.

        Return unicode. If I have no ``name``, omit the left-hand side.

        """
        return (('%s = %s' % (self.name, self._as_rhs())) if self.name else
                self._as_rhs())

    def _as_rhs(self):
        """Return the right-hand side of a rule that represents me.

        Implemented by subclasses.

        """
        raise NotImplementedError

class Epsilon(Expression):
    """ Match the empty string.
    """
    __slots__ = []

    def _uncached_match(self, text, pos, cache, error):
        return Node(self.name, text, pos, pos)

    def _as_rhs(self):
        return "()"


class Literal(Expression):
    """A string literal

    Use these if you can; they're the fastest.

    """
    __slots__ = ['literal']

    def __init__(self, literal, name=''):
        super(Literal, self).__init__(name)
        self.literal = literal

    def _uncached_match(self, text, pos, cache, error):
        if text.startswith(self.literal, pos):
            return Node(self.name, text, pos, pos + len(self.literal))

    def _as_rhs(self):
        # TODO: Get backslash escaping right.
        return '"%s"' % self.literal


class Regex(Expression):
    """An expression that matches what a regex does.

    Use these as much as you can and jam as much into each one as you can;
    they're fast.

    """
    __slots__ = ['re']

    def __init__(self, pattern, name='', ignore_case=False, locale=False,
                 multiline=False, dot_all=False, str=False, verbose=False):
        super(Regex, self).__init__(name)
        self.re = re.compile(pattern, (ignore_case and re.I) |
                                      (locale and re.L) |
                                      (multiline and re.M) |
                                      (dot_all and re.S) |
                                      (str and re.U) |
                                      (verbose and re.X))

    def _uncached_match(self, text, pos, cache, error):
        """Return length of match, ``None`` if no match."""
        m = self.re.match(text, pos)
        if m is not None:
            span = m.span()
            node = RegexNode(self.name, text, pos, pos + span[1] - span[0])
            node.match = m  # TODO: A terrible idea for cache size?
            return node

    def _regex_flags_from_bits(self, bits):
        """Return the textual equivalent of numerically encoded regex flags."""
        flags = 'tilmsux'
        return ''.join(flags[i] if (1 << i) & bits else '' for i in range(6))

    def _as_rhs(self):
        # TODO: Get backslash escaping right.
        return '~"%s"%s' % (self.re.pattern,
                            self._regex_flags_from_bits(self.re.flags))


class _Compound(Expression):
    """An abstract expression which contains other expressions"""

    __slots__ = ['members']

    def __init__(self, *members, **kwargs):
        """``members`` is a sequence of expressions."""
        super(_Compound, self).__init__(kwargs.get('name', ''))
        self.members = members

    def _members_as_rhs(self):
        """Return an iterable of my unicode-represented children, stopping
        descent when we hit a named node so the returned value resembles the
        input rule."""
        return (as_rhs(m) for m in self.members)

    def _child_as_rhs(self):
        """ return the only child as rhs """
        assert(len(self.members) == 1)
        return as_rhs(self.members[0])


class Sequence(_Compound):
    """A series of expressions that must match contiguous, ordered pieces of
    the text

    In other words, it's a concatenation operator: each piece has to match, one
    after another.

    """
    def _uncached_match(self, text, pos, cache, error):
        new_pos = pos
        length_of_sequence = 0
        children = []
        for m in self.members:
            node = m._match(text, new_pos, cache, error)
            if node is None:
                return None
            children.append(node)
            length = node.end - node.start
            new_pos += length
            length_of_sequence += length
        # Hooray! We got through all the members!
        return Node(self.name, text, pos, pos + length_of_sequence, children)

    def _as_rhs(self):
        return '(' + ' '.join(self._members_as_rhs()) + ')'

class OneOf(_Compound):
    """A series of expressions, one of which must match

    Expressions are tested in order from first to last. The first to succeed
    wins.

    """
    def _uncached_match(self, text, pos, cache, error):
        for m in self.members:
            node = m._match(text, pos, cache, error)
            if node is not None:
                # Wrap the succeeding child in a node representing the OneOf:
                return Node(self.name, text, pos, node.end, children=[node])

    def _as_rhs(self):
        return ' / '.join(self._members_as_rhs())


class Lookahead(_Compound):
    """An expression which consumes nothing, even if its contained expression
    succeeds"""

    # TODO: Merge this and Not for better cache hit ratios and less code.
    # Downside: pretty-printed grammars might be spelled differently than what
    # went in. That doesn't bother me.

    def _uncached_match(self, text, pos, cache, error):
        node = self.members[0]._match(text, pos, cache, error)
        if node is not None:
            return Node(self.name, text, pos, pos)

    def _as_rhs(self):
        return '&(%s)' % self._child_as_rhs()


class Not(_Compound):
    """An expression that succeeds only if the expression within it doesn't

    In any case, it never consumes any characters; it's a negative lookahead.

    """
    def _uncached_match(self, text, pos, cache, error):
        # FWIW, the implementation in Parsing Techniques in Figure 15.29 does
        # not bother to cache NOTs directly.
        node = self.members[0]._match(text, pos, cache, error)
        if node is None:
            return Node(self.name, text, pos, pos)

    def _as_rhs(self):
        # TODO: Make sure this parenthesizes the member properly if it's an OR
        # or AND.
        return '!(%s)' % self._child_as_rhs()


# Quantifiers. None of these is strictly necessary, but they're darn handy.

class Optional(_Compound):
    """An expression that succeeds whether or not the contained one does

    If the contained expression succeeds, it goes ahead and consumes what it
    consumes. Otherwise, it consumes nothing.

    """
    def _uncached_match(self, text, pos, cache, error):
        node = self.members[0]._match(text, pos, cache, error)
        return (Node(self.name, text, pos, pos) if node is None else
                Node(self.name, text, pos, node.end, children=[node]))

    def _as_rhs(self):
        return '(%s)?' % self._child_as_rhs()


# TODO: Merge with OneOrMore.
class ZeroOrMore(_Compound):
    """An expression wrapper like the * quantifier in regexes."""
    def _uncached_match(self, text, pos, cache, error):
        new_pos = pos
        children = []
        while True:
            node = self.members[0]._match(text, new_pos, cache, error)
            if node is None or not (node.end - node.start):
                # Node was None or 0 length. 0 would otherwise loop infinitely.
                return Node(self.name, text, pos, new_pos, children)
            children.append(node)
            new_pos += node.end - node.start

    def _as_rhs(self):
        return '(%s)*' % self._child_as_rhs()


class OneOrMore(_Compound):
    """An expression wrapper like the + quantifier in regexes.

    You can also pass in an alternate minimum to make this behave like "2 or
    more", "3 or more", etc.

    """
    __slots__ = ['min']

    # TODO: Add max. It should probably succeed if there are more than the max
    # --just not consume them.

    def __init__(self, member, name='', min=1):
        super(OneOrMore, self).__init__(member, name=name)
        self.min = min

    def _uncached_match(self, text, pos, cache, error):
        new_pos = pos
        children = []
        while True:
            node = self.members[0]._match(text, new_pos, cache, error)
            if node is None:
                break
            children.append(node)
            length = node.end - node.start
            if length == 0:  # Don't loop infinitely.
                break
            new_pos += length
        if len(children) >= self.min:
            return Node(self.name, text, pos, new_pos, children)

    def _as_rhs(self):
        return '(%s)+' % self._child_as_rhs()
