
from parsimonious.grammar import Grammar

class Compiler:
    def __init__(self, specLang):
        self.defs = specLang.defs

    def parse(self, spec):
        node = Grammar(self.defs, '_spec').parse(spec)
        print(node)
