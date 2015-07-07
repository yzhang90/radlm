'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Verify struct and topic have named fields.
'''
from radler.radlr.errors import error
from radler.radlr.rast import Ident, AstVisitor


def onstruct(visitor, node, _):
    for f in node['FIELDS']:
        if isinstance(f, Ident):
            error("Struct and topic fields must be named.\n"
                  "No Ident allowed, only aliases.", f._location)

def do_pass(ast):
    visitor = AstVisitor({'struct': onstruct, 'topic': onstruct}, kind='bf')
    visitor.visit(ast)
