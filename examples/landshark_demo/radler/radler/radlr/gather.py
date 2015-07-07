'''
Created on Jan, 2015

@author: Léonard Gérard leonard.gerard@sri.com


This pass gather informations.

Currently gather the publisher information.

'''
from radler.astutils.nodetrees import fun_dict_of
from radler.radlr import infos
from radler.radlr.errors import warning
from radler.radlr.rast import AstVisitor


def node(visitor, node, _):
    for pub in node['PUBLISHES']:
        top = pub['TOPIC']
        if top._qname in infos.publishers:
            warning("Topic {} has multiple publishers."
                    "  -> Be cautious with the --ROS backend."
                    "".format(top), top._location)
        infos.publishers[top._qname] = node._qname


def do_pass(ast):
    v = AstVisitor(fun_dict_of((node,)), kind='bf')
    v.visit(ast, ())
