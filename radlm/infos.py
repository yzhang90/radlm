'''
Created on March, 2015

    Module used to store global properties.

'''

from pathlib import Path

from radlm.astutils.names import RootNamespace


########## Common ##########

calling_dir = None
"The directory where radlm script was called (it is set by the main)."

ws_dir = None
"The destination Path directory"

source_file = Path()
"The source Path file being processed now."

########## Compiler ##########


########## Weaver ##########

##########
# Global properties concerning everything which is loaded
##########

root_namespace = RootNamespace()
"The global root namespace."

interceptors = dict()
"Mapping from node qname to its corresponding interceptor node."

implants = dict()
"Mapping from location in the plant to node qname to be appended."

weaved = dict()
"Statistics for whether an interceptor/implant is weaved"

cxx = dict()
"Mapping from node qname to its corresponding cxx info."

semantics = None
"Semantics object for the language. It is created during bootstrapping."


##########
# Global properties concerning the current source file.
##########

radl_ast = None
"The radl ast. Actually set by the parser."

radlm_ast = None
"The radlm ast. Actually set by the parser."

