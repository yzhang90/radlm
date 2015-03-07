'''
Created on March, 2015

    Module used to store global properties.

'''

from pathlib import Path

from transformer.astutils.names import RootNamespace


calling_dir = None
"The directory where radlm script was called (it is set by the main)."

script_dir = Path(__file__).absolute()
"The radler Path directory."

##########
# Global properties concerning everything which is loaded
##########

root_namespace = RootNamespace()
"The global root namespace."

interceptors = dict()
"Mapping from node qname to its corresponding interceptor node."

semantics = None
"Semantics object for the language. It is created during bootstrapping."


##########
# Global properties concerning the current source file.
##########

radl_ast = None
"The radl ast. Actually set by the parser."

radlm_ast = None
"The radlm ast. Actually set by the parser."

source_file = Path()
"The source Path file being processed now."
