'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from pathlib import Path

from radler.astutils.tools import write_file, listjoin, relative_path
from radler.radlr import infos
from radler.radlr.gen_utils import qn
from radler.radlr.gen_utils.user_sources import    gather_node_user_file
from radler.radlr.rast import AstVisitor, follow_links
from radler.radlr.ros.rosnode import gennode


def app(d, templates):
    for (s,t) in templates.items():
        v = t.format(**d)
        if s not in d or not d[s]:
            d[s] = v
        else:
            d[s] += v

def clear(d, templates):
    for s in templates:
        d[s] = ''


## cmake (top) level
#
# module : the module name
# find_modules : the modules to be found with catkin
# run_modules : the modules to add as run dependencies of this package


cmake_templates = {
'cmakeliststxt': """
cmake_minimum_required(VERSION 2.8.12)
project({module})

find_package(catkin REQUIRED {find_modules} {ros_modules})

add_definitions(-DIN_RADL_GENERATED_CONTEXT)
"""
"{msgs_gen}"
"""
catkin_package(CATKIN_DEPENDS {run_modules})

include_directories(
  include
  PRIVATE ${{catkin_INCLUDE_DIRS}} src
)"""
"{node_defs}"
"""

install(TARGETS
  {to_install}
  ARCHIVE DESTINATION ${{CATKIN_PACKAGE_LIB_DESTINATION}}
  LIBRARY DESTINATION ${{CATKIN_PACKAGE_LIB_DESTINATION}}
  RUNTIME DESTINATION ${{CATKIN_PACKAGE_BIN_DESTINATION}}
)

install(FILES
  {extra_files}
  DESTINATION ${{CATKIN_PACKAGE_SHARE_DESTINATION}}
)
"""
}

# To be applied only if msg_files is not empty
# msg_files : the msg files to be generated

cmake_msgs_templates = {
'msgs_gen':"""
add_message_files(FILES {msg_files})
generate_messages(DEPENDENCIES {ros_modules})
"""
}

## node level
#
# node_name : the node name
# node_target : the node target name
# node_h_filename : the generated header filename
# node_dirs : the node include directories
# node_sources : the node executable sources
# node_libs : the node libraries needed to build it
# node_find_libs : the node libraries dependency resolving

node_templates_cmake_sublevel = {
'to_install':
" {node_target}"
,
'node_defs':"""
get_target_property({node_user_src_var} {node_module_lib} radl_user_src)"""
"{node_find_libs}"
"""
add_executable({node_target} {node_sources})
set_target_properties({node_target} PROPERTIES OUTPUT_NAME {node_name})
target_include_directories({node_target} PUBLIC {node_dirs} PRIVATE {node_gen_folder})
add_dependencies({node_target} {module}_generate_messages_cpp)
target_compile_definitions({node_target}
  PRIVATE RADL_MODULE_NAME={node_modulename}
  PRIVATE RADL_MODULE={node_module_ast_fun}\(\)
  PRIVATE RADL_NODE_NAME={node_name}
  PRIVATE RADL_NODE_QNAME={node_qname}
  PRIVATE RADL_HEADER="{node_h_filename}"
  PRIVATE RADL_STATE={node_user_state_type}
  PRIVATE RADL_STEP_FUN={node_user_step_fun}
  PRIVATE RADL_INIT_FUN={node_user_init_fun}
  PRIVATE RADL_FINISH_FUN={node_user_finish_fun}
)
target_link_libraries({node_target}
  ${{catkin_LIBRARIES}} {node_libs}
)"""
}


def _from_node(visitor, node, d):
    nodemodule = node._qname.rootmodule()
    d['node_name'] = node._name
    d['node_qname'] = str(node._qname)
    d['node_modulename'] = nodemodule.name()
    d['node_module_lib'] = qn.cmake_ast(nodemodule)
    d['node_target'] = qn.cmake_rosnode(node)
    d['node_module_ast_fun'] = qn.c_astfun(nodemodule)
    d['node_user_state_type'] = qn.c_user_state(node)
    d['node_user_step_fun'] = qn.c_user_step(node)
    d['node_user_init_fun'] = qn.c_user_init(node)
    d['node_user_finish_fun'] = qn.c_user_finish(node)

    node_h_path, node_cpp_path = gennode(node, d['_pub_node'])
    d['node_h_filename'] = node_h_path.name
    d['node_sources'] = str(relative_path(node_cpp_path, d['_localroot']))
    d['node_gen_folder'] = str(relative_path(node_h_path.parent, d['_localroot']))

    d['node_user_src_var'] = qn.cmake_user_src(node)
    user_src_path = Path("${" + d['node_user_src_var'] + "}")

    srcs, dirs, libs, find_libs = gather_node_user_file(node, user_src_path)
    d['node_dirs'] = dirs
    d['node_sources'] += srcs
    d['node_libs'] = libs
    d['node_find_libs'] = find_libs

    app(d, node_templates_cmake_sublevel)


def gen(localroot, msg_list, msg_dir, ast, generate_all, pub_node, extra_files=None):

    if generate_all:
        onleaf = onleaf=follow_links(AstVisitor.leaf_bf)  # @UndefinedVariable
    else:
        onleaf = AstVisitor.leaf_bf  # @UndefinedVariable

    visitor = AstVisitor({'node' : _from_node}, kind='bf', onleaf=onleaf)

    d = {'module'     : ast._name,
         'module_lib' : qn.cmake_ast(ast._qname.rootmodule()),
         'ast_fun'    : qn.c_astfun(ast),
         '_localroot' : localroot,
         '_pub_node'  : pub_node}

    clear(d, cmake_templates)
    clear(d, cmake_msgs_templates)
    clear(d, node_templates_cmake_sublevel)

    visitor.visit(ast, d)

    if generate_all:
        # Everything is included, no external dependencies.
        toload = []
    else:
        toload = infos.loaded_modules
    loaded_modules = ' '.join(qn.cmake_ast(n) for n in toload)
    d['find_modules'] = d['module_lib'] + ' radl_lib roscpp ' + loaded_modules
    d['run_modules'] = ' roscpp'
    d['ros_modules'] = ' '.join(n.name() for n in toload)

# Trying to be smart make dependencies hard since otherwise we can't blindly
# add any ros package as a message dependency.
#     if len(msg_list) > 0:

    msg_files = (str(relative_path(m, msg_dir)) for m in msg_list)
    d['msg_files'] = listjoin(' ', msg_files)
    d['extra_files'] = listjoin(' ', extra_files) if extra_files else ''
    d['find_modules'] += ' message_generation'
    d['run_modules'] += ' message_runtime'

    app(d, cmake_msgs_templates)

    app(d, cmake_templates)

    write_file(localroot / "CMakeLists.txt", d['cmakeliststxt'])


