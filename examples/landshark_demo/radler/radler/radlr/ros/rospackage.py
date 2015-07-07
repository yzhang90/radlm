'''
Created on Dec, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
from radler.astutils.tools import ensure_dir
from radler.radlr import infos
from radler.radlr.gen_utils import catkin_pkg
from radler.radlr.ros import rosmsg, roscmake, rosutils
from radler.radlr.workspace import ws_rospath


def do_pass(ast, pub_node):

    package_name = ast._name
    package_folder = ws_rospath(package_name)
    msg_folder = rosutils.msg_folder(package_folder)
    ensure_dir(msg_folder) # Catkin is not happy without a msg folder

    # Generate the package.xml file
    build_deps = ['message_generation', 'message_runtime', 'roscpp', 'radl_lib']
    run_deps = ['message_runtime', 'roscpp']

    # Modular compilation, we have to add dependencies
    for p in infos.loaded_modules:
        build_deps.append(p)

    catkin_pkg.gen(package_folder, package_name, build_deps, run_deps)

    # Generate the message files
    msg_list = rosmsg.generate_package_msg_files(package_folder, package_name, ast)

    # Generate the nodes and cmake file
    roscmake.gen(package_folder, msg_list, msg_folder, ast, False, pub_node)
