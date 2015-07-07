'''
Created on Jun, 2014
@author: Léonard Gérard leonard.gerard@sri.com

    Utilities to handle file paths, filenames, etc.
'''

from radler.radlr.errors import internal_error
from radler.radlr.workspace import ws_rospath


def qn_topic(qname):
    return qname.qname('/', root='/')


def qn_file(qname, prefix='', suffix=''):
    p = list(qname.pathwalk())
    if len(p) < 2:
        internal_error("trying to use a qn with size < 2 for file gen.")
    return ws_rospath('/'.join(p[0:-1]) + '/src/' + prefix + p[-1] + suffix)

def qn_srcfile(qname, prefix='', suffix=''):
    return qn_file(qname, prefix, suffix)


def msg_folder(package_folder):
    return package_folder / 'msg'

def msg_msg_file(package_folder, msg_name):
    return msg_folder(package_folder) / (msg_name + '.msg')

def msg_cpp_header(msg_package_name, msg_name):
    return msg_package_name + '/' + msg_name + '.h'

def msg_cpp_qname(msg_package_name, msg_name):
    return msg_package_name + '::' + msg_name

def msg_ros_qname(msg_package_name, msg_name):
    return msg_package_name + '/' + msg_name