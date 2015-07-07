'''
Created on Dec 16, 2014

@author: lgerard
'''

from radler.astutils.tools import link_path
from radler.radlr import infos


def ws_path(relative_filepath):
    return infos.ws_dir / relative_filepath

def ws_link_pervasives():
    link_path(ws_path('radl_lib'), infos.lib_dir, relative=infos.relative_links)
    link_path(ws_path('radlast_4_radl'), infos.radl_module(), relative=infos.relative_links)
    link_path(ws_path('kestrel_lib'), infos.kestrel_lib_dir, relative=infos.relative_links)


def ws_rospath(relative_filepath):
    return infos.ws_dir / 'ros' / relative_filepath

def ws_link_rospervasives():
    link_path(ws_rospath('radl'), infos.radl_rosmodule())