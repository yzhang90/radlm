'''
Created on Dec, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from radler.astutils.tools import write_file
from radler.radlr import infos


package_xml_template = ("""<?xml version="1.0"?>
<package>
  <name>{package}</name>
  <version>0.0.1</version>
  <description>Generated from {source}</description>
  <maintainer email="leonard.gerard@sri.com">Léonard Gérard</maintainer>
  <license>GPL</license>"""
"{build_deps}"
"""
  <buildtool_depend>catkin</buildtool_depend>"""
"{run_deps}"
"""
</package>
"""
)

run_dep_template = """
  <run_depend>{package_dep}</run_depend>"""

build_dep_template = """
  <build_depend>{package_dep}</build_depend>"""


def gen(package_folder, package_name, build_dependencies, run_dependencies):
    d = {'source'  : str(infos.source_file),
         'package' : package_name}

    build_deps = ''
    for p in build_dependencies:
        build_deps += build_dep_template.format(package_dep=p)
    d['build_deps'] = build_deps

    run_deps = ''
    for p in run_dependencies:
        run_deps += run_dep_template.format(package_dep=p)
    d['run_deps'] = run_deps

    package_xml = package_xml_template.format(**d)
    write_file(package_folder / "package.xml", package_xml)
