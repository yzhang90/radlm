'''
Created on April, 2015

  Parse the spec file and generate radl/radlm files, step functions and etc.

'''

import re
from pathlib import Path

from transformer.radlm import infos, wd
from transformer.radlm.utils import write_file

spec_infos = {'type'       : None,
              'period'     : None,
              'maxlatency' : None,
              'path'       : None,
              'nodes'      : None}

radlm_template = {
'monitor_radlm':
'''
{interceptor_def}
{monitor_def}'''
}

interceptor_template = {
'interceptor_def':
'''{node}_interceptor : interceptor {{
  NODE {node_name}
  PUBLISHES
    {node}_report {{ TOPIC {mtopic_name} }}
  CXX {{
    HEADER "{wrapper_header}"
    CLASS "{wrapper_class}"
    FILENAME "{wrapper_cxx_file}"
  }}
}}

'''
}

monitor_template = {
'monitor_def':
'''health_monitor : node {{
  SUBSCRIBES
{monitor_subscription}
  PERIOD {monitor_period}
  CXX {{
    PATH "{monitor_path}"
    HEADER "HealthMonitor.h"
    CLASS "HealthMonitor"
    FILENAME "HealthMonitor.cpp"
  }}
}}
'''
}

monitor_subscription_template = {
'monitor_subscription':
'''    {node}_report {{ TOPIC {mtopic_name} MAXLATENCY {maxlatency} }}
'''
,'flag_check':
'''
  if(radl_is_timeout(inflags->{node}_report)) {{
    ROS_INFO("node {node_name} is dead");
  }} else {{
    if(radl_is_timeout(in->{node}_report->flag)) {{
      ROS_INFO("node {node_name} received timeout messages");
    }}
  }}
'''
}

topics_template = {
'monitor_topics':
'''
{mtopic}: topic {{
  FIELDS
    flag : uint8 0
}}
'''
}

monitor_code_template = {
'monitor_h_file':
'''#include RADL_HEADER

class HealthMonitor {{
 public:
  void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
}};
'''
,'monitor_cpp_file':
'''#include "HealthMonitor.h"
#include "ros/ros.h"

void HealthMonitor::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                         radl_out_t * out, radl_out_flags_t* outflags) {{
{flag_check}
}}
'''
}

node_wrapper_template = {
'node_h_file':
'''#include RADL_HEADER
#include "{node_header}"

class {wrapper_class} {{
  private:
    {node_class} _node;
  public:
    void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
}};
'''
,'node_cpp_file':
'''#include "{wrapper_header}"
#include "ros/ros.h"

void {wrapper_class}::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                           radl_out_t * out, radl_out_flags_t* outflags) {{
   _node.step(in, inflags, out, outflags);
   out->{node}_report->flag = outflags->{node}_report;
   outflags->{node}_report = radl_STALE;
}}
'''
}


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

def gen(spec):
    lines = spec.splitlines()
    for line in lines:
        matchObj = re.match(r'(.*)\s?=\s?(.*)',line)
        if matchObj:
            key = matchObj.group(1).strip()
            value = matchObj.group(2).strip()
            spec_infos[key] = value
    if spec_infos['type'] == 'health':
        health_gen()
    
def health_gen():
    d = {'monitor_path'       : spec_infos['path'],
         'maxlatency' : spec_infos['maxlatency'],
         'monitor_period': spec_infos['period']}
           
    nodes = spec_infos['nodes'].split(' ')
    for n in nodes:
        clear(d, node_wrapper_template)
        node_cxx = infos.cxx[n]
        nn = n.replace('.','_')
        nw_class = nn.title().replace('_','') + '_Wrapper'
        d['node'] = nn
        d['node_name'] = n
        d['node_header'] = node_cxx['NODE']['CXX']['HEADER']._val
        d['node_class'] = node_cxx['NODE']['CXX']['CLASS']._val
        d['mtopic'] = nn + '_health'
        d['mtopic_name'] = "monitor_topics." + nn + '_health'
        d['wrapper_header'] = nw_class + '.h'
        d['wrapper_class'] = nw_class
        d['wrapper_cxx_file'] = nw_class + '.cpp'
        app(d, interceptor_template)
        app(d, monitor_subscription_template)
        app(d, topics_template)
        app(d, node_wrapper_template)
        write_file(infos.ws_dir / wd.of(node_cxx['NODE']['CXX']) / d['wrapper_header'], d['node_h_file'])
        write_file(infos.ws_dir / wd.of(node_cxx['NODE']['CXX']) / d['wrapper_cxx_file'], d['node_cpp_file'])
    app(d, monitor_template)
    app(d, radlm_template)
    app(d, monitor_code_template)
    write_file(infos.ws_dir / 'monitor.radlm', d['monitor_radlm'])
    write_file(infos.ws_dir / 'monitor_topics.radl', d['monitor_topics'])
    write_file(infos.ws_dir / spec_infos['path'] / 'HealthMonitor.h', d['monitor_h_file'])
    write_file(infos.ws_dir / spec_infos['path'] / 'HealthMonitor.cpp', d['monitor_cpp_file'])
