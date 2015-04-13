'''
Created on April, 2015

  Parse the spec file and generate radl/radlm files, step functions and etc.

'''

import re
from pathlib import Path

from transformer.radlm import infos
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
{node_def}'''
}

interceptor_template = {
'interceptor_def':
'''{node}_interceptor : interceptor {{
  NODE {node_name}
  PUBLISHES
    {node}_report {{ TOPIC {mtopic_name} }}
  CXX {{
    HEADER "{header}"
    CLASS "{class}"
    FILENAME "{filename}"
  }}
}}

'''
}

node_template = {
'node_def':
'''health_monitor : node {{
  SUBSCRIBES
{node_subscription}
  PERIOD {node_period}
  CXX {{
    PATH "{path}"
    HEADER "HealthMonitor.h"
    CLASS "HealthMonitor"
    FILENAME "HealthMonitor.cpp"
  }}
}}
'''
}

node_subscription_template = {
'node_subscription':
'''    {node}_report {{ TOPIC {mtopic_name} MAXLATENCY {maxlatency} }}
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

}


def app(d, templates):
    for (s,t) in templates.items():
        v = t.format(**d)
    if s not in d or not d[s]: 
        d[s] = v
    else: 
        d[s] += v

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
    d = {'path'       : spec_infos['path'],
         'maxlatency' : spec_infos['maxlatency'],
         'node_period': spec_infos['period']}
           
    nodes = spec_infos['nodes'].split(' ')
    for n in nodes:
        nn = n.replace('.','_')
        nn_class = nn.title().replace('_','')
        d['node'] = nn
        d['node_name'] = n
        d['mtopic'] = nn + '_health'
        d['mtopic_name'] = "monitor_topics." + nn + '_health'
        d['header'] = nn_class + '.h'
        d['class'] = nn_class
        d['filename'] = nn_class + '.cpp'
        app(d, interceptor_template)
        app(d, node_subscription_template)
        app(d, topics_template)
    app(d, node_template)
    app(d, radlm_template)
    app(d, monitor_code_template)
    write_file(infos.ws_dir / 'monitor.radlm', d['monitor_radlm'])
    write_file(infos.ws_dir / 'monitor_topics.radl', d['monitor_topics'])
    write_file(infos.ws_dir / spec_infos['path'] / 'HealthMonitor.h', d['monitor_h_file'])
