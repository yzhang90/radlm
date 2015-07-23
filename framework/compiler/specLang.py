defs = r"""
_spec = _ monitor_section

monitor_section = "[monitor]:" _ monitor_def*

monitor_def = node_keyword _ "=" _ value

node_keyword = ~r"(PATH|PERIOD)"

value = ~r"[^!]*"

_ = ~r"\s*(#[^\r\n]*\s*)*\s*"

"""
