
'''
Created on March, 2015

'''

version = 'RADLM 0.987'

extra_keywords = {
#C++ keywords
'alignas', 'alignof', 'and', 'and_eq', 'asm', 'auto', 'bitand', 'bitor',
'bool', 'break', 'case', 'catch', 'char', 'char16_t', 'char32_t', 'class',
'compl', 'const', 'constexpr', 'const_cast', 'continue', 'decltype', 'default',
'delete', 'do', 'double', 'dynamic_cast', 'else', 'enum', 'explicit', 'export',
'extern', 'false', 'float', 'for', 'friend', 'goto', 'if', 'inline', 'int',
'long', 'mutable', 'namespace', 'new', 'noexcept', 'not', 'not_eq', 'nullptr',
'operator', 'or', 'or_eq', 'private', 'protected', 'public', 'register',
'reinterpret_cast', 'return', 'short', 'signed', 'sizeof', 'static',
'static_assert', 'static_cast', 'struct', 'switch', 'template', 'this',
'thread_local', 'throw', 'true', 'try', 'typedef', 'typeid', 'typename',
'union', 'unsigned', 'using', 'virtual', 'void', 'volatile', 'wchar_t',
'while', 'xor', 'xor_eq',
#Usual macro definitions non commencing with __
'linux', 'unix'
}

forbidden_prefix = "radl"

defs = r"""
type int8
    REGEX ~r"(?P<value>(\b|[+-])\d+)\b(?!\.)"

type uint8
    REGEX ~r"\b(?P<value>\d+)\b(?!\.)"

type int16
    REGEX ~r"(?P<value>(\b|[+-])\d+)\b(?!\.)"

type uint16
    REGEX ~r"\b(?P<value>\d+)\b(?!\.)"

type int32
    REGEX ~r"(?P<value>(\b|[+-])\d+)\b(?!\.)"

type uint32
    REGEX ~r"\b(?P<value>\d+)\b(?!\.)"

type int64
    REGEX ~r"(?P<value>(\b|[+-])\d+)\b(?!\.)"

type uint64
    REGEX ~r"\b(?P<value>\d+)\b(?!\.)"

type float32
    REGEX ~r"(?P<value>(\b|[+-])(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?)\b(?!\.)"

type float64
    REGEX ~r"(?P<value>(\b|[+-])(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?)\b(?!\.)"

type bool
    REGEX ~r"\b(?P<value>true|false)"

type string
    REGEX ~r'"(?P<value>[^"]*)"'

type duration
    REGEX ~r"(?P<value>(\b|[+-])\d+)(?P<unit>sec|msec|usec|nsec)"

type time
    REGEX ~r"(?P<value>\b\d+)(?P<unit>sec|msec|usec|nsec)"

type ip
    REGEX ~r"\b(?P<value>\d\d?\d?\.\d\d?\d?\.\d\d?\d?\.\d\d?\d?)\b"


class module_settings
    MODULE_BASE_PATH string ?
    PATH string ?
    HEADER string ?
    FILENAME string *
    LIB cmake_library/static_library *
    POST_INIT_HOOK string ?

class cxx_class
    PATH string ?
    HEADER string
    FILENAME string *
    LIB cmake_library/static_library *
    CLASS string
    STEP_METHOD string ?

class cxx_file
    PATH string ?
    FILENAME string *
    LIB cmake_library/static_library *


class c_class
    PATH string ?
    HEADER string
    FILENAME string *
    LIB cmake_library/static_library *
    TYPENAME string ?        #defaults to 'state'
    INIT_FUNCTION string ?   #defaults to the radl defined RADL_INIT_FUN macro
    STEP_FUNCTION string ?   #defaults to the radl defined RADL_STEP_FUN macro
    FINISH_FUNCTION string ? #defaults to the radl defined RADL_FINISH_FUN macro

class cmake_library
    PATH string ?
    CMAKE_MODULE string
    CMAKE_COMPONENTS string *
    CMAKE_VAR_LIBRARIES string ?     #defaults to {CMAKE_MODULE}_LIBRARIES
    CMAKE_VAR_INCLUDE_DIRS string ?  #defaults to {CMAKE_MODULE}_INCLUDE_DIRS

class static_library
    PATH string ?
    HEADER_PATHS string *
    CXX cxx_file *

class external_rosdef
    PATH string ?
    FULLNAME string
    HEADER string ?

class struct
    FIELDS int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/
           bool/struct/array/duration/time +
    EXTERNAL_ROS_DEF external_rosdef ?
    DEFS int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/bool/struct/array/duration/time/string *

class array
    TYPE string ?
    SIZE uint64 ?
    VALUES int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/
           bool/struct/array/duration/time*

class topic
    #Pay attention to the order of the types, parsing higher priority first
    FIELDS int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/
           bool/struct/array/duration/time +
    EXTERNAL_ROS_DEF external_rosdef ?
    DEFS int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/bool/struct/array/duration/time/string *

class publication
    TOPIC topic

class subscription
    TOPIC topic
    MAXLATENCY duration

class node
    PATH string ?
    PUBLISHES publication *
    SUBSCRIBES subscription *
    CXX cxx_class ?
    C c_class ?
    PERIOD duration
    WCET duration ?
    DEVICES device_interface *
    DEFS int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/bool/struct/array/duration/time/string *

class interceptor
    NODE node
    PUBLISHES publication *
    SUBSCRIBES subscription *
    CXX cxx_class ?

class device_interface
    HEADER string ?
    CXX cxx_file *
    DEFS int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/bool/struct/array/duration/time/string *

################
# Physical description
################

class processor
    TYPE string
    BITS int8
    ENDIANESS string

class device
    IMPLEMENTS device_interface
    REQUIRES_LINUX bool
    CXX cxx_file *

class bus
    ENDPOINTS processor/device *


class disk_image
    IMG string


################
# Mapping
################

class plant
    MACHINES machine *

##
# Machine level

class machine
    OS linux/lynxsecure/certikos

##
# Hypervisor level

class lynxsecure
    VMS lynxsecure_vm *

class certikos
    VMS certikos_vm *


##
# Virtual machine level

class lynxsecure_vm
    OS linux

class certikos_vm
    OS linux

##
# System level

class linux
    NODES_UID uint16
    IP ip ?
    IMG string
    NODES node *

##
# Implant

class implant
    LOCATION linux
    NODES node *

"""

