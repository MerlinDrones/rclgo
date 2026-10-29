# Constants of every primitive type, including the extremes.
bool C_BOOL=true
byte C_BYTE=255
char C_CHAR=65
float32 C_F32=1.5
float64 C_F64=-2.25
int8 C_I8=-128
uint8 C_U8=255
int16 C_I16=-32768
uint16 C_U16=65535
int32 C_I32=-2147483648
uint32 C_U32=4294967295
int64 C_I64=-9223372036854775808
uint64 C_U64=18446744073709551615
string C_STR="string, with comma"

# Field names that are Go keywords, or that differ only in underscores.
int32 type
int32 range
int32 func
int32 x_y_z
int32 xyz
string url2

# Defaults.
int32 with_default 42
string str_default "hello, world"
float64[3] array_default [1.5, 2.5, 3.5]
string[] strs_default ["a", "b,c"]

# Bounded types.
string<=5 bounded_str
int32[<=3] bounded_seq
string<=4[<=2] bounded_strs

# Nested, local and from other packages.
Inner inner
Inner[2] inners
Inner[] inner_seq
geometry_msgs/Point point
geometry_msgs/Point[<=2] points
builtin_interfaces/Time stamp
wstring wide
