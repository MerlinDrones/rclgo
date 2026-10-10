# Values in spellings rosidl accepts and Go does not: booleans as True and 1,
# integers that look octal (010 is 10) or have a base prefix.
bool C_TRUE = True
bool C_ONE = 1
int32 C_DEC = 010
int32 C_HEX = 0x10

bool b_true True
bool b_zero 0
int32 i_dec 010
uint8 u_hex 0xff
int64 i_bin -0b101
bool[2] bs [True, 0]
int16[] ints [010, 0x10]
