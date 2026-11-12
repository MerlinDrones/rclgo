/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

/*
These types implement the rosidl_runtime_c primitive types handling semantics.

The *IntoC functions (StringIntoC, X__Sequence_into_C, X__Array_into_C)
convert into C memory that holds a valid value: memory initialized by the
rosidl __init/__create functions, or zeroed. They free what the memory holds
before writing, so that nothing leaks. They replaced the AsCStruct/_to_C
functions, which wrote over the memory without freeing it; bindings generated
for those must be regenerated (they would pass uninitialized memory).
*/
package primitives
