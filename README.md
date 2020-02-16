# dynamic_buffer_v2

## Intention

Proof of concept implementing DynamicBuffer_v2
for containers of containers of bytes

## Required toolchain

This program requires a c++17 or better toolchain.

Some very good toolchains files available here:

https://github.com/ruslo/polly

in which case you can invoke cmake with:

`cmake -DCMAKE_TOOLCHAIN_FILE=<POLLY_DIR>/cxx17.cmake -H<SRC_DIR> -B<BUILD_DIR>`

Where:

* `POLLY_DIR` is the cloned polly repo
* `SRC_DIR` is the directory containing this file
* `BUILD_DIR` is the intended build directory (in-source builds are evil)

