cmake_minimum_required(VERSION 3.15)

include(cmake/HunterGate.cmake)

HunterGate(
        URL "https://github.com/cpp-pm/hunter/archive/v0.23.248.tar.gz"
        SHA1 "a637ec52a1b82beb5f55ab4aa75fac30bcee8904"
)

project(DynamicBufferV2)

hunter_add_package(Boost COMPONENTS system)
hunter_add_package(OpenSSL)

find_package(Boost CONFIG COMPONENTS system)
find_package(OpenSSL)
find_package(Threads)

file(GLOB_RECURSE src_files CONFIGURE_DEPENDS "src/*.cpp" "src/*.hpp")
add_executable(check ${src_files})
target_include_directories(check PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(check PRIVATE Boost::system OpenSSL::Crypto OpenSSL::SSL Threads::Threads)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(check PRIVATE -Werror -Wall -Wextra -pedantic)
endif()