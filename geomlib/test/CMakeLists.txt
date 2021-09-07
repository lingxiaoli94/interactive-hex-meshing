add_executable(test main.cpp)

target_compile_options(test PRIVATE ${cxx_warning_flags})
target_link_libraries(test geomlib spdlog)
