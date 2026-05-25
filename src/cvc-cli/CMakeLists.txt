cmake_minimum_required(VERSION 3.15)

# Unified CVC CLI executable
add_executable(cvc-cli main.cpp)

# Link against the libcvc library (brings in Boost::filesystem, etc.)
target_link_libraries(cvc-cli PRIVATE cvc::cvc)

# We also need Boost::program_options which is not part of the library
find_package(Boost REQUIRED COMPONENTS program_options)
target_link_libraries(cvc-cli PRIVATE Boost::program_options)

# Install the executable as "cvc"
set_target_properties(cvc-cli PROPERTIES OUTPUT_NAME cvc)
install(TARGETS cvc-cli RUNTIME DESTINATION bin)
