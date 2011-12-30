rock_library(qualisys
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(qualisys_bin Main.cpp
    DEPS qualisys)

