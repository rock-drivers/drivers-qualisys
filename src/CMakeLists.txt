rock_library(qualisys
    SOURCES Driver.cpp
    HEADERS Driver.hpp
    DEPS_PKGCONFIG iodrivers_base)

rock_executable(qualisys_bin Main.cpp
    DEPS qualisys)

#add_subdirectory( sdk )
