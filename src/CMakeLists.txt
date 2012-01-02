rock_library(qualisys
    SOURCES Driver.cpp
    HEADERS Driver.hpp
    DEPS_PKGCONFIG iodrivers_base tinyxml)

rock_executable(qualisys_bin Main.cpp
    DEPS qualisys)
