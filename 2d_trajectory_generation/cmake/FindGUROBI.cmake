#### Taken from http://www.openflipper.org/svnrepo/CoMISo/trunk/CoMISo/cmake/FindGUROBI.cmake

# - Try to find GUROBI
# Once done this will define
#  GUROBI_FOUND - System has Gurobi
#  GUROBI_INCLUDE_DIRS - The Gurobi include directories
#  GUROBI_LIBRARIES - The libraries needed to use Gurobi

find_path(GUROBI_INCLUDE_DIRS
    NAMES gurobi_c++.h gurobi_c.h
    HINTS "/home/ind/gurobi902/linux64/include"
    )

find_library(GUROBI_LIBRARY
    NAMES gurobi gurobi81 gurobi90 gurobi_c++.a
    HINTS "/home/ind/gurobi902/linux64/lib"
    )

if(CXX)
    if(MSVC)
        # determine Visual Studio year
        if(MSVC_TOOLSET_VERSION EQUAL 142)
            set(MSVC_YEAR "2019")
        elseif(MSVC_TOOLSET_VERSION EQUAL 141)
            set(MSVC_YEAR "2017")
        elseif(MSVC_TOOLSET_VERSION EQUAL 140)
            set(MSVC_YEAR "2015")
        endif()

        if(MT)
            set(M_FLAG "mt")
        else()
            set(M_FLAG "md")
        endif()
        
        find_library(GUROBI_CXX_LIBRARY
            NAMES gurobi_c++${M_FLAG}${MSVC_YEAR}
            HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
            PATH_SUFFIXES lib)
        find_library(GUROBI_CXX_DEBUG_LIBRARY
            NAMES gurobi_c++${M_FLAG}d${MSVC_YEAR}
            HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
            PATH_SUFFIXES lib)
    else()
        find_library(GUROBI_CXX_LIBRARY
            NAMES gurobi_c++
            HINTS "/home/ind/gurobi902/linux64/lib"
            )
        set(GUROBI_CXX_DEBUG_LIBRARY ${GUROBI_CXX_LIBRARY})
    endif()
endif()

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set GUROBI_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(GUROBI  DEFAULT_MSG
                                  GUROBI_LIBRARY GUROBI_INCLUDE_DIRS)

mark_as_advanced(GUROBI_INCLUDE_DIRS GUROBI_LIBRARY GUROBI_LIBRARIES)
