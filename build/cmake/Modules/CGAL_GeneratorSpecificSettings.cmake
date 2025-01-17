if ( NOT CGAL_GENERATOR_SPECIFIC_SETTINGS_FILE_INCLUDED )
    set( CGAL_GENERATOR_SPECIFIC_SETTINGS_FILE_INCLUDED 1 )

    message( STATUS "CGAL settings: targetting ${CMAKE_GENERATOR}")

    if ( MSVC )
        message( STATUS "CGAL settings: target build enviroment supports auto-linking" )
        set(CGAL_AUTO_LINK_ENABLED TRUE)
    endif()

    if ( MSVC11 )
        set(CGAL_TOOLSET "vc110")
        message( STATUS "CGAL settings: using VC11 compiler." )
    elseif ( MSVC10 )
        set(CGAL_TOOLSET "vc100")
        message( STATUS "CGAL settings: using VC10 compiler." )
    elseif ( MSVC90 )
        set(CGAL_TOOLSET "vc90")
        message( STATUS "CGAL settings: using VC90 compiler." )
    elseif ( MSVC80 )
        set(CGAL_TOOLSET "vc80")
        message( STATUS "CGAL settings: using VC80 compiler." )
    elseif ( MSVC71 )
        set(CGAL_TOOLSET "vc71")
        message( STATUS "CGAL settings: using VC71 compiler." )
    else()
        message( STATUS "CGAL settings: using ${CMAKE_CXX_COMPILER} compiler." )
    endif()


    # From james Bigler, in the cmake users list.
    IF (APPLE)
        exec_program(uname ARGS -v  OUTPUT_VARIABLE DARWIN_VERSION)
        string(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
        message(STATUS "CGAL settings: DARWIN_VERSION=${DARWIN_VERSION}")
        if (DARWIN_VERSION GREATER 8)
            message(STATUS "CGAL settings: Mac Leopard detected")
            set(CGAL_APPLE_LEOPARD 1)
        endif()
    endif()

    if ( NOT "${CMAKE_CFG_INTDIR}" STREQUAL "." )
        set(HAS_CFG_INTDIR TRUE CACHE INTERNAL "CGAL settings: generator uses intermediate configuration directory" )
        message( STATUS "CGAL settings: generator uses intermediate configuration directory: ${CMAKE_CFG_INTDIR}" )
    endif()

endif()
