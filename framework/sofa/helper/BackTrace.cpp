/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/helper/BackTrace.h>

#if !defined(WIN32) && !defined(_XBOX) && !defined(__APPLE__) && !defined(PS3)
#include <execinfo.h>
#include <unistd.h>
#endif
#if defined(__GNUC__) && !defined(PS3)
#include <cxxabi.h>
#endif
#if defined(WIN32)
#include <windows.h>
#include <DbgHelp.h>
#pragma comment(lib, "Dbghelp.lib")
#include <iostream>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace sofa
{

namespace helper
{

void BackTrace::dump()
{
#if defined(__GNUC__) && !defined(__APPLE__) && !defined(WIN32) && !defined(_XBOX) && !defined(PS3)

    void *array[128];
    int size = backtrace(array, sizeof(array) / sizeof(array[0]));
    if (size > 0)
    {
        char** symbols = backtrace_symbols(array, size);
        if (symbols != NULL)
        {
            for (int i = 0; i < size; ++i)
            {
                char* symbol = symbols[i];

                // Decode the method's name to a more readable form if possible
                char *beginmangled = strrchr(symbol,'(');
                if (beginmangled != NULL)
                {
                    ++beginmangled;
                    char *endmangled = strrchr(beginmangled ,')');
                    if (endmangled != NULL)
                    {
                        // remove +0x[0-9a-fA-f]* suffix
                        char* savedend = endmangled;
                        while((endmangled[-1]>='0' && endmangled[-1]<='9') ||
                                (endmangled[-1]>='a' && endmangled[-1]<='f') ||
                                (endmangled[-1]>='A' && endmangled[-1]<='F'))
                            --endmangled;
                        if (endmangled[-1]=='x' && endmangled[-2]=='0' && endmangled[-3]=='+')
                            endmangled -= 3;
                        else
                            endmangled = savedend; // suffix not found
                        char* name = (char*)malloc(endmangled-beginmangled+1);
                        memcpy(name, beginmangled, endmangled-beginmangled);
                        name[endmangled-beginmangled] = '\0';
                        int status;
                        char* realname = abi::__cxa_demangle(name, 0, 0, &status);
                        if (realname != NULL)
                        {
                            free(name);
                            name = realname;
                        }
                        fprintf(stderr,"-> %.*s%s%s\n",(int)(beginmangled-symbol),symbol,name,endmangled);
                        free(name);
                    }
                    else
                        fprintf(stderr,"-> %s\n",symbol);
                }
                else
                    fprintf(stderr,"-> %s\n",symbol);
            }
            free(symbols);
        }
        else
        {
            backtrace_symbols_fd(array, size, STDERR_FILENO);
        }
    }

#elif !defined(__GNUC__) && !defined(__APPLE__) && defined(WIN32) && !defined(_XBOX) && !defined(PS3)

    unsigned int   i;
    void           *stack[100];
    unsigned short frames;
    SYMBOL_INFO    *symbol;
    HANDLE         process;

    process = GetCurrentProcess();

    SymInitialize(process, NULL, TRUE);

    frames = CaptureStackBackTrace(0, 100, stack, NULL);
    symbol = (SYMBOL_INFO *)calloc(sizeof(SYMBOL_INFO) + (MAX_SYM_NAME-1) * sizeof(char), 1);
    symbol->MaxNameLen = MAX_SYM_NAME;
    symbol->SizeOfStruct = sizeof(SYMBOL_INFO);

    for (i = 0; i < frames; i++)
    {
        SymFromAddr(process, (DWORD64)(stack[i]), 0, symbol);
        std::cerr << (frames - i - 1) << ": " << symbol->Name << " - 0x" << std::hex << symbol->Address << std::dec << std::endl;
    }

    free(symbol);

#endif
}

#if !defined(WIN32) && !defined(_XBOX) && !defined(PS3)
bool& getStopProcess()
{
    static bool stopProcess = false;
    return stopProcess;
}
#endif

void BackTrace::autodump(bool stopProcess)
{
    // Read mode from environment variable named SOFA_DEBUG_SIGNAL_MODE
    // Effects of each mode when a signal is received by the process:
    //  - OFF : no effect
    //  - TRACE : dump a backtrace
    //  - STOP : dump a backtrace and stop the process (only supported on Linux)
    // The default mode is TRACE

    const char* mode = "TRACE";
    if (const char* modeFromEnv = std::getenv("SOFA_DEBUG_SIGNAL_MODE"))
    {
        mode = modeFromEnv;
    }

    if (std::strcmp(mode, "OFF") == 0)
    {
        return;
    }
#if !defined(WIN32) && !defined(_XBOX) && !defined(PS3)
    else if (std::strcmp(mode, "STOP") == 0)
    {
        getStopProcess() = stopProcess;
    }
#endif
    else if (std::strcmp(mode, "TRACE") != 0)
    {
        fprintf(stderr, "Invalid SOFA_DEBUG_SIGNAL_MODE environment variable: %s\n", mode);
    }

#if !defined(_XBOX) && !defined(PS3)
#if !defined(WIN32)
    struct sigaction action;
    action.sa_sigaction = BackTrace::sig;
    action.sa_flags = SA_SIGINFO;
    sigfillset(&action.sa_mask);

    sigaction(SIGSEGV, &action, NULL);
    sigaction(SIGILL, &action, NULL);
    sigaction(SIGFPE, &action, NULL);
    sigaction(SIGPIPE, &action, NULL);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGABRT, &action, NULL);
#else
    signal(SIGSEGV, BackTrace::sig);
    signal(SIGILL, BackTrace::sig);
    signal(SIGFPE, BackTrace::sig);
    signal(SIGINT, BackTrace::sig);
    signal(SIGTERM, BackTrace::sig);
    signal(SIGABRT, BackTrace::sig);
#endif
#endif   
}

#if !defined(WIN32) && !defined(_XBOX) && !defined(PS3)

void BackTrace::sig(int sig, siginfo_t *siginfo, void *)
{
    fprintf(stderr,"\n########## SIG %d (%s) ##########\n", sig, strsignal(sig));
    if (sig == SIGFPE)
    {
        printFPE(siginfo->si_code);
    }

    dump();

    // support for just-in-time debugging on Linux
#ifdef SOFA_CHECK_CONTAINER_ACCESS // for some reason NDEBUG is set when building in RelWithDebInfo, so use another macro
    if (getStopProcess() && (sig == SIGSEGV || sig == SIGFPE || sig == SIGILL)) // only pause the process when the program crashes so it is still possible to kill it via other signals
    {
        raise(SIGSTOP);
    }
#endif

    signal(sig, SIG_DFL);
    raise(sig);
}

void BackTrace::printFPE(int si_code)
{
    switch (si_code)
    {
    case FPE_INTDIV: fprintf(stderr, "Integer divide by zero"); break;
    case FPE_INTOVF: fprintf(stderr, "Integer overflow"); break;
    case FPE_FLTDIV: fprintf(stderr, "Floating-point divide by zero"); break;
    case FPE_FLTOVF: fprintf(stderr, "Floating-point overflow"); break;
    case FPE_FLTUND: fprintf(stderr, "Floating-point underflow"); break;
    case FPE_FLTRES: fprintf(stderr, "Floating-point inexact result"); break;
    case FPE_FLTINV: fprintf(stderr, "Floating-point invalid operation"); break;
    case FPE_FLTSUB: fprintf(stderr, "Subscript out of range"); break;
    }
    fprintf(stderr, "\n");
}

#elif defined(WIN32) && !defined(_XBOX) && !defined(PS3)

void BackTrace::sig(int sig)
{
    fprintf(stderr, "\n########## SIG %d ##########\n", sig);

    dump();
    signal(sig, SIG_DFL);
    raise(sig);
}

#endif

} // namespace helper

} // namespace sofa

