/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace sofa
{

namespace helper
{

/// Dump current backtrace to stderr.
/// Currently only works on Linux. NOOP on other architectures.
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
#endif
}

/// Enable dump of backtrace when a signal is received.
/// Useful to have information about crashes without starting a debugger (as it is not always easy to do, i.e. for parallel/distributed applications).
/// Currently only works on Linux. NOOP on other architectures
void BackTrace::autodump()
{
#if !defined(WIN32) && !defined(_XBOX) && !defined(PS3)
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
#endif

} // namespace helper

} // namespace sofa

