/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_HELPER_PREPROCESSOR_H
#define SOFA_HELPER_PREPROCESSOR_H

#define SOFA_FOR_EACH(MACRO, ...) SOFA_MACRO_CALL(SOFA_APPLY(SOFA_FOR_EACH_START, SOFA_NUM_ARGS(__VA_ARGS__))(MACRO, __VA_ARGS__))
#define SOFA_FOR_EACH_START(NB_ARGS) SOFA_FOR_EACH_##NB_ARGS

#define SOFA_APPLY(MACRO, ...) SOFA_MACRO_CALL(MACRO(__VA_ARGS__))
#define SOFA_MACRO_CALL(x) x

#define SOFA_FOR_EACH_1(MACRO, Arg)      MACRO(Arg)
#define SOFA_FOR_EACH_2(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_1(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_3(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_2(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_4(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_3(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_5(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_4(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_6(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_5(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_7(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_6(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_8(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_7(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_9(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_8(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_10(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_9(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_11(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_10(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_12(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_11(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_13(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_12(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_14(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_13(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_15(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_14(MACRO, __VA_ARGS__)
#define SOFA_FOR_EACH_16(MACRO, Arg, ...) MACRO(Arg) SOFA_FOR_EACH_15(MACRO, __VA_ARGS__)

// SOFA_NUM_ARGS(...) evaluates to the literal number of the passed-in arguments.
#define SOFA_NUM_ARGS_RESULT(X64,X63,X62,X61,X60,X59,X58,X57,X56,X55,X54,X53,X52,X51,X50,X49,X48,X47,X46,X45,X44,X43,X42,X41,X40,X39,X38,X37,X36,X35,X34,X33,X32,X31,X30,X29,X28,X27,X26,X25,X24,X23,X22,X21,X20,X19,X18,X17,X16,X15,X14,X13,X12,X11,X10,X9,X8,X7,X6,X5,X4,X3,X2,X1,N,...) N
#define SOFA_NUM_ARGS(...) SOFA_MACRO_CALL(SOFA_NUM_ARGS_RESULT(__VA_ARGS__ ,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0))

#define SOFA_TO_STRING(A) #A

#endif
