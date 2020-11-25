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
#ifndef SOFA_HELPER_PREPROCESSOR_H
#define SOFA_HELPER_PREPROCESSOR_H

#define SOFA_REQUIRE_SEMICOLON_CONCAT2(a,b) a ## b
#define SOFA_REQUIRE_SEMICOLON_CONCAT(a,b) SOFA_REQUIRE_SEMICOLON_CONCAT2(a,b)
#define SOFA_REQUIRE_SEMICOLON SOFA_REQUIRE_SEMICOLON_CONCAT(struct dummy, __LINE__)

#define SOFA_EMPTY
#define SOFA_EMPTY_DELIMITER (SOFA_EMPTY)
#define SOFA_FOR_EACH(MACRO, Delimiter, ...) SOFA_MACRO_CALL(SOFA_APPLY(SOFA_FOR_EACH_START, SOFA_NUMBER_ARGS(__VA_ARGS__))(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_START(NB_ARGS) SOFA_FOR_EACH_##NB_ARGS

#define SOFA_APPLY(MACRO, ...) SOFA_MACRO_CALL(MACRO(__VA_ARGS__))
#define SOFA_MACRO_CALL(x) x
#define SOFA_BUNDLE(...) (__VA_ARGS__)
#define SOFA_UNBUNDLE(...) __VA_ARGS__

#define SOFA_FOR_EACH_0(MACRO, Delimiter, Arg)
#define SOFA_FOR_EACH_1(MACRO, Delimiter, Arg)      MACRO(Arg)
#define SOFA_FOR_EACH_2(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_1(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_3(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_2(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_4(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_3(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_5(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_4(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_6(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_5(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_7(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_6(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_8(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_7(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_9(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_8(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_10(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_9(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_11(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_10(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_12(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_11(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_13(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_12(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_14(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_13(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_15(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_14(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_16(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_15(MACRO, Delimiter, __VA_ARGS__))

#define SOFA_FOR_EACH_17(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_16(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_18(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_17(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_19(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_18(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_20(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_19(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_21(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_20(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_22(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_21(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_23(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_22(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_24(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_23(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_25(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_24(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_26(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_25(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_27(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_26(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_28(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_27(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_29(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_28(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_30(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_29(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_31(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_30(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_32(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_31(MACRO, Delimiter, __VA_ARGS__))

#define SOFA_FOR_EACH_33(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_32(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_34(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_33(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_35(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_34(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_36(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_35(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_37(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_36(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_38(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_37(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_39(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_38(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_40(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_39(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_41(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_40(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_42(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_41(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_43(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_42(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_44(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_43(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_45(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_44(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_46(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_45(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_47(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_46(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_48(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_47(MACRO, Delimiter, __VA_ARGS__))

#define SOFA_FOR_EACH_49(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_48(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_50(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_49(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_51(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_50(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_52(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_51(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_53(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_52(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_54(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_53(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_55(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_54(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_56(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_55(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_57(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_56(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_58(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_57(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_59(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_58(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_60(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_59(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_61(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_60(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_62(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_61(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_63(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_62(MACRO, Delimiter, __VA_ARGS__))
#define SOFA_FOR_EACH_64(MACRO, Delimiter, Arg, ...) MACRO(Arg) SOFA_MACRO_CALL(SOFA_UNBUNDLE Delimiter) SOFA_MACRO_CALL(SOFA_FOR_EACH_63(MACRO, Delimiter, __VA_ARGS__))


#define SOFA_REPLACE_PARENTHESIS_TO_COUNT_0(...) X0,X64,X63,X62,X61,X60,X59,X58,X57,X56,X55,X54,X53,X52,X51,X50,X49,X48,X47,X46,X45,X44,X43,X42,X41,X40,X39,X38,X37,X36,X35,X34,X33,X32,X31,X30,X29,X28,X27,X26,X25,X24,X23,X22,X21,X20,X19,X18,X17,X16,X15,X14,X13,X12,X11,X10,X9,X8,X7,X6,X5,X4,X3,X2,X1

// SOFA_NUMBER_ARGS(...) evaluates to the literal number of the passed-in arguments.
#define SOFA__NUMBER_ARGS_RESULT(X0, X64,X63,X62,X61,X60,X59,X58,X57,X56,X55,X54,X53,X52,X51,X50,X49,X48,X47,X46,X45,X44,X43,X42,X41,X40,X39,X38,X37,X36,X35,X34,X33,X32,X31,X30,X29,X28,X27,X26,X25,X24,X23,X22,X21,X20,X19,X18,X17,X16,X15,X14,X13,X12,X11,X10,X9,X8,X7,X6,X5,X4,X3,X2,X1,N,...) N
#define SOFA__NUMBER_ARGS(...) SOFA_MACRO_CALL(SOFA__NUMBER_ARGS_RESULT(__VA_ARGS__ ,0, 64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0))

#define SOFA_NUMBER_ARGS(...) SOFA__NUMBER_ARGS(SOFA_REPLACE_PARENTHESIS_TO_COUNT_0 __VA_ARGS__())

#define SOFA_TO_STRING_1(A) #A
#define SOFA_TO_STRING(...) SOFA_FOR_EACH(SOFA_TO_STRING_1, (,) , __VA_ARGS__)
 

#endif
