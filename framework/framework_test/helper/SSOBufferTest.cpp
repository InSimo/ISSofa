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
#include <gtest/gtest.h>

#include <sofa/helper/SSOBuffer.h>
#include <vector>
#include <array>

namespace
{

using namespace sofa::helper;

unsigned int DefaultConstructionNumber = 0u;
unsigned int MoveConstructionNumber = 0u;
unsigned int CopyConstructionNumber = 0u;
unsigned int DestructionNumber = 0u;

template <class T>
class ConstrDestrCounter
{
public:
    ConstrDestrCounter() : data()
    {
        DefaultConstructionNumber++;
    }

    ConstrDestrCounter(ConstrDestrCounter&& rhs) : data(std::move(rhs.get()))
    {
        MoveConstructionNumber++;
    }

    ConstrDestrCounter(const ConstrDestrCounter& rhs) : data(rhs.get())
    {
        CopyConstructionNumber++;
    }

    ~ConstrDestrCounter()
    {
        DestructionNumber++;
    }

    ConstrDestrCounter& operator=(ConstrDestrCounter&& rhs) = default;
    ConstrDestrCounter& operator=(ConstrDestrCounter& rhs) = default;

    ConstrDestrCounter& operator=(T&& rhs)
    {
        data = std::move(rhs);
        return *this;
    }

    ConstrDestrCounter& operator=(T& rhs)
    {
        data = rhs;
        return *this;
    }

    T const & get() const
    {
        return data;
    }

    T& get()
    {
        return data;
    }
protected:
    T data;
};

void ResetCounter()
{
    DefaultConstructionNumber = 0u;
    MoveConstructionNumber = 0u;
    CopyConstructionNumber = 0u;
    DestructionNumber = 0u;
}

TEST(SSOBufferTest, smallBufferTest)
{
    ResetCounter();
    SSOBuffer<16> buffer_first;

    std::vector<double> vecDouble{ 1.0, 2.5, 5.1, 3.0 };
    using VecConstIterator = ConstrDestrCounter<std::vector<double>::const_iterator>;

    {
        // Creates iterator in small buffer
        VecConstIterator& it_begin = *buffer_first.getOrCreate<VecConstIterator>();
        it_begin = vecDouble.cbegin();

        ASSERT_TRUE(16 > sizeof(VecConstIterator));
        EXPECT_EQ(1.0, *it_begin.get());
    }

    {
        // Destructs first iterator and creates another one in small buffer
        {
            VecConstIterator& it_end = *buffer_first.create<VecConstIterator>();
            it_end = vecDouble.cend();
        }

        SSOBuffer<16> buffer_second(buffer_first); // Test copy construction
        SSOBuffer<16> buffer_third(std::move(buffer_second)); // Test move construction

        auto& it = buffer_third.getOrCreate<VecConstIterator>()->get();
        EXPECT_EQ(4, std::distance(vecDouble.cbegin(), it));
    }
    EXPECT_EQ(2, DefaultConstructionNumber);
    EXPECT_EQ(1, MoveConstructionNumber);
    EXPECT_EQ(1, CopyConstructionNumber);
    EXPECT_EQ(3, DestructionNumber);
}

TEST(SSOBufferTest, largeBufferTest)
{
    ResetCounter();
    SSOBuffer<16> buffer_first;

    using Array = ConstrDestrCounter<std::array<double, 5>>;

    {
        // Creates iterator in large buffer
        Array& doubles = *buffer_first.getOrCreate<Array>();
        doubles = std::array<double, 5>({ 1.0, 2.5, 5.1, 3.0, 1.0 });

        ASSERT_TRUE(16 < sizeof(Array));
        EXPECT_EQ(2.5, doubles.get()[1]);
    }

    {
        // Destructs first array and creates another one in large buffer
        {
            Array& doubles = *buffer_first.create<Array>();
            doubles = std::array<double, 5>({ 0.0, 1.1, 1.1, 1.5, 1.0 });
            EXPECT_EQ(1.1, doubles.get()[1]);
        }

        SSOBuffer<16> buffer_second(buffer_first); // Test copy construction
        SSOBuffer<16> buffer_third(std::move(buffer_second)); // Test move construction

        auto& doubles = buffer_third.getOrCreate<Array>()->get(); 
        EXPECT_EQ(1.1, doubles[1]);
    }
    EXPECT_EQ(2, DefaultConstructionNumber);
    EXPECT_EQ(0, MoveConstructionNumber);
    EXPECT_EQ(1, CopyConstructionNumber);
    EXPECT_EQ(2, DestructionNumber);
}

TEST(SSOBufferTest, smallAndLargeMixedTest)
{
    ResetCounter();
    SSOBuffer<16> buffer_first;

    using VecConstIterator = ConstrDestrCounter<std::vector<double>::const_iterator>;
    using Array = ConstrDestrCounter<std::array<double, 5>>;

    std::vector<double> vecDouble{ 1.0, 2.5, 5.1, 3.0 };

    {
        // Creates iterator in large buffer
        Array& doubles = *buffer_first.getOrCreate<Array>();
        doubles = std::array<double, 5>({ 5.0, 2.5, 5.1, 3.0, 1.0 });
    }

    {
        // Destructs object in large buffer and creates a small buffer
        VecConstIterator& doubles = *buffer_first.create<VecConstIterator>();
        doubles = vecDouble.cbegin();
        EXPECT_EQ(1.0, *doubles.get());
    }
    EXPECT_EQ(2, DefaultConstructionNumber);
    EXPECT_EQ(0, MoveConstructionNumber);
    EXPECT_EQ(0, CopyConstructionNumber);
    EXPECT_EQ(1, DestructionNumber);
}

}