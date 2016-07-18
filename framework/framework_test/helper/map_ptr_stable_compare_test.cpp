#include <gtest/gtest.h>
#include <sofa/helper/map_ptr_stable_compare.h>


namespace
{
TEST(ptr_stable_id, CheckPtrStableId)
{
    typedef sofa::helper::ptr_stable_id< int > IntPtrStableId;

    int a = 70;
    int b = 23;
    int c = 2;
    int d = 10;

    IntPtrStableId stableId;

    unsigned int id_a = stableId(&a);
    unsigned int id_b = stableId(&b);
    unsigned int id_c = stableId(&c);
    unsigned int id_d = stableId(&d);


    EXPECT_EQ(0, id_a);
    EXPECT_EQ(1, id_b);
    EXPECT_EQ(2, id_c);
    EXPECT_EQ(3, id_d);
}


TEST(map_ptr_stable_compare, checkPtrStableCompare)
{
    typedef sofa::helper::map_ptr_stable_compare<int*, int> IntegerMap;

    int a = 0;
    int b = 1;
    int c = 2;
    int d = 3;

    IntegerMap map;

    map[&a] = a;
    map[&b] = b;
    map[&c] = c;
    map[&d] = d;

    typename IntegerMap::const_iterator it = map.begin();

    EXPECT_EQ(it->second, a);
    ++it;
    EXPECT_EQ(it->second, b);
    ++it;
    EXPECT_EQ(it->second, c);
    ++it;
    EXPECT_EQ(it->second, d);
}


TEST(map_ptr_stable_compare, checkAssignementOperator)
{
    typedef sofa::helper::map_ptr_stable_compare<int*, int> IntegerMap;

    int a = 0;
    int b = 1;
    int c = 2;
    int d = 3;

    IntegerMap map;

    map[&a] = a;
    map[&b] = b;
    map[&c] = c;
    map[&d] = d;

    IntegerMap mapCopy = map;

    ASSERT_EQ(mapCopy.size(), map.size());

    for (typename IntegerMap::const_iterator itCopy = mapCopy.begin(), it = map.begin(); it != map.end(); 
         ++itCopy, ++it)
    {
        EXPECT_EQ(it->first, itCopy->first);
        EXPECT_EQ(it->second, itCopy->second);
    }
}






}
