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


TEST(map_ptr_stable_compare, checkMapPtrStableCompare)
{
    typedef sofa::helper::map_ptr_stable_compare<int*, int> IntegerMap;
    typedef sofa::helper::ptr_stable_id< int > IntegerStableId;

    int a = 0;
    int b = 1;
    int c = 2;
    int d = 3;

    IntegerMap map;

    map[&b] = b;
    map[&d] = d;
    map[&a] = a;
    map[&c] = c;

    IntegerStableId* idMap = map.key_comp().get_stable_id_map();

    EXPECT_EQ(idMap->id(&b), 0);
    EXPECT_EQ(idMap->id(&d), 1);
    EXPECT_EQ(idMap->id(&a), 2);
    EXPECT_EQ(idMap->id(&c), 3);

    typename IntegerMap::const_iterator it = map.begin();

    EXPECT_EQ(it->second, b);
    ++it;
    EXPECT_EQ(it->second, d);
    ++it;
    EXPECT_EQ(it->second, a);
    ++it;
    EXPECT_EQ(it->second, c);
}


TEST(map_ptr_stable_compare, checkMapPairPtrStableCompare)
{
    typedef sofa::helper::map_ptr_stable_compare<std::pair<int*,int*>, int> IntegerPairMap;
    typedef sofa::helper::ptr_stable_id< int > IntegerStableId;
    int a = 1;
    int b = 2;
    int c = 4;
    int d = 8;

    IntegerPairMap map;

    map[std::make_pair(&a, &a)] = a;
    map[std::make_pair(&a, &b)] = a+b;
    map[std::make_pair(&b, &b)] = b;
    map[std::make_pair(&b, &c)] = b+c;
    map[std::make_pair(&c, &c)] = c;
    map[std::make_pair(&c, &d)] = c+d;
    map[std::make_pair(&d, &d)] = d;

    IntegerStableId* idMap = map.key_comp().get_stable_id_map();

    EXPECT_EQ(idMap->id(&a), 0);
    EXPECT_EQ(idMap->id(&b), 1);
    EXPECT_EQ(idMap->id(&c), 2);
    EXPECT_EQ(idMap->id(&d), 3);

    typename IntegerPairMap::const_iterator it = map.begin();

    EXPECT_EQ(it->second, a);
    ++it;
    EXPECT_EQ(it->second, a+b);
    ++it;
    EXPECT_EQ(it->second, b);
    ++it;
    EXPECT_EQ(it->second, b+c);
    ++it;
    EXPECT_EQ(it->second, c);
    ++it;
    EXPECT_EQ(it->second, c+d);
    ++it;
    EXPECT_EQ(it->second, d);
}


TEST(map_ptr_stable_compare, checkMapPairPtrAssignementOperator)
{
    typedef sofa::helper::map_ptr_stable_compare < std::pair<int*, int*>, int > IntegerPairMap;

    int a = 0;
    int b = 1;
    int c = 2;
    int d = 3;

    IntegerPairMap map;
    map[std::make_pair(&a, &a)] = a;
    map[std::make_pair(&a, &b)] = a + b;
    map[std::make_pair(&b, &b)] = b;
    map[std::make_pair(&b, &c)] = b + c;
    map[std::make_pair(&c, &c)] = c;
    map[std::make_pair(&c, &d)] = c + d;
    map[std::make_pair(&d, &d)] = d;

    IntegerPairMap mapCopy = map;

    ASSERT_EQ(mapCopy.size(), map.size());

    for (typename IntegerPairMap::const_iterator itCopy = mapCopy.begin(), it = map.begin(); it != map.end();
         ++itCopy, ++it)
    {
        EXPECT_EQ(it->first, itCopy->first);
        EXPECT_EQ(it->second, itCopy->second);
    }
}

TEST(map_ptr_stable_compare, checkMapPtrAssignementOperator)
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

TEST(map_ptr_stable_compare, CheckIdMapPersistsAfterClear)
{
    typedef sofa::helper::map_ptr_stable_compare<int*, int> IntegerMap;
    typedef sofa::helper::ptr_stable_id< int > IntegerStableId;

    int a = 0;
    int b = 1;
    int c = 2;
    int d = 3;

    IntegerMap map;

    map[&a] = a;
    map[&b] = b;
    map[&c] = c;
    map[&d] = d;

    IntegerStableId* idMap = map.key_comp().get_stable_id_map();

    EXPECT_EQ(map.size(), 4);

    map.clear();

    EXPECT_EQ(map.size(), 0);
    EXPECT_EQ(idMap->size(), 4);
}






}