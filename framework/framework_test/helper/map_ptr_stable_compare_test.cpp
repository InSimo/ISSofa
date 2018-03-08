#include <gtest/gtest.h>
#include <sofa/helper/map_ptr_stable_compare.h>


namespace
{
TEST(ptr_stable_id, CheckPtrStableId)
{
    typedef sofa::helper::ptr_stable_id< int* > IntPtrStableId;

    int a = 70;
    int b = 23;
    int c = 2;
    int d = 10;

    IntPtrStableId stableId;

    unsigned int id_a = stableId(&a);
    unsigned int id_b = stableId(&b);
    unsigned int id_c = stableId(&c);
    unsigned int id_d = stableId(&d);


    EXPECT_EQ(0u, id_a);
    EXPECT_EQ(1u, id_b);
    EXPECT_EQ(2u, id_c);
    EXPECT_EQ(3u, id_d);
}


TEST(map_ptr_stable_compare, checkMapPtrStableCompareOrdering)
{
    typedef sofa::helper::map_ptr_stable_compare<int*, int> IntegerMap;
    typedef sofa::helper::ptr_stable_id< int* > IntegerStableId;

    int a = 0, b = 1, c = 2, d = 3;

    IntegerMap map;

    map[&b] = b;
    map[&d] = d;
    map[&a] = a;
    map[&c] = c;

    IntegerStableId* idMap = map.key_comp().get_stable_id_map();

    EXPECT_EQ(idMap->id(&b), 0u);
    EXPECT_EQ(idMap->id(&d), 1u);
    EXPECT_EQ(idMap->id(&a), 2u);
    EXPECT_EQ(idMap->id(&c), 3u);

    typename IntegerMap::const_iterator it = map.begin();

    EXPECT_EQ(it->second, b);
    ++it;
    EXPECT_EQ(it->second, d);
    ++it;
    EXPECT_EQ(it->second, a);
    ++it;
    EXPECT_EQ(it->second, c);
}


TEST(map_ptr_stable_compare, checkMapPairPtrStableCompareOrdering)
{
    typedef sofa::helper::map_ptr_stable_compare<std::pair<int*,int*>, int> IntegerPairMap;
    typedef sofa::helper::ptr_stable_id< std::pair<int*, int*> > IntegerPairStableId;
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

    IntegerPairStableId* idMap = map.key_comp().get_stable_id_map();

    EXPECT_EQ(idMap->id(std::make_pair(&a, &a)), 0u);
    EXPECT_EQ(idMap->id(std::make_pair(&a, &b)), 1u);
    EXPECT_EQ(idMap->id(std::make_pair(&b, &b)), 2u);
    EXPECT_EQ(idMap->id(std::make_pair(&b, &c)), 3u);

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


TEST(map_ptr_stable_compare, checkMapPairPtrCopyAssignementOperator)
{
    typedef sofa::helper::map_ptr_stable_compare < std::pair<int*, int*>, int > IntegerPairMap;

    int a = 0, b = 1, c = 2, d = 3;

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

TEST(map_ptr_stable_compare, checkMapPtrCopyAssignementOperator)
{
    typedef sofa::helper::map_ptr_stable_compare<int*, int> IntegerMap;

    int a = 0, b = 1, c = 2, d = 3;

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

TEST(map_ptr_stable_compare, checkMapPairPtrCopyForLoopInsert)
{
    typedef sofa::helper::map_ptr_stable_compare < std::pair<int*, int*>, int > IntegerPairMap;

    IntegerPairMap map;

    int a = 0, b = 1, c = 2, d = 3;

    map[std::make_pair(&a, &b)] = a + b;
    map[std::make_pair(&b, &c)] = b + c;
    map[std::make_pair(&c, &d)] = c + d;

    IntegerPairMap mapCopy;

    //std::copy(map.begin(), map.end(), std::inserter(mapCopy, mapCopy.begin()));

    for (const auto& kv : map)
    {
        mapCopy.insert(IntegerPairMap::value_type(kv.first, kv.second));
    }


    for (typename IntegerPairMap::const_iterator itCopy = mapCopy.begin(), it = map.begin(); it != map.end();
        ++itCopy, ++it)
    {
        EXPECT_EQ(it->first, itCopy->first);
        EXPECT_EQ(it->second, itCopy->second);
    }
}


TEST(map_ptr_stable_compare, CheckIdMapDoesNotPersistsAfterClear)
{
    typedef sofa::helper::map_ptr_stable_compare<int*, int> IntegerMap;
    typedef sofa::helper::ptr_stable_id< int* > IntegerStableId;

    int a = 0, b = 1, c = 2, d = 3;

    IntegerMap map;

    map[&a] = a;
    map[&b] = b;
    map[&c] = c;
    map[&d] = d;

    IntegerStableId* idMap = map.key_comp().get_stable_id_map();

    EXPECT_EQ(map.size(), 4u);

    map.clear();

    EXPECT_EQ(map.size(), 0u);
    EXPECT_EQ(idMap->getMap().size(), 0u);
}


TEST(map_ptr_stable_compare, CheckIdMapPtrStableConsistencyAfterErase)
{
    typedef sofa::helper::map_ptr_stable_compare<int*, int> IntegerMap;
    typedef sofa::helper::ptr_stable_id< int* > IntegerStableId;

    int a = 0, b = 1, c = 2, d = 3;

    IntegerMap map;
    map[&a] = a;
    map[&b] = b;
    map[&c] = c;
    map[&d] = d;

    typedef typename IntegerStableId::MapID StableMapID;
    const StableMapID& mapId = map.key_comp().get_stable_id_map()->getMap();


    ASSERT_EQ(4u, map.size());
    ASSERT_EQ(4u, mapId.size());

    // Size consistency after erase by key
    map.erase(&b);
    ASSERT_EQ(3u, map.size());
    EXPECT_EQ(3u, mapId.size());
    EXPECT_EQ(mapId.end(), mapId.find(&b));

    // Size consistency after erase by it
    typename IntegerMap::iterator itC = map.find(&c);
    map.erase(itC);
    ASSERT_EQ(2u, map.size());
    EXPECT_EQ(2u, mapId.size());
    EXPECT_EQ(mapId.end(), mapId.find(&c));

    typename IntegerMap::const_iterator it = map.begin();

    EXPECT_EQ(it->second, a);
    ++it;
    EXPECT_EQ(it->second, d);
    ++it;
    EXPECT_EQ(it, map.end());
}

TEST(map_ptr_stable_compare, CheckIdMapPairPtrStableConsistencyAfterErase)
{
    typedef sofa::helper::map_ptr_stable_compare < std::pair<int*, int*>, int > IntegerPairMap;
    typedef sofa::helper::ptr_stable_id< std::pair<int*, int*> > IntegerPairStableId;

    int a = 0, b = 1, c = 2, d = 3;

    IntegerPairMap map;
    map[std::make_pair(&a, &a)] = a;
    map[std::make_pair(&a, &b)] = a + b;
    map[std::make_pair(&b, &b)] = b;
    map[std::make_pair(&b, &c)] = b + c;
    map[std::make_pair(&c, &c)] = c;
    map[std::make_pair(&c, &d)] = c + d;
    map[std::make_pair(&d, &d)] = d;
    
    typedef typename IntegerPairStableId::MapID StableMapID;
    const StableMapID& mapId = map.key_comp().get_stable_id_map()->getMap();

    // Size consistency after erase by key
    ASSERT_EQ(7u, map.size());
    map.erase(std::make_pair(&b, &b));
    
    ASSERT_EQ(6u, map.size());

    // Size consistency after erase by it
    typename IntegerPairMap::iterator itC  = map.find(std::make_pair(&c, &c));
    map.erase(itC);
    ASSERT_EQ(5u, map.size());
    EXPECT_EQ(5u, mapId.size());
    EXPECT_EQ(mapId.end(), mapId.find(std::make_pair(&c, &c)));
    map.erase(std::make_pair(&c, &c));
    ASSERT_EQ(5u, map.size());
    EXPECT_EQ(5u, mapId.size());

    typename IntegerPairMap::const_iterator it = map.begin();

    EXPECT_EQ(it->second, a);
    ++it;
    EXPECT_EQ(it->second, a + b);
    ++it;
    EXPECT_EQ(it->second, b + c);
    ++it;
    EXPECT_EQ(it->second, c + d);

    auto itEraseBegin = map.begin();
    ++itEraseBegin;

    map.erase(itEraseBegin, map.end());

    ASSERT_EQ(1u, map.size());
    EXPECT_EQ(map.begin()->second, a);
    EXPECT_EQ(1u, mapId.size());
    EXPECT_EQ(mapId.begin(), mapId.find(std::make_pair(&a, &a)));
}

TEST(map_ptr_stable_compare, checkMapPtrStableConsistencyAfterSwap)
{
    typedef sofa::helper::map_ptr_stable_compare<int*, int> IntegerMap;
    typedef sofa::helper::ptr_stable_id< int* > IntegerStableId;

    int a = 0, b = 1, c = 2, d = 3, e = 4;
    
    IntegerMap map0, map1;
    map0[&a] = a;
    map0[&b] = b;
    map0[&c] = c;

    map1[&d] = d;
    map1[&e] = e;

    const IntegerMap map0Copy = map0, map1Copy = map1;

    std::size_t sizeMap0BeforeSwap = map0.size(), sizeMap1BeforeSwap = map1.size();
    map0.swap(map1);
    std::size_t sizeMap0AfterSwap = map0.size(), sizeMap1AfterSwap = map1.size();
    ASSERT_EQ(sizeMap0BeforeSwap, sizeMap1AfterSwap);
    ASSERT_EQ(sizeMap1BeforeSwap, sizeMap0AfterSwap);

    for (typename IntegerMap::const_iterator it1Copy = map1Copy.begin(), it0 = map0.begin(); it0 != map0.end();
        ++it1Copy, ++it0)
    {
        EXPECT_EQ(it0->first, it1Copy->first);
        EXPECT_EQ(it0->second, it1Copy->second);
    }

    for (typename IntegerMap::const_iterator it0Copy = map0Copy.begin(), it1 = map1.begin(); it1 != map1.end();
        ++it0Copy, ++it1)
    {
        EXPECT_EQ(it1->first, it0Copy->first);
        EXPECT_EQ(it1->second, it0Copy->second);
    }
}

TEST(map_ptr_stable_compare, checkMapPairPtrStableConsistencyAfterSwap)
{
    typedef sofa::helper::map_ptr_stable_compare < std::pair<int*, int*>, int > IntegerPairMap;
    typedef sofa::helper::ptr_stable_id< std::pair<int*, int*> > IntegerPairStableId;

    int a = 0, b = 1, c = 2, d = 3;

    IntegerPairMap map0, map1;
    map0[std::make_pair(&a, &a)] = a;
    map0[std::make_pair(&a, &b)] = a + b;
    map0[std::make_pair(&b, &c)] = b + c;
    
    map1[std::make_pair(&c, &c)] = c;
    map1[std::make_pair(&c, &d)] = c + d;

    const IntegerPairMap map0Copy = map0, map1Copy = map1;

    std::size_t sizeMap0BeforeSwap = map0.size(), sizeMap1BeforeSwap = map1.size();
    map0.swap(map1);
    std::size_t sizeMap0AfterSwap = map0.size(), sizeMap1AfterSwap = map1.size();
    ASSERT_EQ(sizeMap0BeforeSwap, sizeMap1AfterSwap);
    ASSERT_EQ(sizeMap1BeforeSwap, sizeMap0AfterSwap);

    for (typename IntegerPairMap::const_iterator it1Copy = map1Copy.begin(), it0 = map0.begin(); it0 != map0.end();
        ++it1Copy, ++it0)
    {
        EXPECT_EQ(it0->first, it1Copy->first);
        EXPECT_EQ(it0->second, it1Copy->second);
    }

    for (typename IntegerPairMap::const_iterator it0Copy = map0Copy.begin(), it1 = map1.begin(); it1 != map1.end();
        ++it0Copy, ++it1)
    {
        EXPECT_EQ(it1->first, it0Copy->first);
        EXPECT_EQ(it1->second, it0Copy->second);
    }
}

TEST(map_ptr_stable_compare, checkMapPtrStableModifierFunctions)
{
    typedef sofa::helper::map_ptr_stable_compare<int*, int> IntegerMap;

    int a = 0, b = 1, c = 2;

    IntegerMap mapInsert, mapEmplace, mapEmplaceHint;
    
    mapInsert.insert(IntegerMap::value_type(&a, a));
    mapInsert.insert(IntegerMap::value_type(&b, b));
    mapInsert.insert(IntegerMap::value_type(&b, c));

    mapEmplace.emplace(&a, a);
    mapEmplace.emplace(&b, b);
    mapEmplace.emplace(&b, c);

    mapEmplaceHint.emplace(&a, a);
    IntegerMap::iterator it = mapEmplaceHint.begin();
    it = mapEmplaceHint.emplace_hint(it, &b, b);
    mapEmplaceHint.emplace_hint(it, &b, c);

    typename IntegerMap::const_iterator itInsert = mapInsert.begin(), itEmplace = mapEmplace.begin(), itEmplaceHint = mapEmplaceHint.begin();

    EXPECT_EQ(itInsert->second, a);
    ++itInsert;
    EXPECT_EQ(itInsert->second, b);

    EXPECT_EQ(itEmplace->second, a);
    ++itEmplace;
    EXPECT_EQ(itEmplace->second, b);

    EXPECT_EQ(itEmplaceHint->second, a);
    ++itEmplaceHint;
    EXPECT_EQ(itEmplaceHint->second, b);
}

TEST(map_ptr_stable_compare, checkMapPairPtrStableModifierFunctions)
{
    typedef sofa::helper::map_ptr_stable_compare < std::pair<int*, int*>, int > IntegerPairMap;

    int a = 0, b = 1;

    IntegerPairMap mapInsert, mapEmplace, mapEmplaceHint;

    mapInsert.insert(IntegerPairMap::value_type(std::make_pair(&a, &a), a));
    mapInsert.insert(IntegerPairMap::value_type(std::make_pair(&b, &a), b));
    mapInsert.insert(IntegerPairMap::value_type(std::make_pair(&a, &a), b));

    mapEmplace.emplace(std::make_pair(&a, &a), a);
    mapEmplace.emplace(std::make_pair(&b, &a), b);
    mapEmplace.emplace(std::make_pair(&a, &a), b);

    mapEmplaceHint.emplace(std::make_pair(&a, &a), a);
    IntegerPairMap::iterator it = mapEmplaceHint.begin();
    it = mapEmplaceHint.emplace_hint(it, std::make_pair(&b, &a), b);
    mapEmplaceHint.emplace_hint(it, std::make_pair(&a, &a), b);

    typename IntegerPairMap::const_iterator itInsert = mapInsert.begin(), itEmplace = mapEmplace.begin(), itEmplaceHint = mapEmplaceHint.begin();

    EXPECT_EQ(itInsert->second, a);
    ++itInsert;
    EXPECT_EQ(itInsert->second, b);

    EXPECT_EQ(itEmplace->second, a);
    ++itEmplace;
    EXPECT_EQ(itEmplace->second, b);

    EXPECT_EQ(itEmplaceHint->second, a);
    ++itEmplaceHint;
    EXPECT_EQ(itEmplaceHint->second, b);
}

TEST(map_ptr_stable_compare, checkMapPtrStableLookupFunctions)
{
    typedef sofa::helper::map_ptr_stable_compare<int*, int> IntegerMap;

    int a = 0, b = 1, c = 2;

    IntegerMap map;
    map[&a] = a;
    map[&b] = b;

    // Count 0
    size_t count0C = map.count(&c);
    EXPECT_EQ(count0C, 0u);

    // Count 1
    map[&c] = c;
    size_t count1C = map.count(&c);
    EXPECT_EQ(count1C, 1u);

    // Find
    typename IntegerMap::const_iterator itFindKey = map.find(&b);
    EXPECT_EQ(itFindKey->first, &b);
    EXPECT_EQ(itFindKey->second, b);
}

TEST(map_ptr_stable_compare, checkMapPairPtrStableLookupFunctions)
{
    typedef sofa::helper::map_ptr_stable_compare < std::pair<int*, int*>, int > IntegerPairMap;

    int a = 0, b = 1, c = 2;

    IntegerPairMap map;
    map[std::make_pair(&a, &a)] = a;
    map[std::make_pair(&a, &b)] = b;
    map[std::make_pair(&b, &b)] = c;

    // Count 0
    size_t count0C = map.count(std::make_pair(&b, &c));
    EXPECT_EQ(count0C, 0u);
    count0C = map.count(std::make_pair(&b, &c));

    // Count 1
    map[std::make_pair(&b, &c)] = c;
    size_t count1C = map.count(std::make_pair(&b, &c));
    EXPECT_EQ(count1C, 1u);

    // Find
    typename IntegerPairMap::const_iterator itFindKey = map.find(std::make_pair(&a, &b));
    EXPECT_EQ(itFindKey->first, std::make_pair(&a, &b));
    EXPECT_EQ(itFindKey->second, b);
}

TEST(map_ptr_stable_compare, checkEraseThenInsertGivesAnotherKey)
{
    typedef sofa::helper::map_ptr_stable_compare < std::pair<int*, int*>, int > IntegerPairMap;
    typedef sofa::helper::ptr_stable_id< std::pair<int*, int*> >                IntegerPairStableId;

    int a = 0, b = 1, c = 2;

    IntegerPairMap map;
    map[std::make_pair(&a, &a)] = a;
    map[std::make_pair(&a, &b)] = b;
    map[std::make_pair(&b, &b)] = c;

    IntegerPairStableId* idMap = map.key_comp().get_stable_id_map();

    std::pair<int*,int*> key = std::make_pair(&a, &b);
    int value                = b;

    auto it     = idMap->getMap().find(key);
    ASSERT_NE(idMap->getMap().end(), it);
    unsigned idBeforeErase = it->second;

    map.erase(key);

    ASSERT_EQ(2u, map.size());

    it = idMap->getMap().find(key);
    EXPECT_EQ(idMap->getMap().end(), it);

    map.insert(std::make_pair(key,value));

    it = idMap->getMap().find(key);
    ASSERT_NE(idMap->getMap().end(), it);
    unsigned idAfterEraseThenInsert = it->second;

    EXPECT_GT(idAfterEraseThenInsert, idBeforeErase);
}



}
