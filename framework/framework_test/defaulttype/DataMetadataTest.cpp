
#include <gtest/gtest.h>
#include <sofa/defaulttype/Units.h>
#include <iostream>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>

#include <sofa/defaulttype/DataMetadata.h>
#include <sofa/defaulttype/QuantityTypeInfo.h>
#include <sofa/core/objectmodel/BaseClass.h>
#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{
namespace dataMetadataTest
{

class TestOnData : public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(TestOnData, core::objectmodel::BaseObject);

    Data<double > d_test;

    TestOnData()
        : d_test(initData("test", "helpMsg").addMeta(meta::Displayed(), meta::PossibleValues<double>(helper::vector<double>{1.2, 1.4, 1.5})))
    {}

    void doStuff()
    {
        d_test.addMeta(meta::ReadOnly());
        d_test.addMeta(meta::Range<double>(helper::pair<double, double>{0, 10}));
        d_test.addMeta(meta::Units(helper::fixed_array<int, 7>{1, 1, -2, 0, 0, 0, 0}));
        
        meta::Range<double>* dTestRange;
        if (d_test.getMeta(dTestRange))
        {
            EXPECT_EQ(dTestRange->range.first, 0);
            EXPECT_EQ(dTestRange->range.second, 10);

            dTestRange->range = helper::pair<double, double>{ 1,11 };            
            d_test.getMeta(dTestRange);
            EXPECT_EQ(dTestRange->range.first, 1);
            EXPECT_EQ(dTestRange->range.second, 11);

            d_test.setMeta(meta::Range<double>(helper::pair<double, double>{2, 12}));
            d_test.getMeta(dTestRange);
            EXPECT_EQ(dTestRange->range.first, 2);
            EXPECT_EQ(dTestRange->range.second, 12);
        }

        meta::ReadOnly* dTestReadOnly;
        if (d_test.getMeta(dTestReadOnly))
        {
            EXPECT_TRUE(dTestReadOnly->readOnly);
        }

        meta::Displayed* dTestDisplayed;
        if (d_test.getMeta(dTestDisplayed))
        {
            EXPECT_TRUE(dTestDisplayed->displayed);
            dTestDisplayed->displayed = false;
            d_test.getMeta(dTestDisplayed);
            EXPECT_FALSE(dTestDisplayed->displayed);
        }
    }

};


TEST(DataMetadataTest, checkMetadata)
{
    TestOnData test;
    test.doStuff();
}

} // namespace dataMetadataTest
} // namespace sofa
