
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
    Data<units::Force<double> > d_force;

    TestOnData()
        : d_test(initData("test", "helpMsg").addMeta(meta::Displayed(), meta::PossibleValues<double>(helper::vector<double>{1.2, 1.4, 1.5})))
        , d_force(initData("force", "helpMsg").addMeta(meta::ReadOnly()))
    {}

    void doStuff()
    {
        d_test.addMeta(meta::ReadOnly());
        d_test.addMeta(meta::Range<double>(helper::pair<double, double>{0, 10}));
        d_test.addMeta(meta::Units(std::array<int, 7>{{1, 1, -2, 0, 0, 0, 0}}));
        
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
        EXPECT_TRUE(d_test.getMeta(dTestReadOnly));

        meta::Displayed* dTestDisplayed;
        EXPECT_TRUE(d_test.getMeta(dTestDisplayed));
    }

    void doStuffWithTheForce()
    {
        meta::Units* dForceUnits;
        if (d_force.getMeta(dForceUnits))
        {
            EXPECT_EQ(dForceUnits->units[0], 1);
            EXPECT_EQ(dForceUnits->units[1], 1);
            EXPECT_EQ(dForceUnits->units[2], -2);

            meta::ReadOnly* dForceReadOnly;
            EXPECT_TRUE(d_force.getMeta(dForceReadOnly));
        }

    }
};


TEST(DataMetadataTest, checkMetadata)
{
    TestOnData test;
    test.doStuff();
    test.doStuffWithTheForce();
}

} // namespace dataMetadataTest
} // namespace sofa
