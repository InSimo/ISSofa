#include <gtest/gtest.h>
#include <sofa/helper/Factory.inl>
#include <memory>

namespace FactoryTestNamespace
{

class FactoryBaseObject
{
public:
    virtual ~FactoryBaseObject() {}
    virtual int getT() const = 0;
    virtual int getA() const = 0;
};

typedef sofa::helper::Factory<std::string,FactoryBaseObject,int,std::unique_ptr<FactoryBaseObject> > MyFactory;

template<int T>
class FactoryTestObject : public FactoryBaseObject
{
public:
    int m_a;
    FactoryTestObject(int a) : m_a(a) {}
    static std::unique_ptr<FactoryBaseObject> create(FactoryTestObject*, int a)
    {
        return std::unique_ptr<FactoryBaseObject>(new FactoryTestObject(a));
    }
    int getT() const override { return T; }
    int getA() const override { return m_a; }
};

std::unique_ptr<FactoryBaseObject> create2000IfOdd(FactoryTestObject<2000>*, int a)
{
    if (a%2)
        return std::unique_ptr<FactoryBaseObject>(new FactoryTestObject<2000>(a));
    else
        return std::unique_ptr<FactoryBaseObject>();
}

// 1 entry
sofa::helper::Creator<MyFactory,FactoryTestObject<42> > myCreator42("h2g2");
// 3 entries
sofa::helper::Creator<MyFactory,FactoryTestObject<-1> > myCreator_1("i2",false,0,"desc",{"-1","m1"});
// 2 entries
sofa::helper::Creator<MyFactory,FactoryTestObject<100> > myCreator100("multi",true,100,"prior100",{"100"});
// 2 entries
sofa::helper::Creator<MyFactory,FactoryTestObject<111> > myCreator111("multi",true,111,"prior111",{"111"});
// 2 entries
sofa::helper::Creator<MyFactory,FactoryTestObject<100> > myCreator100bis("multi",true,100,"prior100bis",{"100"});
// 2 entries
sofa::helper::Creator<MyFactory,FactoryTestObject<-100> > myCreator_100("multi",true,-100,"prior_100",{"m100"});
// 2 entries
sofa::helper::Creator<MyFactory,FactoryTestObject<1000> > myCreator1000("top",false,1000,"prior1000",{"1000"});
// 2 entries
sofa::helper::CreatorFn<MyFactory,FactoryTestObject<2000> > myCreator2000Fn("top2",create2000IfOdd,false,2000,"prior2000",{"2000"});
// Total: 16 entries

TEST( FactoryTest, CheckIterators)
{
    ASSERT_NE(nullptr, MyFactory::getInstance());
    EXPECT_EQ(16, std::distance(MyFactory::getInstance()->begin(),MyFactory::getInstance()->end()));
}

TEST( FactoryTest, CheckSingleObject)
{
    EXPECT_EQ(true, MyFactory::HasKey("h2g2"));
    std::unique_ptr<FactoryBaseObject> p = MyFactory::CreateObject("h2g2",1337);
    ASSERT_NE(nullptr, p.get());
    EXPECT_EQ(42, p->getT());
    EXPECT_EQ(1337, p->getA());
}

TEST( FactoryTest, CheckMissingKey)
{
    EXPECT_EQ(false, MyFactory::HasKey("h2o"));
    std::unique_ptr<FactoryBaseObject> p = MyFactory::CreateObject("h2o",1);
    EXPECT_EQ(nullptr, p.get());
}

TEST( FactoryTest, CheckMultiObject)
{
    EXPECT_EQ(true, MyFactory::HasKey("multi"));
    std::unique_ptr<FactoryBaseObject> p = MyFactory::CreateObject("multi",2);
    ASSERT_NE(nullptr, p.get());
    EXPECT_EQ(111, p->getT());
    EXPECT_EQ(2, p->getA());
}

TEST( FactoryTest, CheckCreatorFn)
{
    EXPECT_EQ(true, MyFactory::HasKey("top2"));
    std::unique_ptr<FactoryBaseObject> pEven = MyFactory::CreateObject("top2",666);
    EXPECT_EQ(nullptr, pEven.get());
    std::unique_ptr<FactoryBaseObject> pOdd = MyFactory::CreateObject("top2",667);
    ASSERT_NE(nullptr, pOdd.get());
    EXPECT_EQ(2000, pOdd->getT());
    EXPECT_EQ(667, pOdd->getA());
}

TEST( FactoryTest, CheckAnyObject)
{
    EXPECT_EQ(true, MyFactory::HasKey("top"));
    std::unique_ptr<FactoryBaseObject> p = MyFactory::CreateAnyObject(666);
    ASSERT_NE(nullptr, p.get());
    EXPECT_EQ(1000, p->getT());
    EXPECT_EQ(666, p->getA());
}

TEST( FactoryTest, CheckCreatorFnAny)
{
    EXPECT_EQ(true, MyFactory::HasKey("top2"));
    std::unique_ptr<FactoryBaseObject> p = MyFactory::CreateAnyObject(667);
    ASSERT_NE(nullptr, p.get());
    EXPECT_EQ(2000, p->getT());
    EXPECT_EQ(667, p->getA());
}

TEST( FactoryTest, CheckAliases)
{
    EXPECT_EQ(true, MyFactory::HasKey("-1"));
    EXPECT_EQ(true, MyFactory::HasKey("m1"));
    std::unique_ptr<FactoryBaseObject> p = MyFactory::CreateObject("100",10);
    ASSERT_NE(nullptr, p.get());
    EXPECT_EQ(100, p->getT());
    EXPECT_EQ(10, p->getA());
}

TEST( FactoryTest, CheckResetEntry)
{
    EXPECT_EQ(true, MyFactory::HasKey("i2"));
    MyFactory::ResetEntry("i2");
    EXPECT_EQ(false, MyFactory::HasKey("i2"));
    std::unique_ptr<FactoryBaseObject> p = MyFactory::CreateObject("i2",0);
    EXPECT_EQ(nullptr, p.get());
}

TEST( FactoryTest, CheckDuplicateEntry)
{
    EXPECT_EQ(false, MyFactory::HasKey("minus"));
    MyFactory::DuplicateEntry("-1","minus");
    EXPECT_EQ(true, MyFactory::HasKey("minus"));
    std::unique_ptr<FactoryBaseObject> p = MyFactory::CreateObject("minus",-2);
    ASSERT_NE(nullptr, p.get());
    EXPECT_EQ(-1, p->getT());
    EXPECT_EQ(-2, p->getA());
}

TEST( FactoryTest, CheckResetEntryMulti)
{
    EXPECT_EQ(true, MyFactory::HasKey("100"));
    MyFactory::ResetEntry("100");
    EXPECT_EQ(false, MyFactory::HasKey("100"));
    std::unique_ptr<FactoryBaseObject> p = MyFactory::CreateObject("100",0);
    EXPECT_EQ(nullptr, p.get());
}

TEST( FactoryTest, CheckDuplicateEntryMulti)
{
    EXPECT_EQ(false, MyFactory::HasKey("plenty"));
    MyFactory::DuplicateEntry("multi","plenty");
    EXPECT_EQ(true, MyFactory::HasKey("plenty"));
    std::unique_ptr<FactoryBaseObject> p = MyFactory::CreateObject("plenty",-2);
    ASSERT_NE(nullptr, p.get());
    EXPECT_EQ(111, p->getT());
    EXPECT_EQ(-2, p->getA());
}

}

template class sofa::helper::Factory<std::string,FactoryTestNamespace::FactoryBaseObject,int,std::unique_ptr<FactoryTestNamespace::FactoryBaseObject> >;
