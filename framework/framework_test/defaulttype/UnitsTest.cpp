
#include <gtest/gtest.h>
#include <sofa/defaulttype/Units.h>
#include <sofa/defaulttype/EnumTypeInfo.h>
#include <iostream>



namespace sofa
{
namespace unitsTest
{
    
TEST(UnitsTest, checkUnits)
{
    units::Mass         mass;
    units::Acceleration acceleration;
    units::Time         time;
    units::Force        force;
    units::Velocity     velocity;

    ////////////////////////
    // These lines of code will compile

    mass = 7.2 * 4 + 5;
    acceleration = 3.5;
    force = mass * units::Acceleration(15) * 10;
    force = 10 / force * force * force;

    units::Velocity     velocity2(velocity);

    time = 1.4;
    units::Time        time2(10);
    if(time < time2) { }

    std::cout << "DEBUG L" << __LINE__ << " : " << force.value() << std::endl;

    ////////////////////////
    ////// These lines of code won't compile => Enforcing Dimensional Unit Correctness

    // force = mass;
    // force *= acceleration;
    // defaulttype::uLength       length(time);    
    // if (force == 10)  { }
}


} // namespace unitsTest
} // namespace sofa