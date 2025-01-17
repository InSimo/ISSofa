#include <plugins/Compliant/numericalsolver/MinresSolver.h>
#include <plugins/Compliant/numericalsolver/LDLTSolver.h>
#include <plugins/Compliant/Compliant_test/Compliant_test.h>
#include <plugins/Compliant/odesolver/CompliantImplicitSolver.h>
#include <SofaExplicitOdeSolver/EulerSolver.h>
#include <plugins/SceneCreator/SceneCreator.h>
using namespace sofa::modeling;
using namespace sofa::component;

namespace sofa
{

struct CompliantImplicitSolver_test : public CompliantSolver_test
{

    /** @defgroup CompliantImplicitSolver_Unit_Tests CompliantImplicitSolver basic tests.
     *
     * The scene is composed of two particles connected by a spring. One particle is fixed, while the other has an initial velocity.
     * The solver is set to backward Euler: alpha = beta = 1.
     */
    ///@{

    /** Test in the linear case, with velocity parallel to the spring.
      Convergence should occur in one iteration.
      Integrate using backward Euler (alpha=1, beta=1).
      Post-condition: an explicit Euler step with step -dt brings the system back to the original state.
      */
    void testLinearOneFixedOneStiffnessSpringV100(bool debug)
    {
        SReal dt=0.1; // currently, this must be set in the root node AND passed to the animate function
        Node::SPtr root = clearScene();
        root->setGravity( Vec3(0,0,0) );
        root->setDt(dt);

        // The solver
        using odesolver::CompliantImplicitSolver;
        CompliantImplicitSolver::SPtr complianceSolver = addNew<CompliantImplicitSolver>(getRoot());
        complianceSolver->debug.setValue(debug);
        complianceSolver->alpha.setValue(1.0);
        complianceSolver->beta.setValue(1.0);
        SReal precision = 1.0e-6;

        linearsolver::LDLTSolver::SPtr linearSolver = addNew<linearsolver::LDLTSolver>(getRoot());
        linearSolver->debug.setValue(debug);

        // The string
        ParticleString  string1( root, Vec3(0,0,0), Vec3(1,0,0), 2, 1.0*2 ); // two particles
        string1.compliance->isCompliance.setValue(false); // handle it as a stiffness
        string1.compliance->compliance.setValue(1.0e-3);

        FixedConstraint3::SPtr fixed = addNew<FixedConstraint3>(string1.string_node,"fixedConstraint");
        fixed->addConstraint(0);      // attach first particle

        // velocity parallel to the spring
        {
        MechanicalObject3::WriteVecCoord v = string1.DOF->writeVelocities();
        v[1] = Vec3(1,0,0);
        }


        //**************************************************
        sofa::simulation::getSimulation()->init(root.get());
        //**************************************************

        // initial state
        Vector x0 = getVector( core::VecId::position() );
        Vector v0 = getVector( core::VecId::velocity() );

        //**************************************************
        sofa::simulation::getSimulation()->animate(root.get(),dt);
        //**************************************************

        Vector x1 = getVector( core::VecId::position() );
        Vector v1 = getVector( core::VecId::velocity() );

        // An explicit integration step using the opposite dt should bring us back to the initial state
        string1.compliance->isCompliance.setValue(false); string1.compliance->reinit(); // switch the spring as stiffness
        core::MechanicalParams mparams;
        simulation::common::MechanicalOperations mop (&mparams,getRoot()->getContext());
        mop.computeForce( 0+dt, core::VecId::force(), core::VecId::position(), core::VecId::velocity(), false );
        Vector f1 = modeling::getVector( core::VecId::force() );
        Vector v2 = v1 - f1 * dt;
        Vector x2 = x1 - v1 * dt;

        if( debug ){
            cerr<<"CompliantImplicitSolver_test::testLinearOneFixedOneStiffnessSpringV100, time step : " << dt << endl;
            cerr<<"CompliantImplicitSolver_test::testLinearOneFixedOneStiffnessSpringV100, initial positions : " << x0.transpose() << endl;
            cerr<<"CompliantImplicitSolver_test::testLinearOneFixedOneStiffnessSpringV100, initial velocities: " << v0.transpose() << endl;
            cerr<<"CompliantImplicitSolver_test::testLinearOneFixedOneStiffnessSpringV100, new positions : " << x1.transpose() << endl;
            cerr<<"CompliantImplicitSolver_test::testLinearOneFixedOneStiffnessSpringV100, new velocities: " << v1.transpose() << endl;
            cerr<<"CompliantImplicitSolver_test::testLinearOneFixedOneStiffnessSpringV100, new forces: " << f1.transpose() << endl;
            cerr<<"CompliantImplicitSolver_test::testLinearOneFixedOneStiffnessSpringV100, new positions  after backward integration: " << x2.transpose() << endl;
            cerr<<"CompliantImplicitSolver_test::testLinearOneFixedOneStiffnessSpringV100, new velocities after backward integration: " << v2.transpose() << endl;
        }

        ASSERT_TRUE( (x2-x0).lpNorm<Eigen::Infinity>() < precision );
        ASSERT_TRUE( (v2-v0).lpNorm<Eigen::Infinity>() < precision );
    }

    /// Same as @testLinearOneFixedOneStiffnessSpringV100, with a compliant spring instead of a stiff spring
    void testLinearOneFixedOneComplianceSpringV100( bool debug )
    {
        SReal dt=0.1; // currently, this must be set in the root node AND passed to the animate function
        Node::SPtr root = clearScene();
        root->setGravity( Vec3(0,0,0) );
        root->setDt(dt);

        // The solver
        using odesolver::CompliantImplicitSolver;
        CompliantImplicitSolver::SPtr complianceSolver = addNew<CompliantImplicitSolver>(root);
        complianceSolver->debug.setValue( debug );
        complianceSolver->alpha.setValue(1.0);
        complianceSolver->beta.setValue(1.0);
        SReal precision = 1.0e-6;

        linearsolver::LDLTSolver::SPtr linearSolver = addNew<linearsolver::LDLTSolver>(root);
        linearSolver->debug.setValue(debug);

        // The string
        ParticleString  string1( root, Vec3(0,0,0), Vec3(1,0,0), 2, 1.0*2 ); // two particles
        string1.compliance->isCompliance.setValue(true);
        string1.compliance->compliance.setValue(1.0e-3);

        FixedConstraint3::SPtr fixed = modeling::addNew<FixedConstraint3>(string1.string_node,"fixedConstraint");
        fixed->addConstraint(0);      // attach first particle

        // velocity parallel to the spring
        {
        MechanicalObject3::WriteVecCoord v = string1.DOF->writeVelocities();
        v[1] = Vec3(1,0,0);
        }


        //**************************************************
        sofa::simulation::getSimulation()->init(root.get());
        //**************************************************

        // initial state
        Vector x0 = modeling::getVector( core::VecId::position() );
        Vector v0 = modeling::getVector( core::VecId::velocity() );

        //**************************************************
        sofa::simulation::getSimulation()->animate(root.get(),dt);
        //**************************************************

        Vector x1 = modeling::getVector( core::VecId::position() );
        Vector v1 = modeling::getVector( core::VecId::velocity() );

        // We check the explicit step backward without a solver, because it would not accumulate compliance forces
        string1.compliance->isCompliance.setValue(false); string1.compliance->reinit(); // switch the spring as stiffness
        core::MechanicalParams mparams;
        simulation::common::MechanicalOperations mop (&mparams,getRoot()->getContext());
        mop.computeForce( 0+dt, core::VecId::force(), core::VecId::position(), core::VecId::velocity(), false );
        Vector f1 = modeling::getVector( core::VecId::force() );
//        cerr<<"test, f1 = " << f1.transpose() << endl;

        // backward step
        Vector v2 = v1 - f1 * dt;
        Vector x2 = x1 - v1 * dt;

        if( debug ){
        cerr<<"CompliantImplicitSolver_test, initial positions : " << x0.transpose() << endl;
        cerr<<"CompliantImplicitSolver_test, initial velocities: " << v0.transpose() << endl;
        cerr<<"CompliantImplicitSolver_test, new positions : " << x1.transpose() << endl;
        cerr<<"CompliantImplicitSolver_test, new velocities: " << v1.transpose() << endl;
        cerr<<"CompliantImplicitSolver_test, new forces: " << f1.transpose() << endl;
        cerr<<"CompliantImplicitSolver_test, new positions  after backward integration: " << x2.transpose() << endl;
        cerr<<"CompliantImplicitSolver_test, new velocities after backward integration: " << v2.transpose() << endl;
        }

        ASSERT_TRUE( (x2-x0).lpNorm<Eigen::Infinity>() < precision );
        ASSERT_TRUE( (v2-v0).lpNorm<Eigen::Infinity>() < precision );
    }

    /// One stiffness spring, initially extendes
    void testLinearOneFixedOneStiffnessSpringX200( bool debug )
    {
        SReal dt=0.1;   // currently, this must be set in the root node AND passed to the animate function
        Node::SPtr root = clearScene();
        root->setGravity( Vec3(0,0,0) );
        root->setDt(dt);

        // The solver
        typedef odesolver::CompliantImplicitSolver OdeSolver;
        OdeSolver::SPtr odeSolver = addNew<OdeSolver>(root);
        odeSolver->debug.setValue(debug);
        odeSolver->alpha.setValue(1.0);
        odeSolver->beta.setValue(1.0);
        SReal precision = 1.0e-6;

        linearsolver::LDLTSolver::SPtr linearSolver = addNew<linearsolver::LDLTSolver>(root);
        linearSolver->debug.setValue(debug);

        // The string
        ParticleString  string1( root, Vec3(0,0,0), Vec3(1,0,0), 2, 1.0*2 ); // two particles
        string1.compliance->isCompliance.setValue(false);
        string1.compliance->compliance.setValue(1.0e-3);

        FixedConstraint3::SPtr fixed = modeling::addNew<FixedConstraint3>(string1.string_node,"fixedConstraint");
        fixed->addConstraint(0);      // attach first particle

        {
        MechanicalObject3::WriteVecCoord x = string1.DOF->writePositions();
        x[1] = Vec3(2,0,0);
        }


        //**************************************************
        sofa::simulation::getSimulation()->init(root.get());
        //**************************************************

        // initial state
        Vector x0 = modeling::getVector( core::VecId::position() );
        Vector v0 = modeling::getVector( core::VecId::velocity() );

        //**************************************************
        sofa::simulation::getSimulation()->animate(root.get(),dt);
        //**************************************************

        Vector x1 = modeling::getVector( core::VecId::position() );
        Vector v1 = modeling::getVector( core::VecId::velocity() );

        // We check the explicit step backward without a solver, because it would not accumulate compliance forces
        core::MechanicalParams mparams;
        simulation::common::MechanicalOperations mop (&mparams,getRoot()->getContext());
        mop.computeForce( 0+dt, core::VecId::force(), core::VecId::position(), core::VecId::velocity(), false );
        Vector f1 = modeling::getVector( core::VecId::force() );

        // backward step
        Vector v2 = v1 - f1 * dt;
        Vector x2 = x1 - v1 * dt;

//        cerr<<"CompliantImplicitSolver_test, initial positions : " << x0.transpose() << endl;
//        cerr<<"CompliantImplicitSolver_test, initial velocities: " << v0.transpose() << endl;
//        cerr<<"CompliantImplicitSolver_test, new positions     : " << x1.transpose() << endl;
//        cerr<<"CompliantImplicitSolver_test, new velocities    : " << v1.transpose() << endl;
//        cerr<<"CompliantImplicitSolver_test, new forces        : " << f1.transpose() << endl;
//        cerr<<"CompliantImplicitSolver_test, new positions  after backward integration: " << x2.transpose() << endl;
//        cerr<<"CompliantImplicitSolver_test, new velocities after backward integration: " << v2.transpose() << endl;

        // check that the implicit integration satisfies the implicit integration equation
        ASSERT_TRUE( (x2-x0).lpNorm<Eigen::Infinity>() < precision );
        ASSERT_TRUE( (v2-v0).lpNorm<Eigen::Infinity>() < precision );

        // The particle is initially in (2,0,0) and the closest rest configuration is (1,0,0)
        // The solution should therefore be inbetween.
        ASSERT_TRUE( x1(3)>1.0 ); // the spring should not get inversed


    }

    /// One compliant spring, initially extendes
    void testLinearOneFixedOneComplianceSpringX200( bool debug )
    {
        SReal dt=1;
        Node::SPtr root = clearScene();
        root->setGravity( Vec3(0,0,0) );
        root->setDt(dt);

        // The solver
        typedef odesolver::CompliantImplicitSolver OdeSolver;
        OdeSolver::SPtr odeSolver = addNew<OdeSolver>(root);
        odeSolver->debug.setValue(debug);
        odeSolver->alpha.setValue(1.0);
        odeSolver->beta.setValue(1.0);
        SReal precision = 1.0e-6;

        linearsolver::LDLTSolver::SPtr linearSolver = addNew<linearsolver::LDLTSolver>(root);
        linearSolver->debug.setValue(debug);

        // The string
        ParticleString  string1( root, Vec3(0,0,0), Vec3(1,0,0), 2, 1.0*2 ); // two particles
        string1.compliance->isCompliance.setValue(true);
        string1.compliance->compliance.setValue(1.0e-3);

        FixedConstraint3::SPtr fixed = modeling::addNew<FixedConstraint3>(string1.string_node,"fixedConstraint");
        fixed->addConstraint(0);      // attach first particle

        {
        MechanicalObject3::WriteVecCoord x = string1.DOF->writePositions();
        x[1] = Vec3(2,0,0);
        }


        //**************************************************
        sofa::simulation::getSimulation()->init(root.get());
        //**************************************************

        // initial state
        Vector x0 = modeling::getVector( core::VecId::position() );
        Vector v0 = modeling::getVector( core::VecId::velocity() );

        //**************************************************
        sofa::simulation::getSimulation()->animate(root.get(),dt);
        //**************************************************

        // state at the end of the step
        Vector x1 = modeling::getVector( core::VecId::position() );
        Vector v1 = modeling::getVector( core::VecId::velocity() );



        // ******* testing lambda propagation *******
        // ie checking force at the end of the time step
        // 1- no lambda propagation -> force must be null
        Vector f1 = modeling::getVector( core::VecId::force() );
        ASSERT_TRUE( f1.sum() == 0 );
        // 2- with lambda propagation -> force must be NOT null
        odeSolver->propagate_lambdas.setValue(true);
        {
        MechanicalObject3::WriteVecCoord x = string1.DOF->writePositions();
        x[1] = Vec3(2,0,0);
        MechanicalObject3::WriteVecCoord v = string1.DOF->writeVelocities();
        v.clear();
        }
        sofa::simulation::getSimulation()->init(root.get());
        sofa::simulation::getSimulation()->animate(root.get(),dt);
        f1 = modeling::getVector( core::VecId::force() );
        ASSERT_TRUE( f1.sum() != 0 );
        // **************


        // We check the explicit step backward without a solver, because it would not accumulate compliance forces
        string1.compliance->isCompliance.setValue(false); string1.compliance->reinit(); // switch the spring as stiffness
        core::MechanicalParams mparams;
        simulation::common::MechanicalOperations mop (&mparams,getRoot()->getContext());
        mop.computeForce( 0+dt, core::VecId::force(), core::VecId::position(), core::VecId::velocity(), false );
        f1 = modeling::getVector( core::VecId::force() );

        // backward step
        Vector v2 = v1 - f1 * dt;
        Vector x2 = x1 - v1 * dt;

//        cerr<<"CompliantImplicitSolver_test, initial positions : " << x0.transpose() << endl;
//        cerr<<"CompliantImplicitSolver_test, initial velocities: " << v0.transpose() << endl;
//        cerr<<"CompliantImplicitSolver_test, new positions     : " << x1.transpose() << endl;
//        cerr<<"CompliantImplicitSolver_test, new velocities    : " << v1.transpose() << endl;
//        cerr<<"CompliantImplicitSolver_test, new forces        : " << f1.transpose() << endl;
//        cerr<<"CompliantImplicitSolver_test, new positions  after backward integration: " << x2.transpose() << endl;
//        cerr<<"CompliantImplicitSolver_test, new velocities after backward integration: " << v2.transpose() << endl;

        // check that the implicit integration satisfies the implicit integration equation
        ASSERT_TRUE( (x2-x0).lpNorm<Eigen::Infinity>() < precision );
        ASSERT_TRUE( (v2-v0).lpNorm<Eigen::Infinity>() < precision );

        // The particle is initially in (2,0,0) and the closest rest configuration is (1,0,0)
        // The solution should therefore be inbetween.
        ASSERT_TRUE( x1(3)>1.0 ); // the spring should not get inversed


    }

};

//=================
// do run the tests
//=================
// simple linear cases
TEST_F(CompliantImplicitSolver_test, OneFixedOneComplianceSpringV100 ){    testLinearOneFixedOneComplianceSpringV100(false);  }
TEST_F(CompliantImplicitSolver_test, OneFixedOneStiffnessSpringV100 ){     testLinearOneFixedOneStiffnessSpringV100(false);  }
TEST_F(CompliantImplicitSolver_test, OneFixedOneStiffnessSpringX200 ){     testLinearOneFixedOneStiffnessSpringX200(false);  }
TEST_F(CompliantImplicitSolver_test, OneFixedOneComplianceSpringX200 ){    testLinearOneFixedOneComplianceSpringX200(false);  }

}// sofa



