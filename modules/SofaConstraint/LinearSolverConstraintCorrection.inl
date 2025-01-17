/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 MGH, INRIA, USTL, UJF, CNRS, InSimo                *
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
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_CORE_COLLISION_LINEARSOLVERCONTACTCORRECTION_INL
#define SOFA_CORE_COLLISION_LINEARSOLVERCONTACTCORRECTION_INL

#include "LinearSolverConstraintCorrection.h"
#include <sofa/core/behavior/ConstraintCorrection.inl>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/core/visual/VisualParams.h>

#include <sstream>
#include <list>

namespace sofa
{

namespace component
{

namespace constraintset
{
#define MAX_NUM_CONSTRAINT_PER_NODE 100
#define EPS_UNITARY_FORCE 0.01

template<class DataTypes>
LinearSolverConstraintCorrection<DataTypes>::LinearSolverConstraintCorrection(sofa::core::behavior::MechanicalState<DataTypes> *mm)
: Inherit(mm)
, wire_optimization(initData(&wire_optimization, false, "wire_optimization", "constraints are reordered along a wire-like topology (from tip to base)"))
, solverName( initData(&solverName, "solverName", "name of the constraint solver") )
, d_complianceFactor(initData(&d_complianceFactor, (Real)1.0, "complianceFactor", "Factor applied to the position factor and velocity factor used to calculate compliance matrix"))
, odesolver(NULL)
{
}

template<class DataTypes>
LinearSolverConstraintCorrection<DataTypes>::~LinearSolverConstraintCorrection()
{
}



//////////////////////////////////////////////////////////////////////////
//   Precomputation of the Constraint Correction for all type of data
//////////////////////////////////////////////////////////////////////////

template<class DataTypes>
void LinearSolverConstraintCorrection<DataTypes>::init()
{
    Inherit::init();

    sofa::core::objectmodel::BaseContext* c = this->getContext();

    c->get(odesolver);

    const helper::vector<std::string>& solverNames = solverName.getValue();

    linearsolvers.clear();

    if (solverNames.size() == 0)
    {
        linearsolvers.push_back(c->get<sofa::core::behavior::LinearSolver>());
    }
    else
    {
        for (unsigned int i=0; i<solverNames.size(); ++i)
        {
            sofa::core::behavior::LinearSolver* s = NULL;
            c->get(s, solverNames[i]);
            if (s) linearsolvers.push_back(s);
            else serr << "Solver \"" << solverNames[i] << "\" not found." << sendl;
        }
    }

    if (odesolver == NULL)
    {
        serr << "LinearSolverConstraintCorrection: ERROR no OdeSolver found."<<sendl;
        return;
    }
    if (linearsolvers.size()==0)
    {
        serr << "LinearSolverConstraintCorrection: ERROR no LinearSolver found."<<sendl;
        return;
    }
#if 0 // refMinv is not use in normal case
    int n = mstate->getSize()*Deriv::size();

    std::stringstream ss;
    ss << this->getContext()->getName() << ".comp";
    std::string file=ss.str();
    sout << "try to open : " << ss.str() << sendl;
    if (sofa::helper::system::DataRepository.findFile(file))
    {
        std::string invName=sofa::helper::system::DataRepository.getFile(ss.str());
        std::ifstream compFileIn(invName.c_str(), std::ifstream::binary);
        refMinv.resize(n,n);
        //complianceLoaded = true;
        compFileIn.read((char*)refMinv.ptr(), n*n*sizeof(double));
        compFileIn.close();
    }
#endif
}

template<class TDataTypes>
void LinearSolverConstraintCorrection<TDataTypes>::computeJ(sofa::defaulttype::BaseMatrix* W, const MatrixDeriv& c)
{
    const unsigned int numDOFs = this->mstate->getSize();
    const unsigned int N = Deriv::size();
    const unsigned int numDOFReals = numDOFs*N;
    const unsigned int totalNumConstraints = W->rowSize();

    J.resize(totalNumConstraints, numDOFReals);

    MatrixDerivRowConstIterator rowItEnd = c.end();

    for (MatrixDerivRowConstIterator rowIt = c.begin(); rowIt != rowItEnd; ++rowIt)
    {
        const int cid = rowIt.index();

        MatrixDerivColConstIterator colItEnd = rowIt.end();

        for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
        {
            const unsigned int dof = colIt.index();
            const Deriv n = colIt.val();

            for (unsigned int r = 0; r < N; ++r)
            {
                J.add(cid, dof * N + r, n[r]);
            }
        }
    }
}

template<class DataTypes>
void LinearSolverConstraintCorrection<DataTypes>::addComplianceInConstraintSpace(const sofa::core::ConstraintParams *cparams, sofa::defaulttype::BaseMatrix* W)
{
    if (!this->mstate || !odesolver || (linearsolvers.size()==0)) return;

    const Real complianceFactor = d_complianceFactor.getValue();

    // use the OdeSolver to get the position integration factor
    double factor = 1.0;

    switch (cparams->constOrder())
    {
    case core::ConstraintParams::POS_AND_VEL :
    case core::ConstraintParams::POS :
        factor = odesolver->getPositionIntegrationFactor();
        break;

    case core::ConstraintParams::ACC :
    case core::ConstraintParams::VEL :
        factor = odesolver->getVelocityIntegrationFactor();
        break;

    default :
        break;
    }

#if 0 // refMinv is not use in normal case    
    const unsigned int numDOFs = this->mstate->getSize();
    const unsigned int N = Deriv::size();
    const unsigned int numDOFReals = numDOFs*N;

    if (refMinv.rowSize() > 0)			// What's for ??
    {
        std::cout<<"refMinv.rowSize() > 0"<<std::endl;
        J.resize(numDOFReals,numDOFReals);
        for (unsigned int i=0; i<numDOFReals; ++i)
            J.set(i,i,1);
        defaulttype::FullMatrix<Real> Minv;
        Minv.resize(numDOFReals,numDOFReals);
        // use the Linear solver to compute J*inv(M)*Jt, where M is the mechanical linear system matrix
        linearsolvers[0]->addJMInvJt(&Minv, &J, factor);
        double err=0,fact=0;
        for (unsigned int i=0; i<numDOFReals; ++i)
            for (unsigned int j=0; j<numDOFReals; ++j)
            {
                //sout << "Minv("<<i<<","<<j<<") = "<<Minv.element(i,j)<<"\t refMinv("<<i<<","<<j<<") = "<<refMinv.element(i,j)<<sendl;
                if (fabs(refMinv.element(i,j)) > 1.0e-30)
                {
                    err += fabs(Minv.element(i,j)-refMinv.element(i,j))/refMinv.element(i,j);
                    fact += fabs(Minv.element(i,j)/refMinv.element(i,j));
                }
                else
                {
                    err += fabs(Minv.element(i,j)-refMinv.element(i,j));
                    fact += 1.0f;
                }
            }
        sout << "LinearSolverConstraintCorrection: mean relative error: "<<err/(SReal)(numDOFReals*numDOFReals)<<sendl;
        sout << "LinearSolverConstraintCorrection: mean relative factor: "<<fact/(SReal)(numDOFReals*numDOFReals)<<sendl;
        refMinv.resize(0,0);
    }
#endif

    // Compute J
    this->computeJ(W, cparams->readJ(this->mstate)->getValue(cparams));

    factor *= complianceFactor;

    // use the Linear solver to compute J*inv(M)*Jt, where M is the mechanical linear system matrix
    for (unsigned i = 0; i < linearsolvers.size(); i++)
    {
        linearsolvers[i]->setSystemLHVector(sofa::core::MultiVecDerivId::null());
        linearsolvers[i]->addJMInvJt(W, &J, factor);
    }
}


template<class DataTypes>
void LinearSolverConstraintCorrection<DataTypes>::rebuildSystem(double massFactor, double forceFactor)
{
    for (unsigned i = 0; i < linearsolvers.size(); i++)
    {
        //serr << "REBUILD " <<  linearsolvers[i]->getName() << " m="<<massFactor << "f=" << forceFactor << sendl;
        linearsolvers[i]->rebuildSystem(massFactor, forceFactor);
    }
}

template<class DataTypes>
void LinearSolverConstraintCorrection<DataTypes>::getComplianceMatrix(defaulttype::BaseMatrix* Minv) const
{
    if (!this->mstate || !odesolver || (linearsolvers.size()==0)) return;

    const Real complianceFactor = d_complianceFactor.getValue();

    // use the OdeSolver to get the position integration factor
    //const double factor = 1.0;
    //const double factor = odesolver->getPositionIntegrationFactor(); // dt
    const double factor = odesolver->getPositionIntegrationFactor() * complianceFactor; //*odesolver->getPositionIntegrationFactor(); // dt*dt

    const unsigned int numDOFs = this->mstate->getSize();
    const unsigned int N = Deriv::size();
    const unsigned int numDOFReals = numDOFs*N;
    static linearsolver::SparseMatrix<SReal> J; //local J
    if (J.rowSize() != (defaulttype::BaseMatrix::Index)numDOFReals)
    {
        J.resize(numDOFReals,numDOFReals);
        for (unsigned int i=0; i<numDOFReals; ++i)
            J.set(i,i,1);
    }

    Minv->resize(numDOFReals,numDOFReals);
    // use the Linear solver to compute J*inv(M)*Jt, where M is the mechanical linear system matrix
    linearsolvers[0]->addJMInvJt(Minv, &J, factor);
#if 0 // refMinv is not use in normal case
    double err=0,fact=0;
    for (unsigned int i=0; i<numDOFReals; ++i)
        for (unsigned int j=0; j<numDOFReals; ++j)
        {
            //sout << "Minv("<<i<<","<<j<<") = "<<Minv.element(i,j)<<"\t refMinv("<<i<<","<<j<<") = "<<refMinv.element(i,j)<<sendl;
            if (fabs(refMinv.element(i,j)) > 1.0e-30)
            {
                err += fabs(Minv->element(i,j)-refMinv.element(i,j))/refMinv.element(i,j);
                fact += fabs(Minv->element(i,j)/refMinv.element(i,j));
            }
            else
            {
                err += fabs(Minv->element(i,j)-refMinv.element(i,j));
                fact += 1.0f;
            }
        }
    sout << "LinearSolverConstraintCorrection: mean relative error: "<<err/(SReal)(numDOFReals*numDOFReals)<<sendl;
    sout << "LinearSolverConstraintCorrection: mean relative factor: "<<fact/(SReal)(numDOFReals*numDOFReals)<<sendl;
#endif
}

template< class DataTypes >
void LinearSolverConstraintCorrection< DataTypes >::computeMotionCorrection(const core::ConstraintParams* /*cparams*/, core::MultiVecDerivId dx, core::MultiVecDerivId f)
{
    if (this->mstate)
    {
        linearsolvers[0]->setSystemRHVector(f);
        linearsolvers[0]->setSystemLHVector(dx);
        linearsolvers[0]->solveSystem();
        linearsolvers[0]->writeSolution();
    }
}

template< class DataTypes >
void LinearSolverConstraintCorrection< DataTypes >::applyMotionCorrection(const core::ConstraintParams * cparams, Data< VecCoord > &x_d, Data< VecDeriv > &v_d, Data< VecDeriv > &dx_d, const Data< VecDeriv > &correction_d)
{
    if (this->mstate)
    {
        const unsigned int numDOFs = this->mstate->getSize();

        auto x = sofa::helper::write(x_d, cparams);
        auto v = sofa::helper::write(v_d, cparams);
        auto dx = sofa::helper::write(dx_d, cparams);

        const Real complianceFactor = d_complianceFactor.getValue(cparams);
        const VecDeriv& correction = correction_d.getValue(cparams);
        const VecCoord& x_free = cparams->readX(this->mstate)->getValue();
        const VecDeriv& v_free = cparams->readV(this->mstate)->getValue();

        const double positionFactor = odesolver->getPositionIntegrationFactor() * complianceFactor;
        const double velocityFactor = odesolver->getVelocityIntegrationFactor() * complianceFactor;

        for (unsigned int i = 0; i < numDOFs; i++)
        {
            const Deriv dxi = correction[i] * positionFactor;
            const Deriv dvi = correction[i] * velocityFactor;
            x[i] = x_free[i] + dxi;
            v[i] = v_free[i] + dvi;
            dx[i] = dxi;
        }
    }
}


template< class DataTypes >
void LinearSolverConstraintCorrection< DataTypes >::applyPositionCorrection(const sofa::core::ConstraintParams *cparams, Data< VecCoord >& x_d, Data< VecDeriv>& dx_d, const Data< VecDeriv >& correction_d)
{
    if (this->mstate)
    {
        const unsigned int numDOFs = this->mstate->getSize();
        auto x  = sofa::helper::write(x_d, cparams);
        auto dx = sofa::helper::write(dx_d, cparams);

        const Real complianceFactor = d_complianceFactor.getValue(cparams);
        const VecDeriv& correction = correction_d.getValue(cparams);
        const VecCoord& x_free = cparams->readX(this->mstate)->getValue();

        const double positionFactor = odesolver->getPositionIntegrationFactor() * complianceFactor;
        for (unsigned int i = 0; i < numDOFs; i++)
        {
            const Deriv dxi = correction[i] * positionFactor;
            x[i] = x_free[i] + dxi;
            dx[i] = dxi;
        }
    }
}


template< class DataTypes >
void LinearSolverConstraintCorrection< DataTypes >::applyVelocityCorrection(const sofa::core::ConstraintParams *cparams, Data< VecDeriv>& v_d, Data< VecDeriv>& dv_d, const Data< VecDeriv >& correction_d)
{
    if (this->mstate)
    {
        const unsigned int numDOFs = this->mstate->getSize();

        auto v  = sofa::helper::write(v_d,cparams);
        auto dv = sofa::helper::write(dv_d, cparams); 

        const Real complianceFactor = d_complianceFactor.getValue(cparams);
        const VecDeriv& correction = correction_d.getValue(cparams);
        const VecDeriv& v_free     = cparams->readV(this->mstate)->getValue();

        const double velocityFactor = odesolver->getVelocityIntegrationFactor() * complianceFactor;

        for (unsigned int i = 0; i < numDOFs; i++)
        {
            Deriv dvi = correction[i] * velocityFactor;
            v[i] = v_free[i] + dvi;
            dv[i] = dvi;
        }
    }
}


template<class DataTypes>
void LinearSolverConstraintCorrection<DataTypes>::applyContactForce(const defaulttype::BaseVector *f)
{
    core::VecDerivId forceID(core::VecDerivId::V_FIRST_DYNAMIC_INDEX);
    core::VecDerivId dxID = core::VecDerivId::dx();

    const unsigned int numDOFs = this->mstate->getSize();

    Data<VecDeriv>& dataDx = *this->mstate->write(dxID);
    VecDeriv& dx = *dataDx.beginEdit();

    dx.clear();
    dx.resize(numDOFs);

    Data<VecDeriv>& dataForce = *this->mstate->write(forceID);
    VecDeriv& force = *dataForce.beginEdit();

    force.clear();
    force.resize(numDOFs);
#if 0
    const unsigned int N = Deriv::size();
    const unsigned int numDOFReals = numDOFs*N;
    F.resize(numDOFReals);
    const defaulttype::FullVector<Real>* fcast = defaulttype::FullVector<Real>::DynamicCast(f);
    if (fcast)
        J.addMulTranspose(F, *fcast); // fast
    else
        J.addMulTranspose(F, f); // slow but generic
    for (unsigned int i=0; i< numDOFs; i++)
        for (unsigned int r=0; r<N; ++r)
            force[i][r] = F[i*N+r];
#else
    const MatrixDeriv& c = this->mstate->read(core::ConstMatrixDerivId::constraintJacobian())->getValue();

    MatrixDerivRowConstIterator rowItEnd = c.end();

    for (MatrixDerivRowConstIterator rowIt = c.begin(); rowIt != rowItEnd; ++rowIt)
    {
        const double fC1 = f->element(rowIt.index());

        if (fC1 != 0.0)
        {
            MatrixDerivColConstIterator colItEnd = rowIt.end();

            for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
            {
                force[colIt.index()] += colIt.val() * fC1;
            }
        }
    }
#endif

    //for (unsigned int i=0; i< numDOFs; i++)
    //    sout << "f("<<i<<")="<<force[i]<<sendl;
    linearsolvers[0]->setSystemRHVector(forceID);
    linearsolvers[0]->setSystemLHVector(dxID);
    linearsolvers[0]->solveSystem();

    //TODO: tell the solver not to recompute the matrix

    const Real complianceFactor = d_complianceFactor.getValue();

    // use the OdeSolver to get the position integration factor
    const double positionFactor = odesolver->getPositionIntegrationFactor() * complianceFactor;

    // use the OdeSolver to get the position integration factor
    const double velocityFactor = odesolver->getVelocityIntegrationFactor() * complianceFactor;

    Data<VecCoord>& xData     = *this->mstate->write(core::VecCoordId::position());
    Data<VecDeriv>& vData     = *this->mstate->write(core::VecDerivId::velocity());
    const Data<VecCoord> & xfreeData = *this->mstate->read(core::ConstVecCoordId::freePosition());
    const Data<VecDeriv> & vfreeData = *this->mstate->read(core::ConstVecDerivId::freeVelocity());
    VecCoord& x = *xData.beginEdit();
    VecDeriv& v = *vData.beginEdit();
    const VecCoord& x_free = xfreeData.getValue();
    const VecDeriv& v_free = vfreeData.getValue();

    for (unsigned int i=0; i< numDOFs; i++)
    {
        //sout << "dx("<<i<<")="<<dx[i]<<sendl;
        Deriv dxi = dx[i]*positionFactor;
        Deriv dvi = dx[i]*velocityFactor;
        x[i] = x_free[i] + dxi;
        v[i] = v_free[i] + dvi;
        dx[i] = dxi;

        if (this->f_printLog.getValue()) std::cout << "dx[" << i << "] = " << dx[i] << std::endl;
    }


//     for (MatrixDerivRowConstIterator rowIt = c.begin(); rowIt != rowItEnd; ++rowIt)
//     {
//         const double fC1 = f->element(rowIt.index());
//
//         if (fC1 != 0.0)
//         {
//             MatrixDerivColConstIterator colItEnd = rowIt.end();
//
//             for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
//             {
//                 v[colIt.index()] = Deriv();
//             }
//         }
//     }

    dataDx.endEdit();
    dataForce.endEdit();
    xData.endEdit();
    vData.endEdit();

    /// @todo: freeing forceID here is incorrect as it was not allocated
    /// Maybe the call to vAlloc at the beginning of this method should be enabled...
    /// -- JeremieA, 2011-02-16
    this->mstate->vFree(core::ExecParams::defaultInstance(), forceID);
}


template<class DataTypes>
void LinearSolverConstraintCorrection<DataTypes>::resetContactForce()
{
    Data<VecDeriv>& forceData = *this->mstate->write(core::VecDerivId::force());
    VecDeriv& force = *forceData.beginEdit();
    for( unsigned i=0; i<force.size(); ++i )
        force[i] = Deriv();
    forceData.endEdit();
}


template<class DataTypes>
bool LinearSolverConstraintCorrection<DataTypes>::hasConstraintNumber(int index)
{
    const MatrixDeriv& c = this->mstate->read(core::ConstMatrixDerivId::constraintJacobian())->getValue();

    return c.readLine(index) != c.end();
}


template<class DataTypes>
void LinearSolverConstraintCorrection<DataTypes>::verify_constraints()
{
    // New design prevents duplicated constraints.
}

template<class DataTypes>
void LinearSolverConstraintCorrection<DataTypes>::resetForUnbuiltResolution(double * f, std::list<unsigned int>& renumbering)
{
    if (d_complianceFactor.getValue() != 1)
    {
        sout << " compliance factor isn't taken into account in unbuild version " << sendl;
    }

    //std::cout<<" \n \n \n +++++++ resetForUnbuiltResolution  +++++++++ "<<std::endl;
    verify_constraints();


    const MatrixDeriv& constraints = this->mstate->read(core::ConstMatrixDerivId::constraintJacobian())->getValue();

    constraint_force.clear();
    constraint_force.resize(this->mstate->getSize());

    constraint_dofs.clear();

    ////// TODO : supprimer le classement par indice max
    //std::vector<unsigned int> VecMaxDof;
    //VecMaxDof.resize(numConstraints);

    const unsigned int nbConstraints = constraints.size();
    std::vector<unsigned int> VecMinDof;
    VecMinDof.resize(nbConstraints);

    unsigned int c = 0;

    MatrixDerivRowConstIterator rowItEnd = constraints.end();

    for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
    {

        const int indexC = rowIt.index(); // id constraint


        // buf the value of force applied on concerned dof : constraint_force
        // buf a table of indice of involved dof : constraint_dofs
        double fC = f[indexC];

        if (fC != 0.0)
        {
            MatrixDerivColConstIterator colItEnd = rowIt.end();

            for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
            {
                const unsigned int dof = colIt.index();
                constraint_force[dof] += colIt.val() * fC;
            }
        }

        //////////// for wire optimization ////////////
        // VecMinDof contains the smallest id of the dofs involved in each constraint [c]

        MatrixDerivColConstIterator colItEnd = rowIt.end();


        VecMinDof[c] = this->mstate->getSize()+1;

        for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
        {
            const unsigned int dof = colIt.index();
            constraint_dofs.push_back(dof);
            if (dof < VecMinDof[c])
                VecMinDof[c] = dof;
        }

        c++;
    }
    // constraint_dofs buff the DOF that are involved with the constraints
    constraint_dofs.unique();


    // in the following the list "renumbering" is modified so that the constraint, in the list appears from the smallest dof to the greatest
    // However some constraints are not concerned by the structure... in such case, their order should not be changed
    // (in practice they will be put at the beginning of the list)
    if (wire_optimization.getValue())
    {
        //std::cout<<" wire_optimization "<<std::endl;


        std::vector< std::vector<unsigned int> > ordering_per_dof;
        ordering_per_dof.resize(this->mstate->getSize());   // for each dof, provide the list of constraint for which this dof is the smallest involved

        MatrixDerivRowConstIterator rowItEnd = constraints.end();
        unsigned int c = 0;


        // we process each constraint of the Mechanical State to know the smallest dofs
        // the constraints that are concerns the object are removed
        for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
        {
            ordering_per_dof[VecMinDof[c]].push_back(rowIt.index());
            c++;
            renumbering.remove( rowIt.index() );
        }


        // fill the end renumbering list with the new order

        for (int dof = 0; dof < this->mstate->getSize(); dof++)
        {
            for (unsigned int c = 0; c < ordering_per_dof[dof].size(); c++)
            {
                renumbering.push_back(ordering_per_dof[dof][c]); // push_back the list of constraint by starting from the smallest dof
            }
        }



//        std::cout<<" +++++ RENUBERING END= [";
//        for (std::list<unsigned int>::iterator it_c = renumbering.begin(); it_c!=  renumbering.end(); it_c++)
//            std::cout<<(*it_c)<<" ";
//        std::cout<<"]"<<std::endl;
    }


    /////////////// SET INFO FOR LINEAR SOLVER /////////////

    core::VecDerivId forceID(core::VecDerivId::V_FIRST_DYNAMIC_INDEX);
    core::VecDerivId dxID = core::VecDerivId::dx();

    linearsolvers[0]->setSystemRHVector(forceID);
    linearsolvers[0]->setSystemLHVector(dxID);


    systemMatrix_buf   = linearsolvers[0]->getSystemBaseMatrix();
    systemRHVector_buf = linearsolvers[0]->getSystemRHBaseVector();
    systemLHVector_buf = linearsolvers[0]->getSystemLHBaseVector();

    // systemRHVector_buf is set to constraint_force;
    //std::cerr<<"WARNING: resize is called"<<std::endl;
    const unsigned int derivDim = Deriv::size();
    const unsigned int systemSize = this->mstate->getSize() * derivDim;
    systemRHVector_buf->resize(systemSize) ;
    systemLHVector_buf->resize(systemSize) ;
    //std::cerr<<"resize ok"<<std::endl;



    for ( int i=0; i<this->mstate->getSize(); i++)
    {
        for  (unsigned int j=0; j<derivDim; j++)
            systemRHVector_buf->set(i*derivDim+j, constraint_force[i][j]);
    }

    // Init the internal data of the solver for partial solving
    linearsolvers[0]->init_partial_solve();


    ///////// new : precalcul des liste d'indice ///////
    Vec_I_list_dof.clear(); // clear = the list is filled during the block compliance computation

	// Resize   
    unsigned int maxIdConstraint = 0;
    for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
    {
        const unsigned int indexC = rowIt.index(); // id constraint
        if (indexC > maxIdConstraint) // compute the max of the Id
            maxIdConstraint = indexC;
    }

    Vec_I_list_dof.resize(maxIdConstraint + 1);

    last_disp = 0;
    last_force = nbConstraints - 1;
    _new_force = false;
}

template<class DataTypes>
void LinearSolverConstraintCorrection<DataTypes>::addConstraintDisplacement(double *d, int begin, int end)
{
    //std::cout<<" addConstraintDisplacement... begin ="<<begin<<"  end ="<<end<<std::endl ;

    const MatrixDeriv& constraints = this->mstate->read(core::ConstMatrixDerivId::constraintJacobian())->getValue();
    const unsigned int derivDim = Deriv::size();

    last_disp = begin;


    linearsolvers[0]->partial_solve(Vec_I_list_dof[last_disp], Vec_I_list_dof[last_force], _new_force);

    _new_force = false;

    // TODO => optimisation => for each bloc store J[bloc,dof]
    for (int i = begin; i <= end; i++)
    {
        MatrixDerivRowConstIterator rowIt = constraints.readLine(i);

        if (rowIt != constraints.end()) // useful ??
        {
            MatrixDerivColConstIterator rowEnd = rowIt.end();

            for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != rowEnd; ++colIt)
            {
                const unsigned int dof = colIt.index();
                Deriv disp;

                for(unsigned int j = 0; j < derivDim; j++)
                {
                    disp[j] = (Real)(systemLHVector_buf->element(dof * derivDim + j) * odesolver->getPositionIntegrationFactor());
                }

                d[i] += colIt.val() * disp;  // J[bloc[i],dof] * D[dof]  // dof= Vec_I_list_dof[i][colIt ]
            }
        }
    }
}

template<class DataTypes>
void LinearSolverConstraintCorrection<DataTypes>::setConstraintDForce(double *df, int begin, int end, bool update)
{


    const MatrixDeriv& constraints = this->mstate->read(core::ConstMatrixDerivId::constraintJacobian())->getValue();
    const unsigned int derivDim = Deriv::size();


    last_force = begin;

    if (!update)
        return;

    _new_force = true;



    //std::cout<<" setConstraintDForce : df = ";
    // TODO => optimisation !!!
    for (int i = begin; i <= end; i++)
    {

        //std::cout<<"["<<i<<"]="<<df[i]<<"  ";
        MatrixDerivRowConstIterator rowIt = constraints.readLine(i);

        if (rowIt != constraints.end())
        {
            MatrixDerivColConstIterator colItEnd = rowIt.end();

            for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
            {
                const Deriv n = colIt.val();
                const unsigned int dof = colIt.index();

                constraint_force[dof] += n * df[i]; // sum of the constraint force in the DOF space

            }
        }
    }
    //std::cout<<std::endl;

    //std::cout<<"constraint_force = "<<constraint_force<<std::endl;


    // course on indices of the dofs involved invoved in the bloc //
    std::list<int>::const_iterator it_dof(Vec_I_list_dof[last_force].begin()), it_end(Vec_I_list_dof[last_force].end());
    for(; it_dof!=it_end; ++it_dof)
    {
        int dof =(*it_dof) ;
        for  (unsigned int j=0; j<derivDim; j++)
            systemRHVector_buf->set(dof * derivDim + j, constraint_force[dof][j]);
    }

}

template<class DataTypes>
void LinearSolverConstraintCorrection<DataTypes>::getBlockDiagonalCompliance(defaulttype::BaseMatrix* W, int begin, int end)
{

    // std::cout<<" getBlockDiagonalCompliance..." ;

    if (!this->mstate || !odesolver || (linearsolvers.size()==0)) return;

    // use the OdeSolver to get the position integration factor
    const double factor = odesolver->getPositionIntegrationFactor(); //*odesolver->getPositionIntegrationFactor(); // dt*dt

    const unsigned int numDOFs = this->mstate->getSize();
    const unsigned int N = Deriv::size();
    const unsigned int numDOFReals = numDOFs*N;

    // Compute J
    const MatrixDeriv& constraints = this->mstate->read(core::ConstMatrixDerivId::constraintJacobian())->getValue();
    const unsigned int totalNumConstraints = W->rowSize();

    J.resize(totalNumConstraints, numDOFReals);

    for (int i = begin; i <= end; i++)
    {


        MatrixDerivRowConstIterator rowIt = constraints.readLine(i);

        if (rowIt != constraints.end())
        {
            MatrixDerivColConstIterator colItEnd = rowIt.end();

            unsigned int dof_buf = 0;
            int debug = 0;

            for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
            {
                const unsigned int dof = colIt.index();
                const Deriv n = colIt.val();

                for (unsigned int r = 0; r < N; ++r)
                    J.add(i, dof * N + r, n[r]);

                if (debug!=0)
                {
                    int test = dof_buf - dof;
                    if (test>2 || test< -2)
                        sout << "YES !!!! for constraint id1 dof1 = " << dof_buf << " dof2 = " << dof << sendl;
                }

                dof_buf = dof;
            }
        }
    }

    // use the Linear solver to compute J*inv(M)*Jt, where M is the mechanical linear system matrix
    linearsolvers[0]->addJMInvJt(W, &J, factor);

    // construction of  Vec_I_list_dof : vector containing, for each constraint block, the list of dof concerned

    ListIndex list_dof;

    for (int i = begin; i <= end; i++)
    {


        MatrixDerivRowConstIterator rowIt = constraints.readLine(i);

        if (rowIt != constraints.end())
        {
            MatrixDerivColConstIterator colItEnd = rowIt.end();

            for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
            {
                list_dof.push_back(colIt.index());
            }
        }
    }

    list_dof.sort();
    list_dof.unique();

    for (int i = begin; i <= end; i++)
    {
        Vec_I_list_dof[i] = list_dof;
    }

    //std::cout<<" end "<<std::endl;

}

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
