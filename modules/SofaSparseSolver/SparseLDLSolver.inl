/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
// Author: Hadrien Courtecuisse

#ifndef SOFA_COMPONENT_LINEARSOLVER_SparseLDLSolver_INL
#define SOFA_COMPONENT_LINEARSOLVER_SparseLDLSolver_INL

#include <sofa/SofaGeneral.h>
#include <SofaSparseSolver/SparseLDLSolver.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include "sofa/helper/system/thread/CTime.h"
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/behavior/LinearSolver.h>
#include <math.h>
#include <sofa/helper/system/thread/CTime.h>
#include <SofaBaseLinearSolver/CompressedRowSparseMatrix.inl>
#include <SofaConstraint/ConstraintSolverImpl.h>
#include <sofa/helper/AdvancedTimer.h>

namespace sofa {

namespace component {

namespace linearsolver {

template<class TMatrix, class TVector, class TThreadManager>
SparseLDLSolver<TMatrix,TVector,TThreadManager>::SparseLDLSolver()
    : numStep(0)
    , f_saveMatrixToFile( initData(&f_saveMatrixToFile, false, "saveMatrixToFile", "save matrix to a text file (can be very slow, as full matrix is stored"))
{}

template<class TMatrix, class TVector, class TThreadManager>
void SparseLDLSolver<TMatrix,TVector,TThreadManager>::solve (Matrix& M, Vector& z, Vector& r) {
    Inherit::solve_cpu(&z[0],&r[0],(InvertData *) this->getMatrixInvertData(&M));
}

template<class TMatrix, class TVector, class TThreadManager>
void SparseLDLSolver<TMatrix,TVector,TThreadManager>::solveSystem()
{
    core::ConstraintParams cparams = core::ConstraintParams();
    cparams.setAssembleConstitutiveConstraints(true);
    cparams.setOrder(core::ConstraintParams::ConstOrder::VEL);

    std::size_t numConstraints = 0;

    core::objectmodel::BaseContext* context = this->getContext();

    // ////////////////////////////////////////////////////////////////////
    // get the constraint matrices at the independant mechanical states

    sofa::helper::AdvancedTimer::stepBegin("Reset constitutive constraints");

    // mechanical action executed from root node to propagate the constraints
    simulation::MechanicalResetConstraintVisitor(&cparams).execute(context);

    sofa::helper::AdvancedTimer::stepNext("Reset constitutive constraints", "Build constitutive constraints");

    // execute "constraint::buildConstraintMatrix() for all the constraints selected for factorization
    simulation::MechanicalBuildConstraintMatrix(&cparams, cparams.j(), numConstraints).execute(context);

    if(numConstraints == std::size_t(0))
    {
        Inherit::solveSystem();
        return;
    }

    sofa::helper::AdvancedTimer::stepNext("Build constitutive constraints", "Accumulate matrix of constitutive constraints");

    // execute "mapping::applyJT()" for all the mappings
    const bool reverseAccumulateOrder = false;
    simulation::MechanicalAccumulateMatrixDeriv(&cparams, cparams.j(), reverseAccumulateOrder).execute(context);

    sofa::helper::AdvancedTimer::stepNext("Accumulate matrix of constitutive constraints", "Project Jacobian of constitutive constraints");

    //// suppress the constraints that are on DOFS currently concerned by projective constraint
    core::MechanicalParams mparams = core::MechanicalParams(cparams);
    simulation::MechanicalProjectJacobianMatrixVisitor(&mparams).execute(context);

    sofa::helper::AdvancedTimer::stepEnd("Project Jacobian of constitutive constraints");
        
    // ////////////////////////////////////////////////////////////////////
    // accumulate matrix

    buildConstitutiveConstraintsJMatrix(&cparams, numConstraints);
    
    buildConstitutiveConstraintsSystemMatrix(&cparams);
    
    buildConstitutiveConstraintsHVectors(&cparams);
    
    Inherit::solveSystem();
}


template<class TMatrix, class TVector, class TThreadManager>
void SparseLDLSolver<TMatrix, TVector, TThreadManager>::buildConstitutiveConstraintsJMatrix(const sofa::core::ConstraintParams* cparams, std::size_t numConstitutiveConstraints)
{
    JMatrixType * j_local = internalData.getLocalJ();
    j_local->clear();
    j_local->resize(numConstitutiveConstraints, currentGroup->systemMatrix->colSize());

    if (numConstitutiveConstraints == 0)
    {
        return;
    }

    executeVisitor(simulation::MechanicalGetConstraintJacobianVisitor(cparams, j_local));

}


template<class TMatrix, class TVector, class TThreadManager>
void SparseLDLSolver<TMatrix, TVector, TThreadManager>::buildConstitutiveConstraintsSystemMatrix(const sofa::core::ConstraintParams* cparams)
{
    const JMatrixType * j_local = internalData.getLocalJ();

    std::size_t mechanicalSystemSize = currentGroup->systemMatrix->colSize();
    std::size_t constitutiveConstraintsSystemSize = j_local->rowSize() + mechanicalSystemSize;

    currentGroup->systemSize = constitutiveConstraintsSystemSize;

    currentGroup->systemMatrix->extend(constitutiveConstraintsSystemSize,constitutiveConstraintsSystemSize);

    for(auto& line : *j_local)
    {    
        std::size_t rowIndex = line.first;
        for(auto& elem : line.second)
        {
            std::size_t colIndex = elem.first;
            const double& value = elem.second;

            if(value == double(0))
            {
                continue;
            }
            currentGroup->systemMatrix->add(rowIndex + mechanicalSystemSize, colIndex, value);
            currentGroup->systemMatrix->add(colIndex, rowIndex + mechanicalSystemSize, value);
        }
    }

    currentGroup->needInvert = true;
}

template<class TMatrix, class TVector, class TThreadManager>
void SparseLDLSolver<TMatrix, TVector, TThreadManager>::buildConstitutiveConstraintsHVectors(const core::ConstraintParams* cparams)
{ 
    currentGroup->systemRHVector->extend(currentGroup->systemSize);
    currentGroup->systemLHVector->extend(currentGroup->systemSize);

    const JMatrixType * j_local = internalData.getLocalJ();
    std::size_t nbConstraints = j_local->rowSize();
    TVector violation;
    violation.resize(nbConstraints);
    component::constraintset::MechanicalGetConstraintViolationVisitor(cparams, &violation).execute(this->getContext());

    for (std::size_t i = 0; i < nbConstraints; ++i)
    {
        double value = violation.element(i);
        currentGroup->systemRHVector->set(currentGroup->systemSize - nbConstraints + i, -value);
    }
}




template<class TMatrix, class TVector, class TThreadManager>
void SparseLDLSolver<TMatrix,TVector,TThreadManager>::invert(Matrix& M) {
    if (f_saveMatrixToFile.getValue()) {
        std::ofstream f;
        char name[100];
        sprintf(name, "LDL_matrix_%04d.txt", numStep);
        f.open(name);
        f << M;
        f.close();
    }

    Mfiltered.copyNonZeros(M);
    Mfiltered.compress();

    int n = M.colSize();

    int * M_colptr = (int *) Mfiltered.getRowBegin().data();
    int * M_rowind = (int *) Mfiltered.getColsIndex().data();
    Real * M_values = (Real *) Mfiltered.getColsValue().data();

    InvertData * data = (InvertData *) this->getMatrixInvertData(&M);
    Inherit::factorize(n,M_colptr,M_rowind,M_values,data);

    if (f_saveMatrixToFile.getValue()) {
        { // L
            std::ofstream f;
            char name[100];
            sprintf(name, "LDL_Lcsr_%04d.txt", numStep);
            f.open(name);
            f << data->n << std::endl;
            f << data->L_colptr << std::endl;
            f << data->L_rowind << std::endl;
            f << data->L_values << std::endl;
            f << std::endl;
            f.close();
        }
        { // Dinv
            std::ofstream f;
            char name[100];
            sprintf(name, "LDL_Dinv_%04d.txt", numStep);
            f.open(name);
            f << data->n << std::endl;
            f << data->invD << std::endl;
            f << std::endl;
            f.close();
        }
        { // Perm
            std::ofstream f;
            char name[100];
            sprintf(name, "LDL_Perm_%04d.txt", numStep);
            f.open(name);
            f << data->perm << std::endl;
            f << data->invperm << std::endl;
            f << std::endl;
            f.close();
        }
        { // M pattern
            std::ofstream f;
            char name[100];
            sprintf(name, "LDL_Mpattern_%04d.txt", numStep);
            f.open(name);
            const int nbRow = Mfiltered.rowSize();
            const int nbCol = Mfiltered.colSize();
            for (int r = 0; r < nbRow; ++r)
            {
                const int rBegin = M_colptr[r];
                const int rEnd = M_colptr[r+1];
                int c = 0;
                f << '[';
                for (int it = rBegin; it != rEnd; ++it)
                {
                    int col = M_rowind[it];
                    for (;c < col; ++c) f << ' ';
                    f << 'x'; ++c;
                }
                for (;c < nbCol; ++c) f << ' ';
                f << ']';
                f << std::endl;
            }
            f << std::endl;
            f.close();
        }
        { // L pattern
            std::ofstream f;
            char name[100];
            sprintf(name, "LDL_Lpattern_%04d.txt", numStep);
            f.open(name);
            const int nbRow = Mfiltered.rowSize();
            const int nbCol = Mfiltered.colSize();
            for (int r = 0; r < nbRow; ++r)
            {
                const int rBegin = data->L_colptr[r];
                const int rEnd = data->L_colptr[r+1];
                int c = 0;
                f << '[';
                for (int it = rBegin; it != rEnd; ++it)
                {
                    int col = data->L_rowind[it];
                    for (;c < col; ++c) f << ' ';
                    f << 'x'; c++;
                }
                for (;c < nbCol; ++c) f << ' ';
                f << ']';
                f << std::endl;
            }
            f << std::endl;
            f.close();
        }
    }

    numStep++;
}

/// Default implementation of Multiply the inverse of the system matrix by the transpose of the given matrix, and multiply the result with the given matrix J
template<class TMatrix, class TVector, class TThreadManager>
bool SparseLDLSolver<TMatrix,TVector,TThreadManager>::addJMInvJtLocal(TMatrix * M, ResMatrixType * result,const JMatrixType * J, double fact) {
    if (J->rowSize()==0) return true;

    Jlocal2global.clear();
    for (typename SparseMatrix<Real>::LineConstIterator jit = J->begin() , jitend = J->end(); jit != jitend; ++jit) {
        int l = jit->first;
        Jlocal2global.push_back(l);
    }

    if (Jlocal2global.empty()) return true;

    const unsigned int JlocalRowSize = (unsigned int)Jlocal2global.size();

    InvertData * data = (InvertData *) this->getMatrixInvertData(M);

    JLinv.clear();
    JLinv.resize(J->rowSize(),data->n);
    JLinvDinv.resize(J->rowSize(),data->n);

    unsigned int localRow = 0;
    for (typename SparseMatrix<Real>::LineConstIterator jit = J->begin() , jitend = J->end(); jit != jitend; ++jit, ++localRow) {
        Real * line = JLinv[localRow];
        for (typename SparseMatrix<Real>::LElementConstIterator it = jit->second.begin(), i2end = jit->second.end(); it != i2end; ++it) {
            int col = data->invperm[it->first];
            double val = it->second;

            line[col] = (Real)val;
        }
    }

    //Solve the lower triangular system
    for (unsigned c=0;c<JlocalRowSize;c++) {
        Real * line = JLinv[c];

        for (int j=0; j<data->n; j++) {
            for (int p = data->LT_colptr[j] ; p<data->LT_colptr[j+1] ; p++) {
                int col = data->LT_rowind[p];
                double val = data->LT_values[p];
                line[j] -= (Real)(val * line[col]);
            }
        }
    }

    //apply diagonal
    for (unsigned j=0; j<JlocalRowSize; j++) {
        Real * lineD = JLinv[j];
        Real * lineM = JLinvDinv[j];
        for (unsigned i=0;i<(unsigned)data->n;i++) {
            lineM[i] = lineD[i] * data->invD[i];
        }
    }

    for (unsigned j=0; j<JlocalRowSize; j++) {
        Real * lineJ = JLinvDinv[j];
        int globalRowJ = Jlocal2global[j];
        for (unsigned i=j;i<JlocalRowSize;i++) {
            Real * lineI = JLinv[i];
            int globalRowI = Jlocal2global[i];

            double acc = 0.0;
            for (unsigned k=0;k<(unsigned)data->n;k++) {
                acc += lineJ[k] * lineI[k];
            }
            acc *= fact;
            result->add(globalRowJ,globalRowI,acc);
            if(globalRowI!=globalRowJ) result->add(globalRowI,globalRowJ,acc);
        }
    }

    return true;
}

} // namespace linearsolver

} // namespace component

} // namespace sofa

#endif
