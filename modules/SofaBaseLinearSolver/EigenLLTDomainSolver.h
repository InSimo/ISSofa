#ifndef SOFA_COMPONENT_LINEARSOLVER_EIGENLLTDOMAINSOLVER_H
#define SOFA_COMPONENT_LINEARSOLVER_EIGENLLTDOMAINSOLVER_H


#include <sofa/SofaBase.h>
#include <sofa/defaulttype/CompressedRowSparseMatrixMechanical.h>
#include <sofa/core/behavior/LinearSolver.h>
#include <sofa/core/behavior/MechanicalState.h>
#include "DefaultMultiMatrixAccessor.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#ifdef SOFA_HAVE_METIS
#include <Eigen/MetisSupport> 
#endif


namespace sofa
{
namespace component
{
namespace linearsolver
{


template< class TDataTypes >
struct MatrixAssemblyTraits
{
    using DataTypes            = TDataTypes;
    static constexpr int BSIZE = DataTypes::deriv_total_size;
    using Block                = sofa::defaulttype::Mat<BSIZE, BSIZE, double>;
    using BSRMatrix            = sofa::defaulttype::CompressedRowSparseMatrixMechanical< Block >;
};


template<class DataTypes>
class EigenLLTDomainSolver : public sofa::core::behavior::LinearSolver
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(EigenLLTDomainSolver, DataTypes), sofa::core::behavior::LinearSolver);

    using EigenSparseMatrix = Eigen::SparseMatrix<double>;
    using EigenSparseVector = Eigen::SparseVector<double>;
#ifdef SOFA_HAVE_METIS
    using EigenSimplicialLLT = Eigen::SimplicialLLT< EigenSparseMatrix, Eigen::Lower, Eigen::MetisOrdering<EigenSparseMatrix::Index>  >;
#else
    using EigenSimplicialLLT = Eigen::SimplicialLLT< EigenSparseMatrix, Eigen::Lower>;
#endif


    using TMechanicalState    = sofa::core::behavior::MechanicalState<DataTypes>;
    using MechanicalStateLink = sofa::SingleLink<MyType, TMechanicalState, sofa::BaseLink::FLAG_STRONGLINK |
                                                                           sofa::BaseLink::FLAG_STOREPATH >;
    using AssemblyTraits      = MatrixAssemblyTraits< DataTypes >;
    using BSRMatrix           = typename AssemblyTraits::BSRMatrix;
    using Block               = typename AssemblyTraits::Block;

    using Deriv    = typename DataTypes::Deriv;
    using VecDeriv = typename DataTypes::VecDeriv;
    using MatrixDeriv = typename DataTypes::MatrixDeriv;

    using VecIndex = typename BSRMatrix::VecIndex;
    using VecBlock = typename BSRMatrix::VecBloc;

    using MatrixAccessor = DefaultMultiMatrixAccessor;
    

    void resetSystem() override;

    void setSystemMBKMatrix(const sofa::core::MechanicalParams* mparams) override;

    void setSystemMBKMatrix(int scalarSize, const VecIndex& rowIndex, const VecIndex& rowBegin, const VecIndex& colsIndex, const VecBlock& colsValue);

    void setSystemRHVector(sofa::core::MultiVecDerivId v) override;

    void setSystemLHVector(sofa::core::MultiVecDerivId v) override;

    void solveSystem() override;

    void writeSolution() override;

    void invertSystem() override;

    bool buildComplianceMatrix(const sofa::core::ConstraintParams* cparams, sofa::defaulttype::BaseMatrix* result, SReal fact) override;

    void applyConstraintForce(const sofa::core::ConstraintParams* cparams, sofa::core::MultiVecDerivId dx, const sofa::defaulttype::BaseVector* f) override;

    const sofa::core::behavior::MultiMatrixAccessor* getSystemMultiMatrixAccessor() const override;

    bool analizePattern();

    bool factorize();

    bool solve(const double* b, double* x) const;

    void solveTriangular(const MatrixDeriv& J);

    void assembleCompliance(sofa::defaulttype::BaseMatrix* result, SReal fact) const;


    template<class T>
    static bool canCreate(T*& obj, sofa::core::objectmodel::BaseContext* context, sofa::core::objectmodel::BaseObjectDescription* arg)
    {
        if (TMechanicalState::DynamicCast(context->getMechanicalState()) == nullptr)
        {
            return false;
        }
        return BaseObject::canCreate(obj, context, arg);
    }

    std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const EigenLLTDomainSolver<DataTypes>* = nullptr)
    {
        return DataTypes::Name();
    }


protected:
    EigenLLTDomainSolver();

private:

    using EigenRowMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    struct Pattern
    {
        VecIndex rowBegin;
        VecIndex colsIndex;
        int    size = -1;
    };

    struct TriangularSolve
    {
        EigenRowMatrix  LinvJ;///< LinvJ is col major, we store in each of its column the result of the triangular solve
        VecIndex        indexMap;///< array which maps a column index of LinvJ to a constraint index

        void resize(int numConstraints, int mstateMatrixSize)
        {
            LinvJ.resize( numConstraints, mstateMatrixSize);
            indexMap.resize(numConstraints);
        }

        bool empty() const
        {
            return indexMap.empty();
        }
    };


    void setSystemEigenMatrixFromMBK();

    bool needFactorizeSymbolic() const;

    MechanicalStateLink l_mstate;
    MatrixAccessor      m_matrixAccessor;
    BSRMatrix           m_MBK;
    Pattern             m_pattern;
    EigenSparseMatrix   m_matrix;
    EigenSimplicialLLT  m_decomposition;

    sofa::core::VecDerivId       m_lhsId;
    sofa::core::ConstVecDerivId  m_rhsId;
    TriangularSolve              m_triangularSolve;
    
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_BASE_LINEAR_SOLVER)
extern template class SOFA_BASE_LINEAR_SOLVER_API EigenLLTDomainSolver< sofa::defaulttype::Vec1Types >;
extern template class SOFA_BASE_LINEAR_SOLVER_API EigenLLTDomainSolver< sofa::defaulttype::Vec3Types >;
extern template class SOFA_BASE_LINEAR_SOLVER_API EigenLLTDomainSolver< sofa::defaulttype::Rigid3Types >;
#endif

}

}

}

#endif // SOFA_COMPONENT_LINEARSOLVER_EIGENLLTDOMAINSOLVER_H
