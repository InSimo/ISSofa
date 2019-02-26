#include <gtest/gtest.h>
#include <sofa/defaulttype/CompressedRowSparseMatrixConstraintEigenUtils.h>
#include <Eigen/Sparse>
#include <sofa/defaulttype/Vec3Types.h>

namespace
{


TEST(CompressedRowSparseMatrixEigenUtilsTest, checkEigenSparseMatrixLowLeveAPI)
{
    typedef Eigen::SparseMatrix<double, Eigen::RowMajor> EigenSparseMatrix;

    EigenSparseMatrix mat(5,5);

    std::vector< Eigen::Triplet<double> > matEntries =
    {
        {0,0,0}, {0,1,3},
        {1,0,22}, {1,4,17},
        {2,0,5}, {2,1,5}, {2,4,1},
        {4,2,14}, {4,4,8}
    };

    mat.setFromTriplets(matEntries.begin(), matEntries.end());

    mat.makeCompressed();

    int*    outerIndexPtr = mat.outerIndexPtr();
    int*    innerIndexPtr = mat.innerIndexPtr();
    double* valuePtr = mat.valuePtr();


    int nonZero_0 = *(outerIndexPtr + 0+1) - *(outerIndexPtr+0);
    EXPECT_EQ(2, nonZero_0);
    
    int nonZero_1 = *(outerIndexPtr + 1+1) - *(outerIndexPtr+1);
    EXPECT_EQ(2, nonZero_1);

    int nonZero_2 = *(outerIndexPtr + 2 + 1) - *(outerIndexPtr + 2);
    EXPECT_EQ(3, nonZero_2);

    int nonZero_3 = *(outerIndexPtr + 3 + 1) - *(outerIndexPtr + 3);
    EXPECT_EQ(0, nonZero_3);

    int nonZero_4 = *(outerIndexPtr + 4 + 1) - *(outerIndexPtr + 4);
    EXPECT_EQ(2, nonZero_4);

    EXPECT_EQ(0,*(innerIndexPtr + 0));
    EXPECT_EQ(1,*(innerIndexPtr + 1));
    EXPECT_EQ(0,*(innerIndexPtr + 2));
    EXPECT_EQ(4,*(innerIndexPtr + 3));
    EXPECT_EQ(0,*(innerIndexPtr + 4));
    EXPECT_EQ(1,*(innerIndexPtr + 5));
    EXPECT_EQ(4,*(innerIndexPtr + 6));
    EXPECT_EQ(2,*(innerIndexPtr + 7));
    EXPECT_EQ(4,*(innerIndexPtr + 8));

    EXPECT_EQ((int)matEntries.size(), (int)mat.nonZeros());
    
    for (int i = 0; i < mat.nonZeros(); ++i)
    {
        double value = *(valuePtr + i);
        EXPECT_EQ(matEntries[i].value(), value);
    }
}


TEST(CompressedRowSparseMatrixEigenUtilsTest, checkConversionEigenSparseCompressedRowSparseVec1d)
{
    typedef sofa::defaulttype::Vec<1, double > TVec;

    typedef sofa::defaulttype::EigenSparseToCompressedRowSparseMatrix< TVec > EigenSparseToCompressedRowSparseVec1d;
    EigenSparseToCompressedRowSparseVec1d eigenSparseToCompressedRowSparseVec1d;
    typedef typename EigenSparseToCompressedRowSparseVec1d::EigenSparseMatrix EigenSparseMatrix;
    typedef typename EigenSparseToCompressedRowSparseVec1d::TCompressedRowSparseMatrix TCompressedRowSparseMatrix;

    std::vector< Eigen::Triplet<double> > matEntries =
    {
        { 0,0,7 },{ 0,1,3 },
        { 1,0,22 },{ 1,4,17 },
        { 2,0,5 },{ 2,1,5 },{ 2,4,1 },
        { 4,2,14 },{ 4,4,8 }
    };

    EigenSparseMatrix eigenMat(5,5);
    eigenMat.setFromTriplets(matEntries.begin(),matEntries.end() );
    eigenMat.finalize();

    TCompressedRowSparseMatrix mat = eigenSparseToCompressedRowSparseVec1d(eigenMat);
    mat.compress();

    int indexEntry = 0;
    for (auto row = mat.begin(); row != mat.end(); ++row)
    {
        for (auto col = row.begin(); col != row.end(); ++col)
        {
            EXPECT_EQ( matEntries[indexEntry++].value(), col.val()[0]  );
        }
    }
    EXPECT_EQ((int)matEntries.size(), indexEntry);
}


TEST(CompressedRowSparseMatrixEigenUtilsTest, checkConversionEigenSparseCompressedRowSparseVec3d)
{
    typedef  sofa::defaulttype::Vec<3, double > TVec;
    typedef sofa::defaulttype::EigenSparseToCompressedRowSparseMatrix< TVec > EigenSparseToCompressedRowSparseVec3d;
    EigenSparseToCompressedRowSparseVec3d eigenSparseToCompressedRowSparseVec3d;
    typedef typename EigenSparseToCompressedRowSparseVec3d::EigenSparseMatrix EigenSparseMatrix;
    typedef typename EigenSparseToCompressedRowSparseVec3d::TCompressedRowSparseMatrix TCompressedRowSparseMatrix;


    EigenSparseMatrix eigenMat(12, 12);
    std::vector< Eigen::Triplet<double> > matEntries =
    {
        { 3,3,0.1 },{ 3,4,0.2 },{ 3,5,0.3 }
    };

    eigenMat.setFromTriplets(matEntries.begin(), matEntries.end());
    eigenMat.finalize();

    TCompressedRowSparseMatrix mat = eigenSparseToCompressedRowSparseVec3d(eigenMat);
    mat.compress();

    int indexEntry = 0;
    for (auto row = mat.begin(); row != mat.end(); ++row)
    {
        for (auto col = row.begin(); col != row.end(); ++col)
        {
            for (std::size_t i = 0; i < TVec::total_size; ++i)
            {
                EXPECT_EQ(matEntries[indexEntry++].value(), col.val()[i]);
            }
        }
    }

    EXPECT_EQ((int)matEntries.size(), indexEntry);
}


TEST(CompressedRowSparseMatrixEigenUtilsTest, checkConversionCompressedRowSparseVec1dEigenSparse)
{
    typedef sofa::defaulttype::Vec<1, double > TVec;
    typedef sofa::defaulttype::CompressedRowSparseMatrixToEigenSparse< TVec > CompressedRowSparseMatrixToEigenSparseVec1d;
    CompressedRowSparseMatrixToEigenSparseVec1d compressedRowSparseToEigenSparse;
    typedef typename CompressedRowSparseMatrixToEigenSparseVec1d::EigenSparseMatrix EigenSparseMatrix;
    typedef typename CompressedRowSparseMatrixToEigenSparseVec1d::TCompressedRowSparseMatrix TCompressedRowSparseMatrix;

    TCompressedRowSparseMatrix mat;

    std::vector< Eigen::Triplet<double> > matEntries =
    {
        { 0,0,1 },{ 0,1,3 },
        { 1,0,22 },{ 1,4,17 },
        { 2,0,5 },{ 2,1,5 },{ 2,4,1 },
        { 4,2,14 },{ 4,4,8 }
    };

    auto it = matEntries.begin();

    while(it != matEntries.end())
    {
        int row = it->row();
        int col = it->col();
        TVec vec;
        for (std::size_t i = 0; i < TVec::size(); ++i)
        {
            vec[i] = it->value();
            ++it;
        }

        mat.writeLine(row).addCol(col,vec);
    }

    EigenSparseMatrix eigenMat = compressedRowSparseToEigenSparse(mat, 5);

    for (auto row = mat.begin(); row != mat.end(); ++row)
    {
        for (auto col = row.begin(); col != row.end(); ++col)
        {
            for (std::size_t i = 0; i < TVec::size(); ++i)
            {
                EXPECT_EQ(col.val()[i], eigenMat.coeff(row.index(), col.index()+i));
            }
        }
    }

    EXPECT_EQ(matEntries.size(), (std::size_t)eigenMat.nonZeros());
}


TEST(CompressedRowSparseMatrixEigenUtilsTest, checkConversionCompressedRowSparseVec3dEigenSparse)
{
    typedef sofa::defaulttype::Vec<1, double > TVec;
    typedef sofa::defaulttype::CompressedRowSparseMatrixToEigenSparse< TVec > checkConversionCompressedRowSparseVec3dEigenSparse;
    checkConversionCompressedRowSparseVec3dEigenSparse compressedRowSparseToEigenSparse;
    typedef typename checkConversionCompressedRowSparseVec3dEigenSparse::EigenSparseMatrix EigenSparseMatrix;
    typedef typename checkConversionCompressedRowSparseVec3dEigenSparse::TCompressedRowSparseMatrix TCompressedRowSparseMatrix;

    TCompressedRowSparseMatrix mat;

    std::vector< Eigen::Triplet<double> > matEntries =
    {
        { 3,3,0.1 },{ 3,4,0.2 },{ 3,5,0.3 }
    };


    auto it = matEntries.begin();

    while (it != matEntries.end())
    {
        int row = it->row();
        int col = it->col();
        TVec vec;
        for (std::size_t i = 0; i < TVec::size(); ++i)
        {
            vec[i] = it->value();
            ++it;
        }

        mat.writeLine(row).addCol(col, vec);
    }

    EigenSparseMatrix eigenMat = compressedRowSparseToEigenSparse(mat, 12);

    for (auto row = mat.begin(); row != mat.end(); ++row)
    {
        for (auto col = row.begin(); col != row.end(); ++col)
        {
            for (std::size_t i = 0; i < TVec::size(); ++i)
            {
                EXPECT_EQ(col.val()[i], eigenMat.coeff(row.index(), col.index()+i));
            }
        }
    }

    EXPECT_EQ(matEntries.size(), (std::size_t)eigenMat.nonZeros());
}


TEST(CompressedRowSparseMatrixEigenUtilsTest, checkAddMultTransposeEigenForCumulativeWrite)
{
    typedef Eigen::SparseMatrix<double, Eigen::RowMajor> EigenSparseMatrix;

    EigenSparseMatrix jacobian(3, 6); // as in rigid mapping block

    std::vector< Eigen::Triplet<double> > matEntries =
    {
        { 0,0,1 }, { 1,1,1 }, {2,2,1}, // identity block

        { 0,4,2 }, {0, 5, -1}, // skew symmetric block
        { 1,3,-2}, {1, 5, -4},
        { 2,3,1},  {2,4,4}
    };

    jacobian.setFromTriplets(matEntries.begin(), matEntries.end());
    jacobian.makeCompressed();


    //std::cout << jacobian << std::endl;

    typedef sofa::defaulttype::Rigid3Types::Deriv RigidDeriv;
    typedef sofa::defaulttype::Vec3Types::MatrixDeriv RhsType;
    typedef sofa::defaulttype::Rigid3Types::MatrixDeriv LhsType;

    LhsType lhs;

    {
        RhsType rhs;
        auto col = rhs.writeLine(0);
        col.addCol(0, sofa::defaulttype::Vec3d(1, 0, 0));
        sofa::defaulttype::addMultTransposeEigen(lhs, jacobian, rhs);
    }

    {
        RhsType rhs;
        auto col = rhs.writeLine(1);
        col.addCol(0, sofa::defaulttype::Vec3d(0, 1, 0));
        sofa::defaulttype::addMultTransposeEigen(lhs, jacobian, rhs);
    }

    {
        RhsType rhs;
        auto col = rhs.writeLine(2);
        col.addCol(0, sofa::defaulttype::Vec3d(0, 0, 1));
        sofa::defaulttype::addMultTransposeEigen(lhs, jacobian, rhs);
    }

    std::cout << lhs << std::endl;

    for (auto row = lhs.begin(), rowEnd = lhs.end(); row != rowEnd; ++row)
    {
        for (auto col = row.begin(), colEnd = row.end(); col != colEnd; ++col)
        {
            for (std::size_t i = 0; i < RigidDeriv::total_size; ++i)
            {
                EXPECT_EQ(col.val()[i], jacobian.coeff(row.index(), col.index()*RigidDeriv::total_size + i ));
            }
        }

    }
}


}
