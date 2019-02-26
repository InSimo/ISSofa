#include <gtest/gtest.h>

#include <sofa/defaulttype/MapMapSparseMatrix.h>

#include <sofa/defaulttype/Vec.h>

#ifndef SOFA_ASSERT
#define SOFA_ASSERT(condition)
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                                                        ///
///                               Many part of test of MapMapSparseMatrix are in ConstraintSparseMatrix_test                               ///
///                                Remaining tests are thoses related to subscript operator and ColIterator                                ///
///                                                                                                                                        ///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace
{

	typedef sofa::defaulttype::MapMapSparseMatrix<sofa::defaulttype::Vec3d>	Matrix;

    namespace TestHelpers
    {

        //////////////////////////////////////////////////
	    struct line_t
	    {
		    Matrix::KeyType rowIndex;

		    struct Data
		    {
			    Matrix::KeyType index;
			    Matrix::Data value;
		    } data1, data2, data3;

            static const unsigned int initialDataCount = 3;
	    };

	    line_t nullLine()
	    {
            line_t line = { 0, { 0, Matrix::Data() }, { 0, Matrix::Data() }, { 0, Matrix::Data() } };
            return line;
	    }

	    //////////////////////////////////////////////////
        const line_t Populate(Matrix& matrix)
	    {
		    line_t result = nullLine();

		    result.rowIndex = 42;

		    Matrix::RowIterator itRow = matrix.writeLine(result.rowIndex);

		    {
			    result.data1.index = 123;
			    result.data1.value = Matrix::Data(1, 2, 3);
			    itRow.setCol(result.data1.index, result.data1.value);
		    }

		    {
			    result.data2.index = 456;
			    result.data2.value = Matrix::Data(4, 5, 6);
			    itRow.setCol(result.data2.index, result.data2.value);
		    }

		    return result;
	    }

        //////////////////////////////////////////////////
        const line_t WriteLine(Matrix& matrix, Matrix::KeyType rowIndex, Matrix::KeyType startColIndex = std::numeric_limits<Matrix::KeyType>::max())
	    {
		    if (startColIndex == std::numeric_limits<Matrix::KeyType>::max()) {
                startColIndex = rowIndex + 1;
            }

            line_t result = nullLine();

		    result.rowIndex = rowIndex;

		    Matrix::RowIterator itRow = matrix.writeLine(result.rowIndex);

		    {
			    result.data1.index = startColIndex;
			    result.data1.value = Matrix::Data(startColIndex + 1, startColIndex + 2, startColIndex + 3);
			    itRow.setCol(result.data1.index, result.data1.value);
		    }

		    {
			    result.data2.index = startColIndex + 4;
			    result.data2.value = Matrix::Data(startColIndex + 5, startColIndex + 6, startColIndex + 7);
			    itRow.setCol(result.data2.index, result.data2.value);
		    }

		    {
			    result.data3.index = startColIndex + 8;
			    result.data3.value = Matrix::Data(startColIndex + 9, startColIndex + 10, startColIndex + 11);
			    itRow.setCol(result.data3.index, result.data3.value);
		    }

		    return result;
	    }

        //////////////////////////////////////////////////
        Matrix::KeyType GetNextUniqueIndex(Matrix::KeyType rowIndex)
        {
            return rowIndex + 13;
        }

    } // namespace TestHelpers

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatTheBeginColIteratorObtainedFromAnEmptyRowConstIteratorIsEquivalentItsThePastTheEndColIterator)
    {
        Matrix matrix;

        {
            matrix.writeLine(123);
            matrix.writeLine(456);
            matrix.writeLine(789);
        }

        Matrix::RowConstIterator itRow = matrix.readLine(456);
        EXPECT_EQ(itRow.end(), itRow.begin());
    }

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatTheBeginColIteratorObtainedFromARowIteratorIsEquivalentToItsPastTheEndColIterator)
    {
        Matrix matrix;

        {
            matrix.writeLine(123);
            matrix.writeLine(456);
            matrix.writeLine(789);
        }

        Matrix::RowIterator itRow = matrix.writeLine(456);
        EXPECT_EQ(itRow.end(), itRow.begin());
    }

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatPreIncrementingSizeTimesTheIteratorToTheBeginningOfAMatrixResultsInThePastTheEndIterator)
	{
		Matrix matrix;

        {
            matrix.writeLine(123);
            matrix.writeLine(456);
            matrix.writeLine(789);
        }

		Matrix::RowIterator itRow = matrix.begin();
		++itRow; ++itRow; ++itRow;
		EXPECT_EQ(matrix.end(), itRow);
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatPostIncrementingSizeTimesTheIteratorToTheBeginningOfAMatrixResultsInThePastTheEndIterator)
	{
		Matrix matrix;

        {
            matrix.writeLine(123);
            matrix.writeLine(456);
            matrix.writeLine(789);
        }

		Matrix::RowIterator itRow = matrix.begin();
		itRow++; itRow++; itRow++;
		EXPECT_EQ(matrix.end(), itRow);
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatALineCanBeReadAndItsDataRetrievedByUsingARowInternalIterator)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

		Matrix::RowIterator itRow = matrix.begin();
		Matrix::RowType row = itRow.row();

		{
			Matrix::RowType::const_iterator itData = row.find(line1.data1.index);
			EXPECT_EQ(line1.data1.index, itData->first);
			Matrix::Data rowData = itData->second;
			EXPECT_EQ(line1.data1.value, rowData);
		}

		{
			Matrix::RowType::const_iterator itData = row.find(line1.data2.index);
			EXPECT_EQ(line1.data2.index, itData->first);
			Matrix::Data rowData = itData->second;
			EXPECT_EQ(line1.data2.value, rowData);
		}
	}

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatTheSubscriptOperatorOnARowReturnsTheExpectedData)
    {
        Matrix matrix;

        TestHelpers::line_t line1 = TestHelpers::WriteLine(matrix, 23);
        TestHelpers::line_t line2 = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line1.rowIndex));

        {
            // [TODO] Matrix::RowIterator& itRow = matrix.writeLine(line1.rowIndex);
            Matrix::RowIterator itRow = matrix.writeLine(line1.rowIndex); //[FIX]
            Matrix::RowType& row = itRow.row();

            SOFA_ASSERT(line1.initialDataCount == 3);
            EXPECT_EQ(line1.data1.value, row[line1.data1.index]);
            EXPECT_EQ(line1.data2.value, row[line1.data2.index]);
            EXPECT_EQ(line1.data3.value, row[line1.data3.index]);
        }

        {
            // [TODO] Matrix::RowIterator& itRow = matrix.writeLine(line2.rowIndex);
            Matrix::RowIterator itRow = matrix.writeLine(line2.rowIndex); // [FIX]
            Matrix::RowType& row = itRow.row();

            SOFA_ASSERT(line2.initialDataCount == 3);
            EXPECT_EQ(line2.data1.value, row[line2.data1.index]);
            EXPECT_EQ(line2.data2.value, row[line2.data2.index]);
            EXPECT_EQ(line2.data3.value, row[line2.data3.index]);
        }
    }

    // [TODO] delete?
    //////////////////////////////////////////////////
	//TEST(SparseMatrixTest, CheckThatTheSubscriptOperatorOfAConstRowInternalReturnsTheExpectedValue)
	//{
	//	Matrix matrix;
	//
	//	const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

	//	Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);
	//	const Matrix::RowType row = itRow.row();
	//
	//	EXPECT_EQ(line1.data1.value, row[line1.data1.index]);
	//}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheSubscriptOperatorOfARowInternalReturnsAWritableReferenceOnAnExistingValue)
	{
		// [WIP]

        Matrix matrix;

        const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

		Matrix::RowIterator itRow = matrix.writeLine(line1.rowIndex);
		Matrix::RowType row = itRow.row();

        const Matrix::KeyType colIndex = line1.data2.index;
		row[colIndex] = Matrix::Data(7, 8, 9);
        EXPECT_EQ(Matrix::Data(7, 8, 9), row.find(colIndex)->second);
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheSubscriptOperatorOfARowInternalCreatesAnElementIfItDoesntExist)
	{
		Matrix matrix;

        const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

		Matrix::RowIterator itRow = matrix.writeLine(line1.rowIndex + 1);
		Matrix::RowType row = itRow.row();

		const Matrix::KeyType colIndex = line1.data2.index + 1;
        row[colIndex];
        EXPECT_EQ(Matrix::Data(), row.find(colIndex)->second);
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheSubscriptOperatorOfARowInternalCreatesAnElementIfItDoesntExistAndReturnsAWritableReference)
	{
		Matrix matrix;

        const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

		Matrix::RowIterator itRow = matrix.writeLine(line1.rowIndex + 1);
		Matrix::RowType row = itRow.row();

        const Matrix::KeyType colIndex = line1.data2.index + 1;
		row[colIndex] = Matrix::Data(7, 8, 9);
        EXPECT_EQ(Matrix::Data(7, 8, 9), row.find(colIndex)->second);
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheSizeOfALineIsSetToZeroWhenItIsCleared)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

        {
            const Matrix::KeyType rowIndex2 = line1.rowIndex + 1;
            Matrix::RowIterator itRow = matrix.writeLine(rowIndex2);
			itRow.setCol(789, Matrix::Data(7, 8, 9));
            itRow.setCol(852, Matrix::Data(8, 5, 2));
        }

        Matrix::RowIterator itRow = matrix.writeLine(line1.rowIndex);
		Matrix::RowType& row = itRow.row();

		row.clear();
        EXPECT_EQ(0u, itRow.row().size());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheLineCountStaysUnchangedWhenALineIsCleared)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);
        TestHelpers::WriteLine(matrix, line1.rowIndex + 1);

        Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);
		Matrix::RowType row = itRow.row();

		const Matrix::KeyType expectedRowCount = matrix.size();
        row.clear();
        EXPECT_EQ(expectedRowCount, matrix.size());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheSizeOfALineIsZeroAfterAllItsElementsHaveBeenErased)
	{
		Matrix matrix;

		const Matrix::KeyType rowIndex = 42;

        const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, rowIndex);

        Matrix::RowIterator itRow = matrix.writeLine(line.rowIndex);

        Matrix::RowType& row = itRow.row();
        //const Matrix::KeyType expectedSize = itRow.row().size() - 1;

        {
            SOFA_ASSERT(line.initialDataCount == 3);
            row.erase(line.data1.index);
            row.erase(line.data2.index);
            row.erase(line.data3.index);
        }

        EXPECT_EQ(0u, itRow.row().size());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheColumnIndexOfAColIteratorCanBeRetrieved)
	{
		Matrix matrix;

		const Matrix::KeyType rowIndex = 42;

		// Populate matrix
		{
			Matrix::RowIterator itRow = matrix.writeLine(rowIndex);
			itRow.setCol(123, Matrix::Data(1, 2, 3));
		}

        Matrix::RowIterator itRow = matrix.writeLine(rowIndex);
		Matrix::ColIterator itData = itRow.begin();
        EXPECT_EQ(123u, itData.index());
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheValueOfADataCanBeChangedViaAColIterator)
	{
		Matrix matrix;

		const Matrix::KeyType rowIndex = 42;

		// Populate matrix
		{
			Matrix::RowIterator itRow = matrix.writeLine(rowIndex);
			itRow.setCol(123, Matrix::Data(1, 2, 3));
		}

		Matrix::RowIterator itRow = matrix.writeLine(rowIndex);
		Matrix::ColIterator itData = itRow.begin();
        itData.val() = Matrix::Data(4, 5, 6);
		EXPECT_EQ(Matrix::Data(4, 5, 6), itData.val());
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatPreIncrementingSizeTimesTheColIteratorToTheBeginningOfARowResultsInThePastTheEndIterator)
	{
		Matrix matrix;
		TestHelpers::Populate(matrix);

		Matrix::RowIterator itRow = matrix.begin();

		Matrix::ColIterator itData = itRow.begin();
		++itData; ++itData;
		EXPECT_EQ(itRow.end(), itData);
	}

    //////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatPostIncrementingSizeTimesTheColIteratorToTheBeginningOfARowResultsInThePastTheEndIterator)
    {
        Matrix matrix;
        TestHelpers::Populate(matrix);

        Matrix::RowIterator itRow = matrix.begin();

        Matrix::ColIterator itData = itRow.begin();
        itData++; itData++;
        EXPECT_EQ(itRow.end(), itData);
    }

}
