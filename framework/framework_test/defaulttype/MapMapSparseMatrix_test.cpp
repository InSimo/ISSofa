#include <gtest/gtest.h>

#include <sofa/defaulttype/MapMapSparseMatrix.h>

#include <sofa/defaulttype/Vec.h>

#ifndef SOFA_ASSERT
#define SOFA_ASSERT(condition)
#endif

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

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatASparseMatrixIsEmptyAfterInstanciation)
	{
		EXPECT_TRUE(Matrix().empty());
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheSizeOfASparseMatrixIsZeroAfterInstanciation)
	{
		EXPECT_EQ(0, Matrix().size());
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheSizeOfASparseMatrixIsOneAfterALineHasBeenWritten)
	{
		Matrix matrix;

		matrix.writeLine(42);

		EXPECT_EQ(1, matrix.size());
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheSizeOfASparseMatrixIsTwoAfterTryingToWriteTwoLinesWithDistinctIndices)
	{
		Matrix matrix;

		{
            matrix.writeLine(123);
		    matrix.writeLine(456);
        }

		EXPECT_EQ(2, matrix.size());
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheSizeOfASparseMatrixIsOneAfterTryingToWriteTwoLinesWithTheSameIndex)
	{
		Matrix matrix;

		{
            matrix.writeLine(123);
		    matrix.writeLine(123);
        }

		EXPECT_EQ(1, matrix.size());
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatAMatrixIsConsideredEmptyAfterIsHasBeenCleared)
	{
		Matrix matrix;

        {
            matrix.writeLine(123);
            matrix.writeLine(456);
            matrix.writeLine(789);
        }

		matrix.clear();
		EXPECT_TRUE(matrix.empty());
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatALineCanBeCorrectlyRetrievedByItsIndex)
	{
		Matrix matrix;

        {
            matrix.writeLine(123);
            matrix.writeLine(456);
            matrix.writeLine(789);
        }

		EXPECT_EQ(123, matrix.readLine(123).index());
		EXPECT_EQ(456, matrix.readLine(456).index());
        EXPECT_EQ(789, matrix.readLine(789).index());
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheFirstLineOfAMatrixCanBeRetrieved)
	{
		Matrix matrix;

		matrix.writeLine(42);

		EXPECT_EQ(42, matrix.begin().index());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTrueIsReturnedWhenComparingTwoRowIteratorsPointingToTheSameLine)
	{
		Matrix matrix;

		Matrix::RowIterator itRowA = matrix.writeLine(42);
		Matrix::RowIterator itRowB = itRowA;

		EXPECT_TRUE(itRowA == itRowB);
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTrueIsReturnedWhenComparingTwoRowConstIteratorsPointingToTheSameLine)
	{
		Matrix matrix;

		{
            matrix.writeLine(123);
		    matrix.writeLine(456);
        }

        Matrix::RowConstIterator itRowA = matrix.readLine(123);
		Matrix::RowConstIterator itRowB = matrix.readLine(123);

		EXPECT_TRUE(itRowA == itRowB);
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTrueIsReturnedWhenComparingTwoRowIteratorsPointingToDistinctLinesForInequality)
	{
		Matrix matrix;

		Matrix::RowIterator itRowA = matrix.writeLine(123);
		Matrix::RowIterator itRowB = matrix.writeLine(456);

		EXPECT_TRUE(itRowA != itRowB);
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTrueIsReturnedWhenComparingTwoRowConstIteratorsPointingToDistinctLinesForInequality)
	{
		Matrix matrix;

        {
            matrix.writeLine(123);
            matrix.writeLine(456);
            matrix.writeLine(789);
        }

        Matrix::RowConstIterator itRow23 = matrix.readLine(123);
		Matrix::RowConstIterator itRow42 = matrix.readLine(456);

		EXPECT_TRUE(itRow23 != itRow42);
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheBeginningAndTheEndAreTheSameWhenAMatrixIsEmpty)
	{
		Matrix matrix;

		EXPECT_EQ(matrix.end(), matrix.begin());
	}

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatTryingToClearAnEmptyMatrixLetItUnchanged)
    {
        Matrix matrix;
        matrix.clear();

        EXPECT_TRUE(matrix.empty());
        EXPECT_EQ(matrix.end(), matrix.begin());
    }

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
	TEST(SparseMatrixTest, CheckThatPreIncrementingSizeTimesTheConstIteratorToTheBeginningOfAMatrixResultsInThePastTheEndIterator)
	{
		Matrix matrix;

        {
            matrix.writeLine(123);
            matrix.writeLine(456);
            matrix.writeLine(789);
        }

		const Matrix& constMatrix = matrix;

        Matrix::RowConstIterator itRow = constMatrix.begin();
		++itRow; ++itRow; ++itRow;
		EXPECT_EQ(constMatrix.end(), itRow);
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatARowIteratorCopiedFromAnotherOneIsConsideredEqualToIt)
	{
		Matrix matrix;

         matrix.writeLine(42);

		const Matrix::RowIterator itRow = matrix.begin();
		Matrix::RowIterator itRowCopy = matrix.begin();
        itRowCopy = itRow;

        EXPECT_EQ(itRowCopy, itRow);
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatARowConstIteratorCopiedFromAnotherOneIsConsideredEqualToIt)
	{
		Matrix matrix;

        matrix.writeLine(42);

		Matrix::RowConstIterator itRow = matrix.readLine(42);
		Matrix::RowConstIterator itRowCopy = matrix.readLine(42);;
        itRowCopy = itRow;

        EXPECT_EQ(itRowCopy, itRow);
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
	TEST(SparseMatrixTest, CheckThatPostIncrementingSizeTimesTheConstIteratorToTheBeginningOfAMatrixResultsInThePastTheEndIterator)
	{
		Matrix matrix;

        {
            matrix.writeLine(123);
            matrix.writeLine(456);
            matrix.writeLine(789);
        }

		const Matrix& constMatrix = matrix;

        Matrix::RowConstIterator itRow = constMatrix.begin();
		itRow++; itRow++; itRow++;
		EXPECT_EQ(constMatrix.end(), itRow);
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatAnIteratorToALineIsReturnedWhenTryingToWriteItButItExistsAlready)
	{
		Matrix matrix;

		Matrix::RowIterator itRowA = matrix.writeLine(42);
		Matrix::RowIterator itRowB = matrix.writeLine(42);

		EXPECT_EQ(itRowB, itRowA);
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatThePastTheEndIteratorIsReturnedWhenTryingToReadALineThatDoesNotExist)
	{
		Matrix matrix;

		{
            matrix.writeLine(42);
		    matrix.writeLine(43);
        }

        const Matrix& constMatrix = matrix;

		EXPECT_EQ(constMatrix.end(), matrix.readLine(44));
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheSizeOfALineCanBeRetrieved)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

		Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);
		EXPECT_EQ(2u, itRow.row().size());
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatALineCanBeReadAndItsDataRetrievedByUsingARowInternalConstIterator)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

		Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);
		const Matrix::RowType row = itRow.row();

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
	TEST(SparseMatrixTest, CheckThatALineCanBeIdenticallyRepopulatedAfterItHasBeenClearedInAOneLineMatrix)
	{
		Matrix matrix;

		const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, 42);

        // Clear
        {
            Matrix::RowConstIterator itRow = matrix.readLine(line.rowIndex);
		    Matrix::RowType row = itRow.row();
            row.clear();
        }

        // Repopulation
        const TestHelpers::line_t newLine = TestHelpers::WriteLine(matrix, line.rowIndex);
        SOFA_ASSERT(newLine.rowIndex == line.rowIndex);

        Matrix::RowConstIterator itRow = matrix.readLine(line.rowIndex);
        Matrix::RowType row = itRow.row();

        {
            Matrix::RowType::const_iterator itData = row.find(newLine.data1.index);
            EXPECT_EQ(newLine.data1.index, itData->first);
            EXPECT_EQ(newLine.data1.value, itData->second);
        }

        {
            Matrix::RowType::const_iterator itData = row.find(newLine.data2.index);
            EXPECT_EQ(newLine.data2.index, itData->first);
            EXPECT_EQ(newLine.data2.value, itData->second);
        }
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatALineCanBeRepopulatedWithDifferentColumnsIndicesAndDataAfterItHasBeenClearedInAOneLineMatrix)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::WriteLine(matrix, 42);

        // Clear
        {
            Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);
		    Matrix::RowType row = itRow.row();
            row.clear();
        }

        // Repopulation
        const TestHelpers::line_t newLine = TestHelpers::WriteLine(matrix, line1.rowIndex, TestHelpers::GetNextUniqueIndex(line1.rowIndex));
        SOFA_ASSERT(newLine.rowIndex == line1.rowIndex);

        Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);
        Matrix::RowType row = itRow.row();

        {
            Matrix::RowType::const_iterator itData = row.find(newLine.data1.index);
            EXPECT_EQ(newLine.data1.index, itData->first);
            EXPECT_EQ(newLine.data1.value, itData->second);
        }

        {
            Matrix::RowType::const_iterator itData = row.find(newLine.data2.index);
            EXPECT_EQ(newLine.data2.index, itData->first);
            EXPECT_EQ(newLine.data2.value, itData->second);
        }
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatATotallyDifferentLineCanBeAddedAfterALineHasBeenClearedInAOneLineMatrix)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::WriteLine(matrix, 42);

        // Clear
        {
            Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);
		    Matrix::RowType row = itRow.row();
            row.clear();
        }

        // Repopulation
        const TestHelpers::line_t newLine = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line1.rowIndex));
        SOFA_ASSERT(newLine.rowIndex != line1.rowIndex);

        Matrix::RowConstIterator itRow = matrix.readLine(newLine.rowIndex);
        Matrix::RowType row = itRow.row();

        {
            Matrix::RowType::const_iterator itData = row.find(newLine.data1.index);
            EXPECT_EQ(newLine.data1.index, itData->first);
            EXPECT_EQ(newLine.data1.value, itData->second);
        }

        {
            Matrix::RowType::const_iterator itData = row.find(newLine.data2.index);
            EXPECT_EQ(newLine.data2.index, itData->first);
            EXPECT_EQ(newLine.data2.value, itData->second);
        }
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatALineCanBeIdenticallyRepopulatedAfterItHasBeenClearedInAMultiLinesMatrix)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::WriteLine(matrix, 42);
        const TestHelpers::line_t line2 = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line1.rowIndex));
        TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line2.rowIndex));

        // Clear
        {
            Matrix::RowConstIterator itRow = matrix.readLine(line2.rowIndex);
		    Matrix::RowType row = itRow.row();
            row.clear();
        }

        // Repopulation
        const TestHelpers::line_t newLine = TestHelpers::WriteLine(matrix, line2.rowIndex);
        SOFA_ASSERT(newLine.rowIndex == line2.rowIndex);

        Matrix::RowConstIterator itRow = matrix.readLine(line2.rowIndex);
        Matrix::RowType row = itRow.row();

        {
            Matrix::RowType::const_iterator itData = row.find(newLine.data1.index);
            EXPECT_EQ(newLine.data1.index, itData->first);
            EXPECT_EQ(newLine.data1.value, itData->second);
        }

        {
            Matrix::RowType::const_iterator itData = row.find(newLine.data2.index);
            EXPECT_EQ(newLine.data2.index, itData->first);
            EXPECT_EQ(newLine.data2.value, itData->second);
        }
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatALineCanBeRepopulatedWithDifferentColumnsIndicesAndDataAfterItHasBeenClearedInAMultiLinesMatrix)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::WriteLine(matrix, 42);
        const TestHelpers::line_t line2 = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line1.rowIndex));
        const TestHelpers::line_t line3 = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line2.rowIndex));

        // Clear
        {
            Matrix::RowConstIterator itRow = matrix.readLine(line2.rowIndex);
		    Matrix::RowType row = itRow.row();
            row.clear();
        }

        // Repopulation
        const TestHelpers::line_t newLine = TestHelpers::WriteLine(matrix, line2.rowIndex, TestHelpers::GetNextUniqueIndex(line3.rowIndex));
        SOFA_ASSERT(newLine.rowIndex == line2.rowIndex);

        Matrix::RowConstIterator itRow = matrix.readLine(line2.rowIndex);
        Matrix::RowType row = itRow.row();

        {
            Matrix::RowType::const_iterator itData = row.find(newLine.data1.index);
            EXPECT_EQ(newLine.data1.index, itData->first);
            EXPECT_EQ(newLine.data1.value, itData->second);
        }

        {
            Matrix::RowType::const_iterator itData = row.find(newLine.data2.index);
            EXPECT_EQ(newLine.data2.index, itData->first);
            EXPECT_EQ(newLine.data2.value, itData->second);
        }
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatATotallyDifferentLineCanBeAddedAfterALineHasBeenClearedInAMultiLineMatrix)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::WriteLine(matrix, 42);
        const TestHelpers::line_t line2 = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line1.rowIndex));
        const TestHelpers::line_t line3 = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line2.rowIndex));

        // Clear
        {
            Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);
		    Matrix::RowType row = itRow.row();
            row.clear();
        }

        // Repopulation
        const TestHelpers::line_t newLine = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line3.rowIndex));
        SOFA_ASSERT(newLine.rowIndex != line1.rowIndex);
        SOFA_ASSERT(newLine.rowIndex != line2.rowIndex);
        SOFA_ASSERT(newLine.rowIndex != line3.rowIndex);

        Matrix::RowConstIterator itRow = matrix.readLine(newLine.rowIndex);
        Matrix::RowType row = itRow.row();

        {
            Matrix::RowType::const_iterator itData = row.find(newLine.data1.index);
            EXPECT_EQ(newLine.data1.index, itData->first);
            EXPECT_EQ(newLine.data1.value, itData->second);
        }

        {
            Matrix::RowType::const_iterator itData = row.find(newLine.data2.index);
            EXPECT_EQ(newLine.data2.index, itData->first);
            EXPECT_EQ(newLine.data2.value, itData->second);
        }
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatOlderLinesStayUnaffectedWhenAMoreRecentOneIsCleared)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::WriteLine(matrix, 42);
        const TestHelpers::line_t line2 = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line1.rowIndex));
        TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line2.rowIndex));

        // Clear
        {
            Matrix::RowConstIterator itRow = matrix.readLine(line2.rowIndex);
		    Matrix::RowType row = itRow.row();
            row.clear();
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(line1.data1.index);
                EXPECT_EQ(line1.data1.index, itData->first);
                EXPECT_EQ(line1.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line1.data2.index);
                EXPECT_EQ(line1.data2.index, itData->first);
                EXPECT_EQ(line1.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line1.data3.index);
                EXPECT_EQ(line1.data3.index, itData->first);
                EXPECT_EQ(line1.data3.value, itData->second);
            }
        }
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatMoreRecentLinesStayUnaffectedWhenAnOlderOneIsCleared)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::WriteLine(matrix, 42);
        const TestHelpers::line_t line2 = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line1.rowIndex));
        const TestHelpers::line_t line3 = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(line2.rowIndex));

        // Clear
        {
            Matrix::RowConstIterator itRow = matrix.readLine(line2.rowIndex);
		    Matrix::RowType row = itRow.row();
            row.clear();
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(line3.rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(line3.data1.index);
                EXPECT_EQ(line3.data1.index, itData->first);
                EXPECT_EQ(line3.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line3.data2.index);
                EXPECT_EQ(line3.data2.index, itData->first);
                EXPECT_EQ(line3.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line3.data3.index);
                EXPECT_EQ(line3.data3.index, itData->first);
                EXPECT_EQ(line3.data3.value, itData->second);
            }
        }
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheSizeOfALineIsDecrementedByOneWhenAnElementIsErased)
	{
		Matrix matrix;

		const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, 42);

        Matrix::RowIterator itRow = matrix.writeLine(line.rowIndex);
		Matrix::RowType& row1 = itRow.row();
		Matrix::RowType& row = itRow.row();

		EXPECT_EQ(&row1, &row);

        const Matrix::KeyType expectedSize = itRow.row().size() - 1;
        row.erase(line.data2.index);
        EXPECT_EQ(expectedSize, row.size());
        EXPECT_EQ(expectedSize, itRow.row().size());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatThePastTheEndIteratorIsReturnedWhenTryingToAccessAnErasedElementInALine)
	{
		Matrix matrix;

		const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, 42);

        Matrix::RowIterator itRow = matrix.writeLine(line.rowIndex);

        {
            Matrix::RowType& row = itRow.row();
            const Matrix::KeyType expectedSize = itRow.row().size() - 1;
            row.erase(line.data3.index);
        }

        Matrix::ColIterator itData = itRow.begin();

        for (int i = 0; i < line.initialDataCount - 1; i++)  {
            ++itData;
        }

        EXPECT_EQ(itRow.end(), itData);
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatOlderElementsStayUnaffectedWhenAMoreRecentOneIsErasedFromALine)
	{
		Matrix matrix;

		const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, 42);

        Matrix::RowConstIterator itRow = matrix.readLine(line.rowIndex);

        {
            Matrix::RowType row = itRow.row();
            const Matrix::KeyType expectedSize = itRow.row().size() - 1;
            row.erase(line.data2.index);
        }

        SOFA_ASSERT(line.initialDataCount == 3);
        Matrix::ColConstIterator itData = itRow.begin();
        EXPECT_EQ(line.data1.index, itData.index());
        EXPECT_EQ(line.data1.value, itData.val());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatMoreRecentElementsStayUnaffectedWhenAnOlderOneIsErasedFromALine)
	{
		Matrix matrix;

		const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, 42);

        Matrix::RowIterator itRow = matrix.writeLine(line.rowIndex);

        {
            Matrix::RowType& row = itRow.row();
            const Matrix::KeyType expectedSize = itRow.row().size() - 1;
            row.erase(line.data2.index);
        }

        SOFA_ASSERT(line.initialDataCount == 3);
        Matrix::ColIterator itData = itRow.begin();
        ++itData;
        EXPECT_EQ(line.data3.index, itData.index());
        EXPECT_EQ(line.data3.value, itData.val());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTryingToEraseAColumnThatDoesNotExistInALineLeavesTheSizeOfTheRowUnchanged)
	{
		Matrix matrix;

		const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, 42);

        Matrix::RowConstIterator itRow = matrix.readLine(line.rowIndex);
		Matrix::RowType row = itRow.row();

        const Matrix::KeyType expectedSize = itRow.row().size();
        row.erase(line.data3.index + 1);
        EXPECT_EQ(expectedSize, itRow.row().size());

        // [TODO]
        /*sofa::defaulttype::IteratorInfo<Matrix::KeyType, Matrix::Data> iteratorInfo(matrix.m_rowInfoContainer, matrix.m_colIndices, matrix.m_dataContainer);
        sofa::defaulttype::IteratorInfo<Matrix::KeyType, Matrix::Data> iteratorInfo2(matrix.m_rowInfoContainer, matrix.m_colIndices, matrix.m_dataContainer);
        iteratorInfo = iteratorInfo2;*/
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTryingToEraseAColumnThatDoesNotExistInALineLeavesItUnchanged)
	{
		Matrix matrix;

		const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, 42);

        Matrix::RowConstIterator itRow = matrix.readLine(line.rowIndex);

        {
            Matrix::RowType row = itRow.row();
            const Matrix::KeyType expectedSize = itRow.row().size() - 1;
            row.erase(line.data3.index + 1);
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(line.rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(line.data1.index);
                EXPECT_EQ(line.data1.index, itData->first);
                EXPECT_EQ(line.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line.data2.index);
                EXPECT_EQ(line.data2.index, itData->first);
                EXPECT_EQ(line.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line.data3.index);
                EXPECT_EQ(line.data3.index, itData->first);
                EXPECT_EQ(line.data3.value, itData->second);
            }
        }
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheSizeOfALineIsZeroAfterAllItsElementsHaveBeenErased)
	{
		Matrix matrix;

		const Matrix::KeyType rowIndex = 42;

        const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, rowIndex);

        Matrix::RowIterator itRow = matrix.writeLine(line.rowIndex);

        Matrix::RowType& row = itRow.row();
        const Matrix::KeyType expectedSize = itRow.row().size() - 1;

        {
            SOFA_ASSERT(line.initialDataCount == 3);
            row.erase(line.data1.index);
            row.erase(line.data2.index);
            row.erase(line.data3.index);
        }

        EXPECT_EQ(0, itRow.row().size());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatNewElementsCanBeInsertedIntoARowAfterAllOthersHaveBeenErased)
	{
		Matrix matrix;

		const Matrix::KeyType rowIndex = 42;

        {
            const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, rowIndex);

            Matrix::RowConstIterator itRow = matrix.readLine(line.rowIndex);

            {
                Matrix::RowType row = itRow.row();

                SOFA_ASSERT(line.initialDataCount == 3);
                row.erase(line.data1.index);
                row.erase(line.data2.index);
                row.erase(line.data3.index);
            }
        }

        const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, rowIndex, TestHelpers::GetNextUniqueIndex(rowIndex));

        {
            Matrix::RowConstIterator itRow = matrix.readLine(line.rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(line.data1.index);
                EXPECT_EQ(line.data1.index, itData->first);
                EXPECT_EQ(line.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line.data2.index);
                EXPECT_EQ(line.data2.index, itData->first);
                EXPECT_EQ(line.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line.data3.index);
                EXPECT_EQ(line.data3.index, itData->first);
                EXPECT_EQ(line.data3.value, itData->second);
            }
        }
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheColumnIndexOfAColConstIteratorCanBeRetrieved)
	{
		Matrix matrix;

		const Matrix::KeyType rowIndex = 42;

		// Populate matrix
		{
			Matrix::RowIterator itRow = matrix.writeLine(rowIndex);
			itRow.setCol(123, Matrix::Data(1, 2, 3));
		}

		Matrix::RowConstIterator itRow = matrix.readLine(rowIndex);
		Matrix::ColConstIterator itData = itRow.begin();
		EXPECT_EQ(123, itData.index());
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
		EXPECT_EQ(123, itData.index());
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheValueOfADataCanBeRetrieved)
	{
		Matrix matrix;

		const Matrix::KeyType rowIndex = 42;

		// Populate matrix
		{
			Matrix::RowIterator itRow = matrix.writeLine(rowIndex);
			itRow.setCol(123, Matrix::Data(1, 2, 3));
		}

		Matrix::RowConstIterator itRow = matrix.readLine(rowIndex);
		Matrix::ColConstIterator itData = itRow.begin();
		EXPECT_EQ(Matrix::Data(1, 2, 3), itData.val());
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
	TEST(SparseMatrixTest, CheckThatAValueCanBeAddedToADataInARow)
	{
		Matrix matrix;

		const Matrix::KeyType rowIndex = 42;
		Matrix::RowIterator itRow = matrix.writeLine(rowIndex);

		const Matrix::KeyType colIndex = 123;

		{
			itRow.setCol(colIndex, Matrix::Data(1, 2, 3));
		}

		Matrix::ColIterator itData = itRow.begin();
		EXPECT_EQ(Matrix::Data(1, 2, 3), itData.val());
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatSettingAnExistingElementUpdatesItsValue)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

        Matrix::RowIterator itRow = matrix.begin();

        {
            itRow.setCol(line1.data2.index, Matrix::Data(7, 5, 3));
        }

        Matrix::ColIterator itData = itRow.begin();
        ++itData;
        EXPECT_EQ(Matrix::Data(7, 5, 3), itData.val());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatSettingAnExistingElementLetTheOthersUnaffected)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

        Matrix::RowIterator itRow = matrix.begin();

        {
            itRow.setCol(line1.data2.index + 1, Matrix::Data(9, 5, 1));
            itRow.setCol(line1.data2.index, Matrix::Data(7, 5, 3));
        }

        Matrix::ColIterator itData = itRow.begin();
		EXPECT_EQ(Matrix::Data(1, 2, 3), itData.val());

        ++itData; ++itData;
        EXPECT_EQ(Matrix::Data(9, 5, 1), itData.val());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheValueOfAnElementCanBeSummedToTheOneOfAnExistingElementWithTheSameColIndex)
	{
		Matrix matrix;

		const TestHelpers::line_t line = TestHelpers::Populate(matrix);

        Matrix::RowIterator itRow = matrix.begin();
        const Matrix::Data value(7, 5, 3);

        {
            itRow.addCol(line.data2.index, value);
        }

        Matrix::ColIterator itData = itRow.begin();
		++itData;
        EXPECT_EQ(line.data2.value + value, itData.val());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatANewElementIsInsertedIntoARowWhenTryingToSumItsValueToANonExistingOne)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

        Matrix::RowIterator itRow = matrix.begin();
        const Matrix::Data value(7, 5, 3);

        {
            itRow.addCol(line1.data2.index + 1, value);
        }

        Matrix::ColIterator itData = itRow.begin();
		++itData; ++itData;
        EXPECT_EQ(value, itData.val());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatANewElementIsInsertedIntoARowWhenTryingToSumItsValueToAnotherButTheRowIsEmpty)
	{
		Matrix matrix;

		matrix.writeLine(23);
        Matrix::RowIterator itRow = matrix.writeLine(42);

        const Matrix::Data value(1, 2, 3);

        {
            itRow.addCol(123, value);
        }

        Matrix::ColIterator itData = itRow.begin();
        EXPECT_EQ(value, itData.val());
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatALineCanBeIdenticallyRepopulatedUsingTheSumFunctionAfterItHasBeenClearedInAOneLineMatrix)
	{
        Matrix matrix;

		const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, 42);

        // Clear
        {
            Matrix::RowIterator itRow = matrix.writeLine(line.rowIndex);
		    Matrix::RowType& row = itRow.row();
            row.clear();
        }

        // Repopulation
        {
            Matrix::RowIterator itRow = matrix.writeLine(line.rowIndex);
            itRow.addCol(line.data1.index, line.data1.value);
            itRow.addCol(line.data2.index, line.data2.value);
        }

        Matrix::RowConstIterator itRow = matrix.readLine(line.rowIndex);
        const Matrix::RowType& row = itRow.row();

        {
            Matrix::RowType::const_iterator itData = row.find(line.data1.index);
            EXPECT_EQ(line.data1.index, itData->first);
            EXPECT_EQ(line.data1.value, itData->second);
        }

        {
            Matrix::RowType::const_iterator itData = row.find(line.data2.index);
            EXPECT_EQ(line.data2.index, itData->first);
            EXPECT_EQ(line.data2.value, itData->second);
        }
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTrueIsReturnedWhenComparingTwoColConstIteratorsPointingToTheSameElement)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

        Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);

		Matrix::ColConstIterator itDataA = itRow.begin();
        Matrix::ColConstIterator itDataB = itDataA;

		EXPECT_TRUE(itDataA == itDataB);
	}

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTrueIsReturnedWhenComparingTwoColConstIteratorsPointingToDistinctElements)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

        Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);

		Matrix::ColConstIterator itDataA = itRow.begin();

        Matrix::ColConstIterator itDataB = itDataA;
        ++itDataB;

		EXPECT_TRUE(itDataA != itDataB);
	}

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatPreIncrementingSizeTimesTheColConstIteratorToTheBeginningOfARowResultsInThePastTheEndIterator)
	{
		Matrix matrix;

		const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

		Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);

		Matrix::ColConstIterator itData = itRow.begin();
		++itData; ++itData;
		EXPECT_EQ(itRow.end(), itData);
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
	TEST(SparseMatrixTest, CheckThatPostIncrementingSizeTimesTheColConstIteratorToTheBeginningOfARowResultsInThePastTheEndIterator)
	{
		Matrix matrix;
		const TestHelpers::line_t line1 = TestHelpers::Populate(matrix);

		Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);

		Matrix::ColConstIterator itData = itRow.begin();
		itData++; itData++;
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

	//////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatANewLineCanBeInsertedAndItsElementsAccessedAfterTheMatrixHasBeenCleared)
	{
		Matrix matrix;

		{
            TestHelpers::WriteLine(matrix, 42);

		    matrix.clear();
        }

        const TestHelpers::line_t line = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(42));

        {
            Matrix::RowConstIterator itRow = matrix.readLine(line.rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(line.data1.index);
                EXPECT_EQ(line.data1.index, itData->first);
                EXPECT_EQ(line.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line.data2.index);
                EXPECT_EQ(line.data2.index, itData->first);
                EXPECT_EQ(line.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line.data3.index);
                EXPECT_EQ(line.data3.index, itData->first);
                EXPECT_EQ(line.data3.value, itData->second);
            }
        }
	}

	////////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatDataCanBeRetrievedWhenAllLinesAreCreatedFirstAndThenDataInserted)
	{
		Matrix matrix;

		{
			Matrix::RowIterator itRow23 = matrix.writeLine(23);
			Matrix::RowIterator itRow42 = matrix.writeLine(42);

			{
				itRow23.setCol(123, Matrix::Data(1, 2, 3));
				itRow23.setCol(456, Matrix::Data(4, 5, 6));
			}

			{
				itRow42.setCol(789, Matrix::Data(7, 8, 9));
				itRow42.setCol(852, Matrix::Data(8, 5, 2));
			}
		}

        {
            Matrix::RowConstIterator itRow = matrix.readLine(23);

            Matrix::ColConstIterator itCol = itRow.begin();
            EXPECT_EQ(123u, itCol.index());
            EXPECT_EQ(Matrix::Data(1, 2, 3), itCol.val());

            ++itCol;

            EXPECT_EQ(456, itCol.index());
            EXPECT_EQ(Matrix::Data(4, 5, 6), itCol.val());
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(42);
            Matrix::ColConstIterator itCol = itRow.begin();
            EXPECT_EQ(789u, itCol.index());
            EXPECT_EQ(Matrix::Data(7, 8, 9), itCol.val());

            ++itCol;

            EXPECT_EQ(852, itCol.index());
            EXPECT_EQ(Matrix::Data(8, 5, 2), itCol.val());
        }
	}

	////////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatDataCanBeRetrievedWhenDataAreInsertedImmediatelyAfterLineCreation)
	{
		Matrix matrix;

		{
			{
				Matrix::RowIterator itRow = matrix.writeLine(23);

			    itRow.setCol(123, Matrix::Data(1, 2, 3));
				itRow.setCol(456, Matrix::Data(4, 5, 6));
			}

			{
				Matrix::RowIterator itRow = matrix.writeLine(42);

			    itRow.setCol(789, Matrix::Data(7, 8, 9));
				itRow.setCol(852, Matrix::Data(8, 5, 2));
			}
		}

        {
            Matrix::RowConstIterator itRow = matrix.readLine(23);

            Matrix::ColConstIterator itCol = itRow.begin();
            EXPECT_EQ(123u, itCol.index());
            EXPECT_EQ(Matrix::Data(1, 2, 3), itCol.val());

            ++itCol;

            EXPECT_EQ(456, itCol.index());
            EXPECT_EQ(Matrix::Data(4, 5, 6), itCol.val());
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(42);
            Matrix::ColConstIterator itCol = itRow.begin();
            EXPECT_EQ(789u, itCol.index());
            EXPECT_EQ(Matrix::Data(7, 8, 9), itCol.val());

            ++itCol;

            EXPECT_EQ(852, itCol.index());
            EXPECT_EQ(Matrix::Data(8, 5, 2), itCol.val());
        }
	}

	////////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatElementsOfALineAreUnaffectedWhenTryingToWriteItWhenItAlreadyExists)
	{
		Matrix matrix;

		{
			Matrix::RowIterator itRow23 = matrix.writeLine(23);
			Matrix::RowIterator itRow42 = matrix.writeLine(42);

			{
				itRow23.setCol(123, Matrix::Data(1, 2, 3));
				itRow23.setCol(456, Matrix::Data(4, 5, 6));
			}

            {
				itRow42.setCol(789, Matrix::Data(7, 8, 9));
				itRow42.setCol(852, Matrix::Data(8, 5, 2));

                matrix.writeLine(42);
			}
		}

        {
            Matrix::RowConstIterator itRow = matrix.readLine(42);

            Matrix::ColConstIterator itCol = itRow.begin();
            EXPECT_EQ(789u, itCol.index());
            EXPECT_EQ(Matrix::Data(7, 8, 9), itCol.val());

            ++itCol;

            EXPECT_EQ(852, itCol.index());
            EXPECT_EQ(Matrix::Data(8, 5, 2), itCol.val());
        }
    }

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatElementsCanBeInsertedWhenALineIsCreated)
    {
        TestHelpers::line_t line = TestHelpers::nullLine();

        Matrix matrix;
        const Matrix::KeyType rowIndex = 23;

        {
            Matrix sourceMatrix;
            line = TestHelpers::WriteLine(sourceMatrix, 42);

            matrix.writeLine(rowIndex, sourceMatrix.begin().row());
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(line.data1.index);
                EXPECT_EQ(line.data1.index, itData->first);
                EXPECT_EQ(line.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line.data2.index);
                EXPECT_EQ(line.data2.index, itData->first);
                EXPECT_EQ(line.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line.data3.index);
                EXPECT_EQ(line.data3.index, itData->first);
                EXPECT_EQ(line.data3.value, itData->second);
            }
        }
    }

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatTheSizeOfALineIsUpdatedWhenItIsReplacedByAnother)
    {
        TestHelpers::line_t line = TestHelpers::nullLine();

        Matrix matrix;
        const Matrix::KeyType rowIndex = 42;

        {
            TestHelpers::WriteLine(matrix, rowIndex);
        }

        {
            Matrix sourceMatrix;
            line = TestHelpers::WriteLine(sourceMatrix, TestHelpers::GetNextUniqueIndex(rowIndex));

            matrix.writeLine(rowIndex, sourceMatrix.begin().row());
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(rowIndex);
            EXPECT_EQ((std::size_t)line.initialDataCount, itRow.row().size());
        }
    }

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatTheExpectedNumberOfElementsCanBeTraversedViaIteratorsWhenALineHasBeenReplacedByAnother)
    {
        TestHelpers::line_t line = TestHelpers::nullLine();

        Matrix matrix;
        const Matrix::KeyType rowIndex = 42;

        {
            TestHelpers::WriteLine(matrix, rowIndex);
        }

        {
            Matrix sourceMatrix;
            line = TestHelpers::WriteLine(sourceMatrix, TestHelpers::GetNextUniqueIndex(rowIndex));

            matrix.writeLine(rowIndex, sourceMatrix.begin().row());
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(rowIndex);

            Matrix::ColConstIterator itData = itRow.begin();
            for (unsigned int i = 0; i < line.initialDataCount; i++)
            {
                SOFA_ASSERT(itData != itRow.end());
                ++itData;
            }
            EXPECT_EQ(itRow.end(), itData);
        }
    }

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatTheExpectedElementsCanBeTraversedViaIteratorsWhenALineHasBeenReplacedByAnother)
    {
        TestHelpers::line_t line = TestHelpers::nullLine();

        Matrix matrix;
        const Matrix::KeyType rowIndex = 42;

        {
            TestHelpers::WriteLine(matrix, rowIndex);
        }

        {
            Matrix sourceMatrix;
            line = TestHelpers::WriteLine(sourceMatrix, TestHelpers::GetNextUniqueIndex(rowIndex));

            matrix.writeLine(rowIndex, sourceMatrix.begin().row());
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(rowIndex);

            Matrix::ColConstIterator itData = itRow.begin();

            {
                EXPECT_EQ(line.data1.index, itData.index());
                EXPECT_EQ(line.data1.value, itData.val());
            }

            ++itData;

            {
                EXPECT_EQ(line.data2.index, itData.index());
                EXPECT_EQ(line.data2.value, itData.val());
            }

            ++itData;

            {
                EXPECT_EQ(line.data3.index, itData.index());
                EXPECT_EQ(line.data3.value, itData.val());
            }
        }
    }

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatExistingLinesStayUnaffectedWhenALineIsReplacedByAnother)
    {
        TestHelpers::line_t line = TestHelpers::nullLine();

        Matrix matrix;
        const Matrix::KeyType rowIndex = 42;

        {
            TestHelpers::WriteLine(matrix, rowIndex);
            line = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(rowIndex));
        }

        {
            Matrix sourceMatrix;
            TestHelpers::WriteLine(sourceMatrix, TestHelpers::GetNextUniqueIndex(line.rowIndex));

            matrix.writeLine(rowIndex, sourceMatrix.begin().row());
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(line.rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(line.data1.index);
                EXPECT_EQ(line.data1.index, itData->first);
                EXPECT_EQ(line.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line.data2.index);
                EXPECT_EQ(line.data2.index, itData->first);
                EXPECT_EQ(line.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line.data3.index);
                EXPECT_EQ(line.data3.index, itData->first);
                EXPECT_EQ(line.data3.value, itData->second);
            }
        }
    }

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatTheContentOfALineCanBeSummedWithTheContentOfAnother)
    {
        TestHelpers::line_t sourceLine = TestHelpers::nullLine();

        Matrix matrix;
        const Matrix::KeyType rowIndex = 23;

        TestHelpers::line_t line = TestHelpers::WriteLine(matrix, rowIndex);

        {
            Matrix sourceMatrix;
            sourceLine = TestHelpers::WriteLine(sourceMatrix, 42, rowIndex + 1);

            matrix.addLine(rowIndex, sourceMatrix.begin().row());
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(sourceLine.data1.index);
                EXPECT_EQ(sourceLine.data1.index, itData->first);
                EXPECT_EQ(line.data1.value + sourceLine.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(sourceLine.data2.index);
                EXPECT_EQ(sourceLine.data2.index, itData->first);
                EXPECT_EQ(line.data2.value + sourceLine.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(sourceLine.data3.index);
                EXPECT_EQ(sourceLine.data3.index, itData->first);
                EXPECT_EQ(line.data3.value + sourceLine.data3.value, itData->second);
            }
        }
    }

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatTheSizeOfALineIsUpdatedWhenNewElementsAreCreatedWhenTryingToSumALineToAnotherWithNoCommonColIndices)
    {
        TestHelpers::line_t sourceLine = TestHelpers::nullLine();

        Matrix matrix;
        const Matrix::KeyType rowIndex = 23;

        TestHelpers::line_t line = TestHelpers::WriteLine(matrix, rowIndex);

        {
            Matrix sourceMatrix;
            sourceLine = TestHelpers::WriteLine(sourceMatrix, 42, TestHelpers::GetNextUniqueIndex(rowIndex));

            matrix.addLine(rowIndex, sourceMatrix.begin().row());
        }

        Matrix::RowConstIterator itRow = matrix.readLine(rowIndex);
        EXPECT_EQ(line.initialDataCount * 2, itRow.row().size());
    }

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatNewElementsAreCreatedWhenTryingToSumALineToAnotherWithNoCommonColIndices)
    {
        TestHelpers::line_t sourceLine = TestHelpers::nullLine();

        Matrix matrix;
        const Matrix::KeyType rowIndex = 23;

        {
            Matrix sourceMatrix;
            sourceLine = TestHelpers::WriteLine(sourceMatrix, 42, TestHelpers::GetNextUniqueIndex(rowIndex));

            matrix.addLine(rowIndex, sourceMatrix.begin().row());
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(sourceLine.data1.index);
                EXPECT_EQ(sourceLine.data1.index, itData->first);
                EXPECT_EQ(sourceLine.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(sourceLine.data2.index);
                EXPECT_EQ(sourceLine.data2.index, itData->first);
                EXPECT_EQ(sourceLine.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(sourceLine.data3.index);
                EXPECT_EQ(sourceLine.data3.index, itData->first);
                EXPECT_EQ(sourceLine.data3.value, itData->second);
            }
        }
    }

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatTheSizeOfALineIsUpdatedWhenNewElementsAreCreatedWhenTryingToSumALineToAnotherWithCommonAndDistinctIndices)
    {
        TestHelpers::line_t sourceLine = TestHelpers::nullLine();

        Matrix matrix;
        const Matrix::KeyType rowIndex = 23;

        {
            Matrix sourceMatrix;
            sourceLine = TestHelpers::WriteLine(sourceMatrix, 42);

            Matrix::RowIterator itRow = matrix.writeLine(rowIndex);
            itRow.setCol(TestHelpers::GetNextUniqueIndex(rowIndex), sourceLine.data2.value);

            matrix.addLine(rowIndex, sourceMatrix.begin().row());
        }

        Matrix::RowConstIterator itRow = matrix.readLine(rowIndex);
        EXPECT_EQ(sourceLine.initialDataCount + 1, itRow.row().size());
    }

    //////////////////////////////////////////////////
	TEST(SparseMatrixTest, CheckThatTheContentOfALineRemainsSortedByColIndex)
	{
		Matrix matrix;

		Matrix::RowIterator itRow = matrix.writeLine(42);

        {
            itRow.setCol(123, Matrix::Data(1, 2, 3));
        }

        {
            itRow.setCol(789, Matrix::Data(7, 8, 9));
        }

        std::cout << "azerty " << std::endl << matrix << std::endl;

        {
            itRow.setCol(456, Matrix::Data(4, 5, 6));
        }

        std::cout << matrix << std::endl;
	}

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatElementsHaveTheExpectedValuesWhenTryingToSumALineToAnotherWithNoCommonColIndices)
    {
        TestHelpers::line_t sourceLine = TestHelpers::nullLine();

        Matrix matrix;
        const Matrix::KeyType rowIndex = 23;

        Matrix::KeyType colIndex = 0;

        {
            Matrix sourceMatrix;
            sourceLine = TestHelpers::WriteLine(sourceMatrix, 42);

            std::cout << matrix << std::endl;
            Matrix::RowIterator itRow = matrix.writeLine(rowIndex);

            colIndex = TestHelpers::GetNextUniqueIndex(sourceLine.rowIndex);
            itRow.setCol(colIndex, Matrix::Data(1, 2, 3));

            std::cout << matrix << std::endl;
            matrix.addLine(rowIndex, sourceMatrix.begin().row());
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(colIndex);
                EXPECT_EQ(colIndex, itData->first);
                EXPECT_EQ(Matrix::Data(1, 2, 3), itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(sourceLine.data1.index);
                EXPECT_EQ(sourceLine.data1.index, itData->first);
                EXPECT_EQ(sourceLine.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(sourceLine.data2.index);
                EXPECT_EQ(sourceLine.data2.index, itData->first);
                EXPECT_EQ(sourceLine.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(sourceLine.data3.index);
                EXPECT_EQ(sourceLine.data3.index, itData->first);
                EXPECT_EQ(sourceLine.data3.value, itData->second);
            }
        }
    }

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatDataCanBeRetrievedWhenCreatingAndFillingLinesInDescendingIndexOrder)
    {
        Matrix matrix;

        TestHelpers::line_t line1 = TestHelpers::WriteLine(matrix, TestHelpers::GetNextUniqueIndex(42));
        TestHelpers::line_t line2 = TestHelpers::WriteLine(matrix, 42);

        {
            Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(line1.data1.index);
                EXPECT_EQ(line1.data1.index, itData->first);
                EXPECT_EQ(line1.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line1.data2.index);
                EXPECT_EQ(line1.data2.index, itData->first);
                EXPECT_EQ(line1.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line1.data3.index);
                EXPECT_EQ(line1.data3.index, itData->first);
                EXPECT_EQ(line1.data3.value, itData->second);
            }
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(line2.rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(line2.data1.index);
                EXPECT_EQ(line2.data1.index, itData->first);
                EXPECT_EQ(line2.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line2.data2.index);
                EXPECT_EQ(line2.data2.index, itData->first);
                EXPECT_EQ(line2.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line2.data3.index);
                EXPECT_EQ(line2.data3.index, itData->first);
                EXPECT_EQ(line2.data3.value, itData->second);
            }
        }
    }

    ////////////////////////////////////////////////////
    TEST(SparseMatrixTest, CheckThatDataCanBeRetrievedWhenCreatingLinesInDescendingIndexOrderAndInsertingADataInTheFirstOneAfterwards)
    {
        Matrix matrix;

        const Matrix::KeyType rowIndex2 = 42;
        const Matrix::KeyType rowIndex1 = TestHelpers::GetNextUniqueIndex(rowIndex2);

        TestHelpers::line_t line1 = TestHelpers::WriteLine(matrix, rowIndex1);
        TestHelpers::line_t line2 = TestHelpers::WriteLine(matrix, rowIndex2);

        {
            Matrix::RowIterator itRow = matrix.writeLine(rowIndex1);
            itRow.setCol(line2.data3.index + 1, line2.data3.value * 2);
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(line1.rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(line1.data1.index);
                EXPECT_EQ(line1.data1.index, itData->first);
                EXPECT_EQ(line1.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line1.data2.index);
                EXPECT_EQ(line1.data2.index, itData->first);
                EXPECT_EQ(line1.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line1.data3.index);
                EXPECT_EQ(line1.data3.index, itData->first);
                EXPECT_EQ(line1.data3.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line2.data3.index + 1);
                EXPECT_EQ(line2.data3.index + 1, itData->first);
                EXPECT_EQ(line2.data3.value * 2, itData->second);
            }
        }

        {
            Matrix::RowConstIterator itRow = matrix.readLine(line2.rowIndex);
            Matrix::RowType row = itRow.row();

            {
                Matrix::RowType::const_iterator itData = row.find(line2.data1.index);
                EXPECT_EQ(line2.data1.index, itData->first);
                EXPECT_EQ(line2.data1.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line2.data2.index);
                EXPECT_EQ(line2.data2.index, itData->first);
                EXPECT_EQ(line2.data2.value, itData->second);
            }

            {
                Matrix::RowType::const_iterator itData = row.find(line2.data3.index);
                EXPECT_EQ(line2.data3.index, itData->first);
                EXPECT_EQ(line2.data3.value, itData->second);
            }
        }
    }

}
