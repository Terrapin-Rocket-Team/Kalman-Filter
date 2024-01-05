#define CATCH_CONFIG_MAIN
#include "catch_amalgamated.hpp"

// Include your matrix functions
#include "../MatrixMult/MatrixMult.h"

// TEST_CASE("Matrix Transpose Test", "[transposeMatrix]") {
//     const int rows = 3;
//     const int cols = 4;

//     long matrix[rows * cols] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
//     long transposedMatrix[cols * rows];

//     transpose_matrix(matrix, transposedMatrix, rows, cols);

//     // Check if the arrays are equal
//     REQUIRE(std::equal(transposedMatrix, transposedMatrix + (cols * rows), expectedTransposedMatrix));
// }

// TEST_CASE("Matrix Inverse Test", "[inverseMatrix]") {
//     const int size = 3;

//     long matrix[size * size] = {1, 2, 3, 0, 1, 4, 5, 6, 0};
//     long* inverseMatrix = inverseMatrix(matrix, size);

//     // Check if the arrays are equal
//     REQUIRE(std::equal(inverseMatrix, inverseMatrix + (size * size), expectedInverseMatrix));

//     delete[] inverseMatrix; // Don't forget to free the allocated memory
// }

TEST_CASE("Matrix Multiplication Test", "[multiplyMatrices]") {
    const int rows1 = 2;
    const int cols1 = 2;
    const int rows2 = 2;
    const int cols2 = 2;

    long matrix1[rows1 * cols1] = {1, 2, 3, 4};
    long matrix2[rows2 * cols2] = {5, 6, 7, 8};   
    long expectedResultMatrix[4] = {19, 22, 43, 50};
    long* resultMatrix = multiply_matrices(matrix1, matrix2, rows1, cols1, rows2, cols2);

    // Check if the arrays are equal
    REQUIRE(std::equal(resultMatrix, resultMatrix + (rows1 * cols2), expectedResultMatrix));

    delete[] resultMatrix; // Don't forget to free the allocated memory
}
