#define CATCH_CONFIG_MAIN
#include "catch_amalgamated.hpp"

// Include your matrix functions
#include "../MatrixMult/MatrixMult.h"

// TEST_CASE("Matrix Transpose Test", "[transposeMatrix]") {
//     const int rows = 3;
//     const int cols = 4;

//     double matrix[rows * cols] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
//     double transposedMatrix[cols * rows];

//     transpose_matrix(matrix, transposedMatrix, rows, cols);

//     // Check if the arrays are equal
//     REQUIRE(std::equal(transposedMatrix, transposedMatrix + (cols * rows), expectedTransposedMatrix));
// }

// TEST_CASE("Matrix Inverse Test", "[inverseMatrix]") {
//     const int size = 3;

//     double matrix[size * size] = {1, 2, 3, 0, 1, 4, 5, 6, 0};
//     double* inverseMatrix = inverseMatrix(matrix, size);

//     // Check if the arrays are equal
//     REQUIRE(std::equal(inverseMatrix, inverseMatrix + (size * size), expectedInverseMatrix));

//     delete[] inverseMatrix; // Don't forget to free the allocated memory
// }

TEST_CASE("Matrix Multiplication Test", "[multiplyMatrices]") {
    const int rows1 = 6;
    const int cols1 = 6;
    const int rows2 = 6;
    const int cols2 = 6;

    double matrix1[rows1 * cols1] = {0.758214, 0.124587, 0.965321, 0.332156, 0.541892, 0.789654, 0.234561, 0.987654, 0.123456, 0.876543, 0.456789, 0.789123, 0.321654, 0.987321, 0.654789, 0.159264, 0.246813, 0.753159, 0.852963, 0.369147, 0.465738, 0.913248, 0.578412, 0.654321, 0.123987, 0.456852, 0.789654, 0.321456, 0.987456, 0.654123, 0.852147, 0.369456, 0.741258, 0.147896, 0.596874, 0.12938};
    double matrix2[rows2 * cols2] = {0.845691, 0.193847, 0.672491, 0.518724, 0.937246, 0.264819, 0.785413, 0.639784, 0.372915, 0.918465, 0.486275, 0.759318, 0.124569, 0.594287, 0.831726, 0.279643, 0.765912, 0.428196, 0.695347, 0.183742, 0.527491, 0.864932, 0.941827, 0.356189, 0.612384, 0.847563, 0.295718, 0.674823, 0.158497, 0.749621, 0.462835, 0.925147, 0.591364, 0.318456, 0.802346, 0.637218};   
    double expectedResultMatrix[4] = {19, 22, 43, 50};
    double* resultMatrix = multiplyMatrices(matrix1, matrix2, rows1, cols1, rows2, cols2);

    // Check if the arrays are equal
    REQUIRE(std::equal(resultMatrix, resultMatrix + (rows1 * cols2), expectedResultMatrix));

    delete[] resultMatrix; // Don't forget to free the allocated memory
}
