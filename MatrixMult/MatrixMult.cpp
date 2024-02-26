#include "MatrixMult.h"
#include <iostream>
#include <cmath>

double* multiplyMatrices(double* matrix1, double* matrix2, int matrix1_rows, int matrix2_rows, int matrix1_cols, int matrix2_cols){
    if (matrix1_cols != matrix2_rows) {
        std::cerr << "Error: Columns in matrix1 must match rows in matrix2." << std::endl;
        return nullptr;
    }

    double* result = new double[matrix1_rows * matrix2_cols];

    for (int i = 0; i < matrix1_rows; ++i) {
        for (int j = 0; j < matrix2_cols; ++j) {
            result[i * matrix2_cols + j] = 0;
            for (int k = 0; k < matrix1_cols; ++k) {
                result[i * matrix2_cols + j] += matrix1[i * matrix1_cols + k] * matrix2[k * matrix2_cols + j];
            }
        }
    }
    
    return result;
}


// Uses Gaussian Elimination with Partial Pivoting to find the inverse of a matrix
void luDecompositionWithPartialPivoting(double* A, int* pivot, int n) {
    for (int i = 0; i < n; ++i) {
        pivot[i] = i;
    }

    for (int i = 0; i < n; ++i) {
        // Partial pivoting
        double max = std::abs(A[i*n + i]);
        int maxRow = i;
        for (int k = i + 1; k < n; ++k) {
            if (std::abs(A[k*n + i]) > max) {
                max = std::abs(A[k*n + i]);
                maxRow = k;
            }
        }

        if (max == 0.0) {
            std::cerr << "Error: Matrix is singular, cannot calculate inverse." << std::endl;
            return;
        }

        // Swap rows in A matrix
        for (int k = 0; k < n; ++k) {
            std::swap(A[i*n + k], A[maxRow*n + k]);
        }
        // Swap pivot indices
        std::swap(pivot[i], pivot[maxRow]);

        // LU Decomposition
        for (int j = i + 1; j < n; ++j) {
            A[j*n + i] /= A[i*n + i];
            for (int k = i + 1; k < n; ++k) {
                A[j*n + k] -= A[j*n + i] * A[i*n + k];
            }
        }
    }
}

void solveLU(double* A, int* pivot, double* b, double* x, int n) {
    // Forward substitution for Ly = Pb
    for (int i = 0; i < n; ++i) {
        x[i] = b[pivot[i]];
        for (int j = 0; j < i; ++j) {
            x[i] -= A[i*n + j] * x[j];
        }
    }

    // Backward substitution for Ux = y
    for (int i = n - 1; i >= 0; --i) {
        for (int j = i + 1; j < n; ++j) {
            x[i] -= A[i*n + j] * x[j];
        }
        x[i] /= A[i*n + i];
    }
}

double* inverseMatrix(double* A, int n) {
    double* inverse = new double[n*n];
    int* pivot = new int[n];
    double* b = new double[n];
    double* temp = new double[n];

    luDecompositionWithPartialPivoting(A, pivot, n);

    for (int i = 0; i < n; ++i) {
        // Set up b vector for the i-th column
        std::fill(b, b + n, 0.0);
        b[i] = 1.0;

        solveLU(A, pivot, b, temp, n);

        // Copy the solution from temp to the correct column in inverse
        for (int j = 0; j < n; ++j) {
            inverse[j*n + i] = temp[j];
        }
    }

    delete[] pivot;
    delete[] b;
    delete[] temp;

    return inverse;
}


double* transposeMatrix(double* matrix, int rows, int cols) {
    double* result = new double[rows * cols];

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[j * rows + i] = matrix[i * cols + j];
        }
    }
    
    return result;
}

double* addMatrices(double* mat1, double* mat2, int rows, int cols) {
    double* result = new double[rows * cols];

    for (int i = 0; i < rows * cols; ++i) {
        result[i] = mat1[i] + mat2[i];
    }

    return result;
}

double* subMatrices(double* mat1, double* mat2, int rows, int cols) {
    double* result = new double[rows * cols];

    for (int i = 0; i < rows * cols; ++i) {
        result[i] = mat1[i] - mat2[i];
    }

    return result;
}

double* ident(int size){
    double* result = new double[size * size];

    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (i == j) {
                result[i * size + j] = 1;
            } else {
                result[i * size + j] = 0;
            }
        }
    }

    return result;
}

double* multiplyByScalar(double* mat, int size, double scalar){
    double* result = new double[size];

    for(int i = 0; i < size; ++i){
        result[i] = mat[i] * scalar;
    }

    return result;
}