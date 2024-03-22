#include "Matrix.h"

//Constructor
// int rows -> number of rows of matrix
// int cols -> number of columns of matrix
// double[] array -> array of elements of matrix in column-major order
Matrix::Matrix(int rows, int cols, double* array) {
  this->rows = rows;
  this->cols = cols;
  this->array = array;
}

Matrix::~Matrix(){
  delete[] this->array;
}

int Matrix::getRows(){
  return this->rows;
}

int Matrix::getCols(){
  return this->cols;
}

Matrix Matrix::operator*(Matrix& other){
  return this->multiply(other);
}

Matrix Matrix::operator*(double scalar){
  return this->multiply(scalar);
}

Matrix Matrix::multiply(Matrix& other){
  if (this->cols != other.rows){
    std::cerr << "Multiplication error: Dimensions do not match!" << std::endl;
  }

  double* result = new double[this->rows * other.cols];

  for (int i = 0; i < this->rows; ++i){
    for (int j = 0; j < other.cols; ++j){
      result[i * other.cols + j] = 0;
      for (int k = 0; k < this->cols; ++k){
	result[i * other.cols + j] += this->array[i * this->cols + k] * other.array[k * other.cols + j];
      }
    }
  }

  return Matrix(this->rows, other.cols, result);
}

Matrix Matrix::multiply(double scalar){
  double* result = new double[this->rows * this->cols];

  for (int i = 0; i < this->rows * this->cols; ++i){
    result[i] = this->array[i] * scalar;
  }

  return Matrix(this->rows, this->cols, result);
}

Matrix Matrix::operator+(Matrix& other){
  return this->add(other);
}

Matrix Matrix::add(Matrix& other){
  if(this->rows != other.rows || this->cols != other.cols){
    std::cerr << "Addition error: Dimensions do not match!" << std::endl;
  }
  
  double* result = new double[this->rows * this->cols];

  for (int i = 0; i < this->rows * this->cols; ++i){
    result[i] = this->array[i] + other.array[i];
  }

  return Matrix(this->rows, this->cols, result);
}

Matrix Matrix::operator-(Matrix& other){
  return this->subtract(other);
}

Matrix Matrix::subtract(Matrix& other){
  if(this->rows != other.rows || this->cols != other.cols){
    std::cerr << "Subtraction error: Dimensions do not match!" << std::endl;
  }

  double* result = new double[this->rows * this->cols];

  for (int i = 0; i < this->rows * this->cols; ++i){
    result[i] = this->array[i] - other.array[i];
  }

  return Matrix(this->rows, this->cols, result);
}

Matrix Matrix::T(){
  return this->transpose();
}

Matrix Matrix::transpose(){
  double* result = new double[this->rows * this->cols];

  for (int i = 0; i < this->rows; ++i){
    for (int j = 0; j < this->cols; ++j){
      result[j * this->rows + i] = this->array[i * this->cols + j];
    }
  }

  return Matrix(this->cols, this->rows, result);
}

void Matrix::luDecompositionWithPartialPivoting(double* A, int* pivot, int n) {
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
      std::cerr << "Inversion error: Matrix is singular!" << std::endl;
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

void Matrix::solveLU(double* A, int* pivot, double* b, double* x, int n) {
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

Matrix Matrix::inv(){
  return this->inverse();
}

Matrix Matrix::inverse(){
  if(this->rows != this->cols){
    std::cerr << "Inversion error: Dimensions do not match!" << std::endl;
  }

  int n = this->rows;
  double* A = new double[n * n];
  for (int i = 0; i < n * n; ++i){
    A[i] = this->array[i];
  }

  double* inverse = new double[n * n];
  int* pivot = new int[n];
  double* b = new double[n];
  double* temp = new double[n];

  this->luDecompositionWithPartialPivoting(A, pivot, n);

  for (int i = 0; i < n; ++i){
    std::fill(b, b + n, 0.0);
    b[i] = 1.0;

    this->solveLU(A, pivot, b, temp, n);

    for (int g = 0; g < n; g++){
      std::cout << "g " << g << " pivot " << pivot[g] << " temp " << temp[g];
    }
    std::cout << std::endl;

    
    for (int j = 0; j < n; ++j){
      inverse[j * n + i] = temp[j];
    }
  }

  delete[] A;
  delete[] pivot;
  delete[] b;
  delete[] temp;

  return Matrix(n, n, inverse);
}

Matrix Matrix::ident(int n){
  double* result = new double[n * n];

  for (int i = 0; i < n; ++i){
    for (int j = 0; j < n; ++j){
      if (i == j){
	result[i * n + j] = 1;
      } else{
	result[i * n + j] = 0;
      }
    }
  }

  return Matrix(n, n, result);
}


void Matrix::disp(){
  for (int i = 0; i < this->rows; ++i){
    for (int j = 0; j < this->cols; ++j){
      std::cout << this->array[i * this->rows + j] << " ";
    }
    std::cout << std::endl;
  }
}
