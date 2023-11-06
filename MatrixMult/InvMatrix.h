#ifndef INV_MATRIX
#define INV_MATRIX
#pragma once

#include <ArduinoEigen.h>
//#include <iostream>
//#include <bits/stdc++.h>
using namespace std;

void getCofactor(vector<vector<double>> &A, vector<vector<double>> &temp, int p, int q, int n);

int determinant(vector<vector<double>> &A, int n);

void adjoint(vector<vector<double>> &A, vector<vector<double>> &adj);

bool inverse(vector<vector<double>> &A, vector<vector<double>> &inverse);

void transpose(vector<vector<double> > &A, vector<vector<double>> &AT);

vector<vector<double>> add_mult(vector<vector<double>> &A, vector<vector<double>> &B);

vector<vector<double>> sub_mult(vector<vector<double>> &A, vector<vector<double>> &B); 

vector<vector<double>> identity(int n); 

// template <class T>
// void display(vector<vector<T>> &A)
// {
//     for (int i = 0; i < A.size(); i++)
//     {
//         for (int j = 0; j < A.size(); j++)
//             cout << A[i][j] << " ";
//         cout << endl;
//     }
// }

#endif
