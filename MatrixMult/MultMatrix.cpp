//#include <bits/stdc++.h>
#include "MultMatrix.h"
using namespace std;
//#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>
                                     
vector<vector<double>> mulMat(vector<vector<double> > &mat1, vector<vector<double> > &mat2)
{
    if (mat1.size() == 0 || mat2.size() == 0) {
     // throw std::invalid_argument("Empty matrices");
    }
    if (mat1[0].size() != mat2.size()) {
      //  throw std::invalid_argument("Numbers of columns of matrix 1 does not equal number of rows of matrix 2");
    }
    vector<vector<double>> rslt( mat1.size() , vector<double> (mat2[0].size(), 0.0)); 
 
 
    for (int i = 0; i < mat1.size(); i++) {
        for (int j = 0; j < mat2[0].size(); j++) {
            rslt[i][j] = 0;
 
            for (int k = 0; k < mat2.size(); k++) {
                rslt[i][j] += mat1[i][k] * mat2[k][j];
            }
 
        }
 
    }
    return rslt;
}
