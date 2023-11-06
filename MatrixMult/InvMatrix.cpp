#include "InvMatrix.h"

void getCofactor(vector<vector<double>> &A, vector<vector<double>> &temp, int p, int q, int n)
{
    int i = 0, j = 0;

    // Looping for each element of the matrix
    for (int row = 0; row < n; row++)
    {

        for (int col = 0; col < n; col++)
        {

            //  Copying into temporary matrix only those
            //  element which are not in given row and
            //  column
            if (row != p && col != q)
            {

                temp[i][j++] = A[row][col]; // causes segfault

                // Row is filled, so increase row index and
                // reset col index
                if (j == n - 1)
                {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

/* Recursive function for finding determinant of matrix.
   n is current dimension of A[][]. */
int determinant(vector<vector<double>> &A, int n)
{
    double D = 0.0; // Initialize result

    //  Base case : if matrix contains single element
    if (n == 1)
        return A[0][0];

    // vector<vector<int> > temp; // To store cofactors
    vector<vector<double>> temp(A.size(), vector<double>(A.size(), 0.0));

    int sign = 1; // To store sign multiplier

    // Iterate for each element of first row
    for (int f = 0; f < n; f++)
    {
        // Getting Cofactor of A[0][f]
        getCofactor(A, temp, 0, f, n);
        D += sign * A[0][f] * determinant(temp, n - 1);

        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}

// Function to get adjoint of A[N][N] in adj[N][N].
void adjoint(vector<vector<double>> &A, vector<vector<double>> &adj)
{
    if (A.size() == 1)
    {
        adj[0][0] = 1.0;
        return;
    }

    // temp is used to store cofactors of A[][]
    // vector<vector<int> > temp;
    vector<vector<double>> temp(A.size(), vector<double>(A.size(), 0.0));
    int sign = 1;

    for (int i = 0; i < A.size(); i++)
    {
        for (int j = 0; j < A.size(); j++)
        {
            // Get cofactor of A[i][j]
            getCofactor(A, temp, i, j, A.size());

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i + j) % 2 == 0) ? 1 : -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj[j][i] = (sign) * (determinant(temp, A.size() - 1));
        }
    }
}

// Function to calculate and store inverse, returns false if
// matrix is singular
bool inverse(vector<vector<double>> &A, vector<vector<double>> &inverse)
{
    // Find determinant of A[][]
    double det = determinant(A, A.size());
    if (det == 0.0)
    {
      //  cout << "Singular matrix, can't find its inverse";
        return false;
    }

    // Find adjoint
    // vector<vector<int> > adj;
    vector<vector<double>> adj(A.size(), vector<double>(A.size(), 0.0));
    adjoint(A, adj);

    // Find Inverse using formula "inverse(A) =
    // adj(A)/det(A)"
    for (int i = 0; i < A.size(); i++)
        for (int j = 0; j < A.size(); j++)
            inverse[i][j] = adj[i][j] / det;

    return true;
}

//Function to calculate Transpose of the matrix A and store in matrix AT
void transpose(vector<vector<double> > &A, vector<vector<double>> &AT) 
{
    for (int i = 0; i < A.size(); i++) {
        for (int j = 0; j < A.size(); j++) {
            AT[j][i] = A[i][j];
        }
    }
}

//Function to add matrix A, matrix B and return in matrix AT
vector<vector<double>> add_mult(vector<vector<double>> &A, vector<vector<double>> &B) 
{
        if (A.size() != B.size() || A[0].size() != B[0].size())
    {
     //   throw std::invalid_argument("The size of Matrix A and B do not match");
    }
    
    vector<vector<double>> AplusB(A.size(), vector<double> (A[0].size(), 0.0));

    for (int i = 0; i < A.size(); i++) {
        for (int j = 0; j < A[0].size(); j++) {
            AplusB[i][j] = A[i][j] + B[i][j];
        }
    }
    return AplusB;
}

vector<vector<double>> sub_mult(vector<vector<double>> &A, vector<vector<double>> &B) 
{
        if (A.size() != B.size() || A[0].size() != B[0].size())
    {
     //   throw std::invalid_argument("The size of Matrix A and B do not match");
    }
    
    vector<vector<double>> AminB(A.size(), vector<double> (A[0].size(), 0.0));

    for (int i = 0; i < A.size(); i++) {
        for (int j = 0; j < A[0].size(); j++) {
            AminB[i][j] = A[i][j] - B[i][j];
        }
    }
    return AminB;
}

vector<vector<double>> identity(int n) 
{
    vector<vector<double>> A(n, vector<double>(n, 0.0));

    for (int i = 0; i < A.size(); i++) {
        for (int j = 0; j < A.size(); j++) {
            if (i==j){
                    A[i][j] = 1;
            }
        }
    }
    return A;
}

// Generic function to display the matrix.  We use it to
// display both adjoin and inverse. adjoin is integer matrix
// and inverse is a float.
//template <class T>
//void display(vector<vector<T>> &A)
//{
 //   for (int i = 0; i < A.size(); i++)
  //  {
   //     for (int j = 0; j < A.size(); j++)
    //        cout << A[i][j] << " ";
     //   cout << endl;
   // }
//}
