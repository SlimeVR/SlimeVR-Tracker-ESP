#pragma once
#ifndef __MYMATHLIB_MATRIX_H__

/* Note: This code is from http://www.mymathlib.com/matrices/ and appears to have no license attached.
 * I doubt that anyone will pursue us for the use of this library with a copyright date of 2004
 * that's been distributed along with magneto for awhile, and the intent of the site seems to be to
 * provide code for people to use, but it's something to be aware of.
 */
namespace mymathlib::matrix {
    void Multiply_Self_Transpose(double*, double*, int, int);
    void Get_Submatrix(double*, int, int, double*, int, int, int);
    int Choleski_LU_Decomposition(double*, int);
    int Choleski_LU_Inverse(double*, int);
    void Multiply_Matrices(double*, double*, int, int, double*, int);
    void Identity_Matrix(double*, int);

    int Hessenberg_Form_Elementary(double*, double*, int);
    void Hessenberg_Elementary_Transform(double*, double*, int[], int);

    void Copy_Vector(double*, double*, int);

    int QR_Hessenberg_Matrix(double*, double*, double[], double[], int, int);
    void One_Real_Eigenvalue(double[], double[], double[], int, double);
    void Two_Eigenvalues(double*, double*, double[], double[], int, int, double);
    void Update_Row(double*, double, double, int, int);
    void Update_Column(double*, double, double, int, int);
    void Update_Transformation(double*, double, double, int, int);
    void Double_QR_Iteration(double*, double*, int, int, int, double*, int);
    void Product_and_Sum_of_Shifts(double*, int, int, double*, double*, double*, int);
    int Two_Consecutive_Small_Subdiagonal(double*, int, int, int, double, double);
    void Double_QR_Step(double*, int, int, int, double, double, double*, int);
    void BackSubstitution(double*, double[], double[], int);
    void BackSubstitute_Real_Vector(double*, double[], double[], int, double, int);
    void BackSubstitute_Complex_Vector(double*, double[], double[], int, double, int);
    void Calculate_Eigenvectors(double*, double*, double[], double[], int);
    void Complex_Division(double, double, double, double, double*, double*);

    void Transpose_Square_Matrix(double*, int);

    int Lower_Triangular_Solve(double* L, double B[], double x[], int n);
    int Lower_Triangular_Inverse(double* L, int n);
    int Upper_Triangular_Solve(double* U, double B[], double x[], int n);

    void Interchange_Rows(double* A, int row1, int row2, int ncols);
    void Interchange_Columns(double* A, int col1, int col2, int nrows, int ncols);
    void Identity_Matrix(double* A, int n);
    void Copy_Vector(double* d, double* s, int n);

    void Hessenberg_Elementary_Transform(double* H, double* S, int perm[], int n);

    void One_Real_Eigenvalue(double Hrow[], double eigen_real[], double eigen_imag[], int row, double shift);
    void Two_Eigenvalues(double* H, double* S, double eigen_real[], double eigen_imag[], int n, int k, double t);
    void Update_Row(double* Hrow, double cos, double sin, int n, int k);
    void Update_Column(double* H, double cos, double sin, int n, int k);
    void Update_Transformation(double* S, double cos, double sin, int n, int k);
    void Double_QR_Iteration(double* H, double* S, int row, int min_row, int n, double* shift, int iteration);
    void Product_and_Sum_of_Shifts(double* H, int n, int max_row, double* shift, double* trace, double* det, int iteration);
    int Two_Consecutive_Small_Subdiagonal(double* H, int min_row, int max_row, int n, double trace, double det);
    void Double_QR_Step(double* H, int min_row, int max_row, int min_col, double trace, double det, double* S, int n);
    void Complex_Division(double x, double y, double u, double v, double* a, double* b);
    void BackSubstitution(double* H, double eigen_real[], double eigen_imag[], int n);
    void BackSubstitute_Real_Vector(double* H, double eigen_real[], double eigen_imag[], int row, double zero_tolerance, int n);
    void BackSubstitute_Complex_Vector(double* H, double eigen_real[], double eigen_imag[], int row, double zero_tolerance, int n);
    void Calculate_Eigenvectors(double* H, double* S, double eigen_real[], double eigen_imag[], int n);
}

#endif