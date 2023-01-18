#include "magneto1.4.h"

#include <math.h>
#include <malloc.h>
#include "mymathlib_matrix.h"

using namespace mymathlib::matrix;

void MagnetoCalibration::sample(double x, double y, double z) {
    sample_count += 1.0;
    norm_sum += sqrt(x * x + y * y + z * z);

    double D[10] = {
        x * x,
        y * y,
        z * z,
        2.0 * y * z,
        2.0 * x * z,
        2.0 * x * y,
        2.0 * x,
        2.0 * y,
        2.0 * z,
        1.0
    };

    Multiply_Self_Transpose(ata, D, 10, 1);
}

void MagnetoCalibration::current_calibration(float BAinv[4][3]) {
    double* S11 = new double[6 * 6];
    Get_Submatrix(S11, 6, 6, ata, 10, 0, 0);
    double* S12 = new double[6 * 4];
    Get_Submatrix(S12, 6, 4, ata, 10, 0, 6);
    double* S12t = new double[4 * 6];
    Get_Submatrix(S12t, 4, 6, ata, 10, 6, 0);
    double* S22 = new double[4 * 4];
    Get_Submatrix(S22, 4, 4, ata, 10, 6, 6);

    double hm = norm_sum / sample_count;

    // this is where we'd deallocate ata or the entire calibration class
    // if we decided to compute the calibration destructively

    Choleski_LU_Decomposition(S22, 4);
    Choleski_LU_Inverse(S22, 4);

    // Calculate S22a = S22 * S12t   4*6 = 4x4 * 4x6   C = AB
    double* S22a = new double[4 * 6];
    Multiply_Matrices(S22a, S22, 4, 4, S12t, 6);
    delete[] S22;
    delete[] S12t;

    // Then calculate S22b = S12 * S22a      ( 6x6 = 6x4 * 4x6)
    double* S22b = new double[6 * 6];
    Multiply_Matrices(S22b, S12, 6, 4, S22a, 6);
    delete[] S12;

    // Calculate SS = S11 - S22b
    double* SS = new double[6 * 6];
    for (int i = 0; i < 36; i++)
        SS[i] = S11[i] - S22b[i];
    delete[] S11;
    delete[] S22b;

    // Create pre-inverted constraint matrix C
    double* C = new double[6 * 6]{
        0.0, 0.5, 0.5,  0.0,  0.0,  0.0,
        0.5, 0.0, 0.5,  0.0,  0.0,  0.0,
        0.5, 0.5, 0.0,  0.0,  0.0,  0.0,
        0.0, 0.0, 0.0, -0.25, 0.0,  0.0,
        0.0, 0.0, 0.0,  0.0, -0.25, 0.0,
        0.0, 0.0, 0.0,  0.0,  0.0, -0.25
    };
    double* E = new double[6 * 6];
    Multiply_Matrices(E, C, 6, 6, SS, 6);
    delete[] C;
    delete[] SS;

    double* SSS = new double[6 * 6];
    Hessenberg_Form_Elementary(E, SSS, 6);

    int index = 0;
    {
        double eigen_real[6];
        double eigen_imag[6];

        QR_Hessenberg_Matrix(E, SSS, eigen_real, eigen_imag, 6, 100);
        delete[] E;

        double maxval = eigen_real[0];
        for (int i = 1; i < 6; i++)
        {
            if (eigen_real[i] > maxval)
            {
                maxval = eigen_real[i];
                index = i;
            }
        }
    }

    double* v1 = new double[6];
    v1[0] = SSS[index];
    v1[1] = SSS[index + 6];
    v1[2] = SSS[index + 12];
    v1[3] = SSS[index + 18];
    v1[4] = SSS[index + 24];
    v1[5] = SSS[index + 30];
    delete[] SSS;

    // normalize v1
    {
        double norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] + v1[3] * v1[3] + v1[4] * v1[4] + v1[5] * v1[5]);
        v1[0] /= norm;
        v1[1] /= norm;
        v1[2] /= norm;
        v1[3] /= norm;
        v1[4] /= norm;
        v1[5] /= norm;
    }

    if (v1[0] < 0.0)
    {
        v1[0] = -v1[0];
        v1[1] = -v1[1];
        v1[2] = -v1[2];
        v1[3] = -v1[3];
        v1[4] = -v1[4];
        v1[5] = -v1[5];
    }

    // Calculate v2 = S22a * v1      ( 4x1 = 4x6 * 6x1)
    double* v2 = new double[4];
    Multiply_Matrices(v2, S22a, 4, 6, v1, 1);
    delete[] S22a;

    double* U = new double[3];
    double* Q = new double[3 * 3];
    double J;
    {
        double v[10];
        v[0] = v1[0];
        v[1] = v1[1];
        v[2] = v1[2];
        v[3] = v1[3];
        v[4] = v1[4];
        v[5] = v1[5];
        delete[] v1;
        v[6] = -v2[0];
        v[7] = -v2[1];
        v[8] = -v2[2];
        v[9] = -v2[3];
        delete[] v2;

        Q[0] = v[0];
        Q[1] = v[5];
        Q[2] = v[4];
        Q[3] = v[5];
        Q[4] = v[1];
        Q[5] = v[3];
        Q[6] = v[4];
        Q[7] = v[3];
        Q[8] = v[2];

        U[0] = v[6];
        U[1] = v[7];
        U[2] = v[8];

        J = v[9];
    }

    double* B = new double[3];
    {
        double Q_1[3 * 3];
        for (int i = 0; i < 9; i++)
            Q_1[i] = Q[i];
        Choleski_LU_Decomposition(Q_1, 3);
        Choleski_LU_Inverse(Q_1, 3);

        // Calculate B = Q-1 * U   ( 3x1 = 3x3 * 3x1)
        Multiply_Matrices(B, Q_1, 3, 3, U, 1);
        delete[] U;
        B[0] = -B[0];     // x-axis combined bias
        B[1] = -B[1];     // y-axis combined bias
        B[2] = -B[2];     // z-axis combined bias
    }

    // First calculate QB = Q * B   ( 3x1 = 3x3 * 3x1)
    double btqb;
    {
        double QB[3];
        Multiply_Matrices(QB, Q, 3, 3, B, 1);

        // Then calculate btqb = BT * QB    ( 1x1 = 1x3 * 3x1)
        Multiply_Matrices(&btqb, B, 1, 3, QB, 1);
    }

    // Calculate SQ, the square root of matrix Q
    double* SSSS = new double[3 * 3];
    Hessenberg_Form_Elementary(Q, SSSS, 3);

    double* Dz = new double[3 * 3]{0};
    {
        double eigen_real3[3];
        double eigen_imag3[3];
        QR_Hessenberg_Matrix(Q, SSSS, eigen_real3, eigen_imag3, 3, 100);
        delete[] Q;

        Dz[0] = sqrt(eigen_real3[0]);
        Dz[4] = sqrt(eigen_real3[1]);
        Dz[8] = sqrt(eigen_real3[2]);
    }

    {
        // normalize eigenvectors
        double norm = sqrt(SSSS[0] * SSSS[0] + SSSS[3] * SSSS[3] + SSSS[6] * SSSS[6]);
        SSSS[0] /= norm;
        SSSS[3] /= norm;
        SSSS[6] /= norm;
        norm = sqrt(SSSS[1] * SSSS[1] + SSSS[4] * SSSS[4] + SSSS[7] * SSSS[7]);
        SSSS[1] /= norm;
        SSSS[4] /= norm;
        SSSS[7] /= norm;
        norm = sqrt(SSSS[2] * SSSS[2] + SSSS[5] * SSSS[5] + SSSS[8] * SSSS[8]);
        SSSS[2] /= norm;
        SSSS[5] /= norm;
        SSSS[8] /= norm;
    }

    double* SQ = new double[3 * 3];
    {
        double vdz[3 * 3];
        Multiply_Matrices(vdz, SSSS, 3, 3, Dz, 3);
        delete[] Dz;
        Transpose_Square_Matrix(SSSS, 3);
        Multiply_Matrices(SQ, vdz, 3, 3, SSSS, 3);
        delete[] SSSS;
    }


    double* A_1 = new double[3 * 3];
    // Calculate hmb = sqrt(btqb - J).
    double hmb = sqrt(btqb - J);

    for (int i = 0; i < 9; i++)
        A_1[i] = SQ[i] * hm / hmb;
    delete[] SQ;

    for (int i = 0; i < 3; i++)
        BAinv[0][i] = B[i];
    delete[] B;

    for (int i = 0; i < 3; i++)
    {
        BAinv[i + 1][0] = A_1[i * 3];
        BAinv[i + 1][1] = A_1[i * 3 + 1];
        BAinv[i + 1][2] = A_1[i * 3 + 2];
    }
    delete[] A_1;
}
