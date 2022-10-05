#include "magneto1.4.h"

#include <math.h>
#include <malloc.h>
#include "mymathlib_matrix.h"

using namespace mymathlib::matrix;

//Magneto 1.3 4/24/2020
void CalculateCalibration(float *buf, int sampleCount, float BAinv[4][3])
{
    double xs = 0, xave = 0;

    //
    // calculate mean (norm) and standard deviation for possible outlier rejection
    //
    for (int i = 0; i < sampleCount; i++)
    {
        double x = buf[i * 3 + 0];
        double y = buf[i * 3 + 1];
        double z = buf[i * 3 + 2];
        double x2 = x * x + y * y + z * z;
        xs += x2;
        xave += sqrt(x2);
    }
    xave = xave / sampleCount; //mean vector length
    xs = sqrt(xs / sampleCount - (xave * xave)); //std. dev.

    // third time through!
    // allocate array space for accepted measurements
    double* D = new double[10 * sampleCount];
    double* raw = new double[3 * sampleCount];

    {
        // summarize statistics, give user opportunity to reject outlying measurements
        double nxsrej = 0;

        int j = 0;  //array index for good measurements
        // printf("\r\nAccepted measurements (file index, internal index, ...)\r\n");
        for (int i = 0; i < sampleCount; i++)
        {
            double x = buf[i * 3 + 0];
            double y = buf[i * 3 + 1];
            double z = buf[i * 3 + 2];
            double x2 = sqrt(x * x + y * y + z * z);  //vector length
            x2 = fabs(x2 - xave) / xs; //standard deviation from mean
            if ((nxsrej == 0) || (x2 <= nxsrej)) {
                // accepted measurement
                //   printf("%d, %d: %6.1f %6.1f %6.1f\r\n",i,j,x,y,z);

                raw[3 * j] = x;
                raw[3 * j + 1] = y;
                raw[3 * j + 2] = z;
                D[j] = x * x;
                D[sampleCount + j] = y * y;
                D[sampleCount * 2 + j] = z * z;
                D[sampleCount * 3 + j] = 2.0 * y * z;
                D[sampleCount * 4 + j] = 2.0 * x * z;
                D[sampleCount * 5 + j] = 2.0 * x * y;
                D[sampleCount * 6 + j] = 2.0 * x;
                D[sampleCount * 7 + j] = 2.0 * y;
                D[sampleCount * 8 + j] = 2.0 * z;
                D[sampleCount * 9 + j] = 1.0;
                j++; //count good measurements
            }
        }
    }
    delete[] raw;

    //printf("\r\nExpected norm of local field vector Hm? (Enter 0 for default %8.1f) ", xave);
    //scanf("%lf", &hm);

    //if (hm == 0.0) hm = xave;
    //printf("\r\nSet Hm = %8.1f\r\n", hm);
    double hm = xave;

    // allocate memory for matrix S
    double* S = new double[10 * 10];
    Multiply_Self_Transpose(S, D, 10, sampleCount);
    delete[] D;

    double* S11 = new double[6 * 6];
    Get_Submatrix(S11, 6, 6, S, 10, 0, 0);
    double* S12 = new double[6 * 4];
    Get_Submatrix(S12, 6, 4, S, 10, 0, 6);
    double* S12t = new double[4 * 6];
    Get_Submatrix(S12t, 4, 6, S, 10, 6, 0);
    double* S22 = new double[4 * 4];
    Get_Submatrix(S22, 4, 4, S, 10, 6, 6);
    delete[] S;

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


    // hm = 0.569;
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