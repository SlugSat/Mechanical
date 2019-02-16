/**
  ******************************************************************************
  * @file           : Matrix.c
  * @brief          : Source file for the Matrix module.
  ******************************************************************************
  ** This module contains matrix and vector math functions. Matrices contain r
	* by c arrays of doubles.
	*
	* Created by Galen Savidge. Edited 2/16/2019.
  ******************************************************************************
  */

#include<Matrix.h>
#include<stdlib.h>
#include<stdio.h>

// Matrix struct
struct _Matrix {
    int r, c;
    double** data;
};

Matrix newMatrix(int r, int c) {
    // Allocate a _Matrix struct
    Matrix m = malloc(sizeof(struct _Matrix));
    
    if(m == NULL) {
        printf("MATRIX ERROR: FAILED TO ALLOCATE");
        while(1);
    }
    
    // Initialize the internal variables and allocate the data array
    m->r = r;
    m->c = c;
    m->data = calloc(r, sizeof(double*));
    for(int i = 0;i < r;i++) {
        m->data[i] = calloc(c, sizeof(double));
        for(int j = 0;j < c;j++) {
            m->data[i][j] = 0;
        }
    }
    
    return m;
}

Matrix matrixCopy(Matrix m) {
    Matrix n = newMatrix(m->r, m->c);
    for(int i = 0;i < m->r;i++) {
        for(int j = 0;j< m->c;j++) {
            matrixSet(n, i, j, matrixGetElement(m, i, j));
        }
    }
    
    return n;
}

int matrixGetRows(Matrix m) {
    return m->r;
}

int matrixGetCol(Matrix m) {
    return m->c;
}

double matrixGetElement(Matrix m, int r, int c) {
    return m->data[r-1][c-1];
}

void matrixSet(Matrix m, int r, int c, double val) {
    m->data[r-1][c-1] = val;
}

void matrixScale(Matrix m, double x) {
    for(int i = 1;i <= m->r;i++) {
        for(int j = 1;j <= m->c;j++) {
            matrixSet(m, i, j, matrixGetElement(m, i, j)*x);
        }
    }
}

void matrixAddScalar(Matrix m, double x) {
    for(int i = 1;i <= m->r;i++) {
        for(int j = 1;j <= m->c;j++) {
            matrixSet(m, i, j, matrixGetElement(m, i, j) + x);
        }
    }
}

Matrix matrixAdd(Matrix m1, Matrix m2) {
    if(m1->r == m2->r && m1->c == m2->c) {
        Matrix sum = newMatrix(m1->r, m1->c);
        for(int i = 0;i < m1->r;i++) {
            for(int j = 0;j < m1->c;j++) {
                float v = matrixGetElement(m1, i, j) + matrixGetElement(m2, i, j);
                matrixSet(sum, i, j, v);
            }
        }
        return sum;
    }
    
    printf("MATRIX MATH ERROR: ADD SIZE MISMATCH");
    while(1);
}

Matrix matrixMult(Matrix m1, Matrix m2) {
    if(m1->c != m2->r) {
        printf("MATRIX MATH ERROR: MULT SIZE MISMATCH");
        while(1);
    }
    
    Matrix prod = newMatrix(m1->r, m2->c);
    
    for(int i = 1;i <= m1->r;i++) {
        for(int j = 1;j <= m2->c;j++) {
            float sum = 0;
            for(int k = 1;k <= m1->c;k++) {
                sum += matrixGetElement(m1, i, k)*matrixGetElement(m2, k, j);
            }
            
            matrixSet(prod, i, j, sum);
        }
    }
    
    return prod;
}

Matrix matrixTranspose(Matrix m) {
    Matrix mt = newMatrix(m->c, m->r);
    for(int i = 1;i <= m->r;i++) {
        for(int j = 1;j <= m->c;j++) {
            matrixSet(mt, j, i, matrixGetElement(m, i, j));
        }
    }
    
    return mt;
}

void printMatrix(Matrix m, char* string) {
    int r = matrixGetRows(m);
    int c = matrixGetCol(m);
    for(int i = 1;i <= r;i++) {
        for(int j = 1;j <= c;j++) {
            sprintf(string, "%8.4f\t", matrixGetElement(m, i, j));
        }
        sprintf(string, "\r\n");
    }
}

void freeMatrix(Matrix *mp) {
    if(mp == NULL || *mp == NULL) {
        printf("MATRIX FREE ERROR: NULL POINTER\r\n");
        while(1);
    }
    
    for(int i = 0;i < (*mp)->r;i++) {
        free((*mp)->data[i]);
    }
    free((*mp)->data);
    free(*mp);
    *mp = NULL;
}
