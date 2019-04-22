/**
  ******************************************************************************
  * @file           : Matrix.c
  * @brief          : Source file for the Matrix module.
  ******************************************************************************
  ** This module contains matrix and vector math functions. Matrices contain r
	* by c arrays of floats.
	*
	* Created by Galen Savidge. Edited 2/23/2019.
  ******************************************************************************
  */

#include <Matrix.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// Matrix struct
struct _Matrix {
    int r, c;
    float** data;
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
    m->data = calloc(r, sizeof(float*));
    for(int i = 0;i < r;i++) {
        m->data[i] = calloc(c, sizeof(float));
        for(int j = 0;j < c;j++) {
            m->data[i][j] = 0;
        }
    }
    
    return m;
}

void matrixCopy(Matrix m, Matrix copy) {
    for(int i = 0;i < m->r;i++) {
        for(int j = 0;j < m->c;j++) {
            copy->data[i][j] = m->data[i][j];
        }
    }
}

void matrixCopyArray(Matrix m, float** array) {
    for(int i = 0;i < m->r;i++) {
        for(int j = 0;j < m->c;j++) {
            m->data[i][j] = array[i][j];
        }
    }
}

char matrixEquals(Matrix m1, Matrix m2) {
    if(m1->r != m2->r || m1->c != m2->c) {
        return 0;
    }
    for(int i = 0;i < m1->r;i++) {
        for(int j = 0;j < m1->c;j++) {
            if(m1->data[i][j] != m2->data[i][j]) {
                return 0;
            }
        }
    }
    return 1;
}

int matrixGetRows(Matrix m) {
    return m->r;
}

int matrixGetCol(Matrix m) {
    return m->c;
}

float matrixGetElement(Matrix m, int r, int c) {
    return m->data[r-1][c-1];
}

void matrixSet(Matrix m, int r, int c, float val) {
    m->data[r-1][c-1] = val;
}

void matrixScale(Matrix m, float x) {
    for(int i = 1;i <= m->r;i++) {
        for(int j = 1;j <= m->c;j++) {
            matrixSet(m, i, j, matrixGetElement(m, i, j)*x);
        }
    }
}

void matrixAddScalar(Matrix m, float x) {
    for(int i = 1;i <= m->r;i++) {
        for(int j = 1;j <= m->c;j++) {
            matrixSet(m, i, j, matrixGetElement(m, i, j) + x);
        }
    }
}

void matrixAdd(Matrix m1, Matrix m2, Matrix sum) {
    if(m1->r == m2->r && m1->c == m2->c) {
        for(int i = 0;i < m1->r;i++) {
            for(int j = 0;j < m1->c;j++) {
                float v = m1->data[i][j] + m2->data[i][j];
                sum->data[i][j] = v;
            }
        }
    }
    else {
			printf("MATRIX MATH ERROR: ADD SIZE MISMATCH");
			while(1);
		}
}

void matrixSubtract(Matrix m1, Matrix m2, Matrix diff) {
    if(m1->r == m2->r && m1->c == m2->c) {
        for(int i = 0;i < m1->r;i++) {
            for(int j = 0;j < m1->c;j++) {
                float v = m1->data[i][j] - m2->data[i][j];
                diff->data[i][j] = v;
            }
        }
    }
    else {
			printf("MATRIX MATH ERROR: ADD SIZE MISMATCH");
			while(1);
		}
}


void matrixMult(Matrix m1, Matrix m2, Matrix prod) {
    if(m1->c != m2->r || m1->r != prod->r || m2->c != prod->c) {
        printf("MATRIX MATH ERROR: MULT SIZE MISMATCH");
        while(1);
    }
    
    for(int i = 0;i < m1->r;i++) {
        for(int j = 0;j < m2->c;j++) {
            float sum = 0;
            for(int k = 0;k < m1->c;k++) {
                sum += m1->data[i][k]*m2->data[k][j];
            }
            
            prod->data[i][j] = sum;
        }
    }
}

void matrixTranspose(Matrix m, Matrix mt) {
    for(int i = 1;i <= m->r;i++) {
        for(int j = 1;j <= m->c;j++) {
            matrixSet(mt, j, i, matrixGetElement(m, i, j));
        }
    }
}

Matrix make3x1Vector(float x, float y, float z) {
	Matrix v = newMatrix(3, 1);
	v->data[0][0] = x;
	v->data[1][0] = y;
	v->data[2][0] = z;
	return v;
}

void vectorCopyArray(Matrix v, float* array, char n) {
	for(int i = 0;i < n;i++) {
		v->data[i][0] = array[i];
	}
}

void vectorSetXYZ(Matrix v, float x, float y, float z) {
	v->data[0][0] = x;
	v->data[1][0] = y;
	v->data[2][0] = z;
}

void vectorRcross(Matrix v, Matrix vcross) {
    // Row 1
    vcross->data[0][0] = 0;
    vcross->data[0][1] = -v->data[2][0]; // -z
    vcross->data[0][2] = v->data[1][0]; // y
    
    // Row 2
    vcross->data[1][0] = v->data[2][0]; // z
    vcross->data[1][1] = 0;
    vcross->data[1][2] = -v->data[0][0]; // -x
    
    // Row 3
    vcross->data[2][0] = -v->data[1][0]; // -y
    vcross->data[2][1] = v->data[0][0]; // x
    vcross->data[2][2] = 0;
}

void vectorCrossProduct(Matrix v1, Matrix v2, Matrix v1xv2) {
	v1xv2->data[0][0] = v1->data[1][0]*v2->data[2][0] - v1->data[2][0]*v2->data[1][0];
	v1xv2->data[1][0] = v1->data[2][0]*v2->data[0][0] - v1->data[0][0]*v2->data[2][0];
	v1xv2->data[2][0] = v1->data[0][0]*v2->data[1][0] - v1->data[1][0]*v2->data[0][0];
}

float vectorDotProduct(Matrix v1, Matrix v2) {
	float sum = 0;
	for(int i = 0;i < v1->r;i++) {
		sum += v1->data[i][0]*v2->data[i][0];
	}
	return sum;
}

float vectorNorm(Matrix v) {
    float sum = 0;
    for(int i = 0;i < v->r;i++) {
        sum += pow(v->data[i][0], 2);
    }
    
    return sqrt(sum);
}

void printMatrix(Matrix m, char* string) {
    int r = matrixGetRows(m);
    int c = matrixGetCol(m);
    string[0] = '\0'; // Make string an empty string
    for(int i = 1;i <= r;i++) {
        char temp[14];
        for(int j = 1;j <= c;j++) {
            sprintf(temp, "%8.4e\t", matrixGetElement(m, i, j));
				    strcat(string, temp); // Add temp to the end of string
        }
        sprintf(temp, "\r\n");
				strcat(string, temp);
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
