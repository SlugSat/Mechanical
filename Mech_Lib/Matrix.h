/**
  ******************************************************************************
  * @file           : Matrix.h
  * @brief          : Header for the Matrix module.
  ******************************************************************************
  ** This module contains matrix and vector math functions. Matrices contain r
	* by c arrays of doubles.
	*
	* Created by Galen Savidge. Edited 2/16/2019.
  ******************************************************************************
  */

#ifndef MATRIX_H
#define	MATRIX_H

/* Public types ------------------------------------------------------------*/
typedef struct _Matrix* Matrix;

/* Public functions prototypes ---------------------------------------------*/

/** 
 * @brief  Allocates a new Matrix
 * @param  Number of rows and columns
 * @return Pointer to the new Matrix
*/
Matrix newMatrix(int r, int c);

/** 
 * @brief  Copies a Matrix to a newly allocated Matrix
 * @param  Matrix to be copied
 * @return Copy Matrix
*/
Matrix matrixCopy(Matrix m);

/** 
 * @brief  Returns the number of rows in a Matrix
 * @param  A Matrix
 * @return Integer number of rows
*/
int matrixGetRows(Matrix m);

/** 
 * @brief  Returns the number of columns in a Matrix
 * @param  A Matrix
 * @return Integer number of columns
*/
int matrixGetCol(Matrix m);

/** 
 * @brief  Returns an element in a Matrix
 * @param  A Matrix, row of the element, column of the element (indexed starting at 1)
 * @return Element (r, c) of the Matrix
*/
double matrixGetElement(Matrix m, int r, int c);

/** 
 * @brief  Sets an element in a Matrix
 * @param  A Matrix, row of the element, column of the element (indexed starting at 1), double value for the element
 * @return None
*/
void matrixSet(Matrix m, int r, int c, double val);

/** 
 * @brief  Multiplies every element of a Matrix by x
 * @param  A Matrix and a double x
 * @return None
*/
void matrixScale(Matrix m, double x);

/** 
 * @brief  Adds x to every element in a Matrix
 * @param  A Matrix and a double x
 * @return None
*/
void matrixAddScalar(Matrix m, double x);

/** 
 * @brief  Adds two Matrix objects
 * @param  Two Matrix objects to be added
 * @return A newly allocated Matrix which is the sum M1+M2
*/
Matrix matrixAdd(Matrix m1, Matrix m2);

/** 
 * @brief  Multiplies two Matrix objects
 * @param  Two Matrix objects to be multiplied
 * @return A newly allocated Matrix which is the matrix product M1M2
*/
Matrix matrixMult(Matrix m1, Matrix m2);

/** 
 * @brief  Copies and transposes a Matrix
 * @param  A Matrix
 * @return A newly allocated Matrix which is M transposed
*/
Matrix matrixTranspose(Matrix m);

/** 
 * @brief  Prints a Matrix to a string in a format suitable for printing over serial
 * @param  A Matrix and a char pointer with enough room to hold the printed Matrix
 * @return None
*/
void printMatrix(Matrix m, char* string);

/** 
 * @brief  Frees a Matrix from memory
 * @param  A Matrix
 * @return None
 */
void freeMatrix(Matrix *mp);

#endif /* MATRIX_H */
