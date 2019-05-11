/**
  ******************************************************************************
  * @file           : Matrix.h
  * @brief          : Header for the Matrix module.
  ******************************************************************************
  ** This module contains matrix and vector math functions. Matrices contain r
	* by c arrays of floats.
	*
	* Created by Galen Savidge. Edited 2/23/2019.
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
 * @brief  Copies a Matrix to another allocated Matrix
 * @param  Matrix to be copied, Matrix to hold the copy
 * @return None
*/
void matrixCopy(Matrix m, Matrix copy);

/** 
 * @brief  Copies the elements in a 2D array to a Matrix of the same dimensions
 * @param  A Matrix, a 2D float array
 * @return None
*/
void matrixCopyArray(Matrix m, float** array);

/** 
 * @brief  Checks if two Matrix objects are exactly equal
 * @param  Two Matrix objects to be compared
 * @return Zero if not equal, nonzero if equal
*/
char matrixEquals(Matrix m1, Matrix m2);

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
float matrixGetElement(Matrix m, int r, int c);

/** 
 * @brief  Sets an element in a Matrix
 * @param  A Matrix, row of the element, column of the element (indexed starting at 1), float value for the element
 * @return None
*/
void matrixSet(Matrix m, int r, int c, float val);

/** 
 * @brief  Multiplies every element of a Matrix by x
 * @param  A Matrix and a float x
 * @return None
*/
void matrixScale(Matrix m, float x);

/** 
 * @brief  Adds x to every element in a Matrix
 * @param  A Matrix and a float x
 * @return None
*/
void matrixAddScalar(Matrix m, float x);

/** 
 * @brief  Adds two Matrix objects
 * @param  Two Matrix objects to be added, a third allocated Matrix in which to store the result
 * @return None
*/
void matrixAdd(Matrix m1, Matrix m2, Matrix sum);

/** 
 * @brief  Subtracts two Matrix objects
 * @param  Two Matrix objects to be subtracted, a third allocated Matrix in which to store the result
 * @return None
*/
void matrixSubtract(Matrix m1, Matrix m2, Matrix diff);

/** 
 * @brief  Multiplies two Matrix objects
 * @param  Two Matrix objects to be multiplied, a third allocated Matrix in which to store the result
 * @return None
*/
void matrixMult(Matrix m1, Matrix m2, Matrix prod);

/** 
 * @brief  Copies and transposes a Matrix
 * @param  A Matrix, an allocated Matrix in which to store the transpose
 * @return None
*/
void matrixTranspose(Matrix m, Matrix mt);

/** 
 * @brief  Allocates a 3x1 column vector Matrix and populates it with x, y, and z
 * @param  Vector components
 * @return A newly allocated column vector Matrix
*/
Matrix make3x1Vector(float x, float y, float z);

/** 
 * @brief  Copies elements from a 1xn array to an allocated nx1 column vector Matrix
 * @param  v: an allocated nx1 Matrix
 * @param  array: 1xn array containing vector elements
 * @param  n: vector order (number of elements)
 * @return A newly allocated column vector Matrix
*/
void vectorCopyArray(Matrix v, float* array, char n);

/** 
 * @brief  Sets elements 1-3 in a vector
 * @param  v: an allocated 3x1 Matrix
 * @param  x, y, z: vector components
 * @return None
*/
void vectorSetXYZ(Matrix v, float x, float y, float z);

/** 
 * @brief  Creates the skew symmetric (cross product) matrix from a vector
 * @param  A 3x1 column vector Matrix, an allocated 3x3 Matrix in which to store the result
 * @return None
*/
void vectorRcross(Matrix v, Matrix vcross);

/** 
 * @brief  Calculates the cross product of two column vectors
 * @param  v1 & v2: Two column vector Matrix objects with the same dimensions
 * @param  v1xv2: An allocated column vector Matrix to hold the result
 * @return None
*/
void vectorCrossProduct(Matrix v1, Matrix v2, Matrix v1xv2);

/** 
 * @brief  Calculates the dot product of two column vectors
 * @param  Two column vector Matrices with the same dimensions
 * @return The dot product v1*v2
*/
float vectorDotProduct(Matrix v1, Matrix v2);

/** 
 * @brief  Calculates the norm of a vector
 * @param  A column vector Matrix
 * @return The norm of v
*/
float vectorNorm(Matrix v);

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
