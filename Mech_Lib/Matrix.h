/**
  ******************************************************************************
  * @file           Matrix.h
  * @brief          A library of matrix and vector math functions
  ******************************************************************************
  ** This module contains matrix and vector math functions. To use, first 
	* allocate  Matrix objects by using newMatrix(), or another function that 
	* retuns a Matrix. Next, pass this Matrix into the other functions in the 
	* library. A Matrix is a pointer to an allocated struct stored on the heap. 
	* Matrices contain r by c arrays of floats.
	*
	* Created by Galen Savidge. Edited 5/12/2019.
  ******************************************************************************
  */

#ifndef MATRIX_H
#define	MATRIX_H


/* Public types ------------------------------------------------------------*/
typedef struct _Matrix* Matrix;


/* Public functions prototypes ---------------------------------------------*/

/** 
 * @brief  Allocates a new Matrix
 * @param  r: number of rows
 * @param  c: number of columns
 * @return Pointer to the new Matrix
*/
Matrix newMatrix(int r, int c);

/** 
 * @brief  Copies a Matrix to another Matrix
 * @param  m: Matrix to be copied
 * @param  copy: initialized Matrix with the same dimensions as m
 * @return copy is now a copy of m
*/
void matrixCopy(Matrix m, Matrix copy);

/** 
 * @brief  Copies the elements in a 2D array to a Matrix of the same dimensions
 * @param  m: initialized Matrix
 * @param  array: a 2D array of floats with dimensions >= m
 * @return m now holds the values from array
*/
void matrixCopyArray(Matrix m, float** array);

/** 
 * @brief  Checks if two Matrix objects are exactly equal
 * @param  m1: first Matrix to be compared
 * @param  m2: second Matrix to be compared
 * @return Nonzero if m1 and m2 have the same dimensions and stored values, zero otherwise
*/
char matrixEquals(Matrix m1, Matrix m2);

/** 
 * @brief  Returns the number of rows in a Matrix
 * @param  m: an initialized Matrix
 * @return The number of rows in m
*/
int matrixGetRows(Matrix m);

/** 
 * @brief  Returns the number of columns in a Matrix
 * @param  m: an initialized Matrix
 * @return The number of columns in m
*/
int matrixGetCol(Matrix m);

/** 
 * @brief  Returns an element in a Matrix
 * @param  m: an initialized Matrix
 * @param  r: row of the element (indexed at 1)
 * @param  c: column of the element (indexed at 1)
 * @return Element (r, c) of the Matrix
*/
float matrixGetElement(Matrix m, int r, int c);

/** 
 * @brief  Sets an element in a Matrix
 * @param  m: an initialized Matrix
 * @param  r: row of the element (indexed at 1)
 * @param  c: column of the element (indexed at 1)
 * @param  val: floating point value for the element
 * @return None
*/
void matrixSet(Matrix m, int r, int c, float val);

/** 
 * @brief  Multiplies every element of a Matrix by the scalar x
 * @param  m: an initialized Matrix
 * @param  x: a scalar value
 * @return None
*/
void matrixScale(Matrix m, float x);

/** 
 * @brief  Adds x to every element in a Matrix
 * @param  m: an initialized Matrix
 * @param  x: a scalar value
 * @return None
*/
void matrixAddScalar(Matrix m, float x);

/** 
 * @brief  Adds two Matrix objects
 * @param  m1: first Matrix to be added
 * @param  m2: second Matrix to be added
 * @param  sum: initialized Matrix in which to store the result
 * @return sum now contains m1 + m2
*/
void matrixAdd(Matrix m1, Matrix m2, Matrix sum);

/** 
 * @brief  Adds two Matrix objects
 * @param  m1: the minuend
 * @param  m2: the subtrahend
 * @param  sum: initialized Matrix in which to store the difference
 * @return sum now contains m1 - m2
*/
void matrixSubtract(Matrix m1, Matrix m2, Matrix diff);

/** 
 * @brief  Multiplies m1 by m2 using standard matrix multiplication
 * @param  m1: Matrix of size nxm
 * @param  m2: Matrix of size mxp
 * @param  prod: initialized Matrix in which to store the result
 * @return prod now contains m1*m2
*/
void matrixMult(Matrix m1, Matrix m2, Matrix prod);

/** 
 * @brief  Copies and transposes a Matrix
 * @param  m: Matrix to be transposed
 * @param  mt: initialized Matrix in which to store the result
 * @return mt is the transpose of m
*/
void matrixTranspose(Matrix m, Matrix mt);

/** 
 * @brief  Allocates a 3x1 column vector Matrix and populates it with x, y, and z
 * @param  x: vector x component
 * @param  y: vector y component
 * @param  z: vector z component
 * @return A newly allocated column vector Matrix containing [x; y; z]
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
 * @param  x: vector x component
 * @param  y: vector y component
 * @param  z: vector z component
 * @return v now contains [x; y; z]
*/
void vectorSetXYZ(Matrix v, float x, float y, float z);

/** 
 * @brief  Creates the skew symmetric (cross product) matrix from a vector
 * @param  v: a 3x1 column vector Matrix
 * @param  vcross: an initialized 3x3 Matrix
 * @return vcross is now the skew symmetric matrix of v
*/
void vectorRcross(Matrix v, Matrix vcross);

/** 
 * @brief  Calculates the cross product of two column vectors
 * @param  v1: an nx1 column vector Matrix
 * @param  v2: an nx1 column vector Matrix
 * @param  v1xv2: An allocated nx1 Matrix to hold the result
 * @return v1xv2 contains the cross product v1 x v2
*/
void vectorCrossProduct(Matrix v1, Matrix v2, Matrix v1xv2);

/** 
 * @brief  Calculates the dot product of two column vectors
 * @param  v1: an nx1 column vector Matrix
 * @param  v2: an nx1 column vector Matrix
 * @return The dot product v1*v2
*/
float vectorDotProduct(Matrix v1, Matrix v2);

/** 
 * @brief  Calculates the norm of a vector
 * @param  v: a column vector Matrix
 * @return The norm of v
*/
float vectorNorm(Matrix v);

/** 
 * @brief  Prints a Matrix to a string in a format suitable for printing over serial
 * @param  m: an initialized Matrix with r row and c columns
 * @param  string: a char array of size at least (20*r*c + 3)
 * @return string contains a printed version of m
*/
void printMatrix(Matrix m, char* string);

#endif /* MATRIX_H */
