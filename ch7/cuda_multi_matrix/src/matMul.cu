#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "cublas_v2.h"

#define M 512
#define K 512
#define N 512
// Tread block size
#define BLOCK_SIZE 16

typedef struct{
    int width;
    int height;
    int stride;
    float *elements;
}Matrix;

// Get a matrix element
__device__ float GetElement(const Matrix A, int row, int col){
    return A.elements[row*A.stride+col];
}

// Set a matrix element
__device__ void SetElement(Matrix A, int row, int col, float value){
    A.elements[row*A.stride+col]=value;
}

// Get the BLOCK_SIZE \times BLOCK_SIZE sub-matrix Asub of A that is
// located col sub-matrices to the right and row sub-matrices down 
// from the upper-left corner of A
__device__ Matrix GetSubMatrix(Matrix A, int row, int col){
    Matrix Asub;
    Asub.width = BLOCK_SIZE;
    Asub.height = BLOCK_SIZE;
    Asub.stride = A.stride;
    Asub.elements = &A.elements[row*A.stride*BLOCK_SIZE+BLOCK_SIZE*col];
    return Asub;
}


// Forward declaration - Host code
// Matrix dimensions are assumed to be multiples of BLOCK_SIZE
__global__ void MatMulKernel(const Matrix, const Matrix, Matrix);

// Matrix multiplication - Host code
// Matrix dimensions are assumed to be multiples of BLOCK_SIZE

void MatMul(const Matrix A, const Matrix B, Matrix C){
    // Load A and B to device memory
    Matrix d_A;
    d_A.width = A.width;
    d_A.stride = A.width;
    d_A.height = A.height;
    size_t size = A.width*A.height*sizeof(float);
    cudaMalloc(&d_A.elements, size);
    cudaMemcpy(d_A.elements, A.elements, size, cudaMemcpyHostToDevice);

    Matrix d_B;
    d_B.width = B.width;
    d_B.stride = B.width;
    d_B.height = B.height;
    size = B.width*B.height*sizeof(float);
    cudaMalloc(&d_B.elements, size);
    cudaMemcpy(d_B.elements, B.elements, size, cudaMemcpyHostToDevice);
    
    //Allocate C in device memory
    Matrix d_C;
    d_C.width = C.width;
    d_C.stide = C.width;
    d_C.height = C.height;
    size = C.width*C.height*sizeof(float);
    cudaMalloc(&d_C.elements, size);

    // Invoke kernel
    dim3 dimBlock(BLOCK_SIZE, BLOCK_SIZE);
    dim3 dimGrid(B.width/dimBlock.x, A.height/dimBlock.y);
    MatMulKernel<<<dimGrid, dimBlock>>>(d_A, d_B, d_C);

    // Read C from Device memory
    cudaMemcpy(C.elements, d_C.elements, size, cudaMemcpyDeviceToHost);

    // Free device Memory
    cudaFree(d_A.elements);
    cudaFree(d_B.elements);
    cudaFree(d_C.elements);
}

// Matrix multiplication kernel called by MatMul()
__global__ void MatMulKernel(Matrix A, Matrix B, Matrix C){
    // Block row and Column
    int blockRow = blockIdx.y;
    int blockCol = blockIdx.x;

    // Each thread block computes one sub-matrix Csub of C
    Matrix Csub = GetSubMatrix(C, blockRow, blockCol);

    // Each thread computes one element of Csub by accumulating results into Cvalue
    float Cvalue = 0;

    // Thread row and column within Csub
    int row = threadIdx.y;
    int col = threadIdx.x;

    for(int m=0; m<(A.width/BLOCK_SIZE); ++m){
        Matrix Asub = GetSubMatrix(A, blockRow, m);
        Matrix Bsub = GetSubMatrix(B, m, blockCol);

        __share__ float As[BLOCK_SIZE][BLOCK_SIZE];
        __share__ float Bs[BLOCK_SIZE][BLOCK_SIZE];

        As[row][col] = GetElement(Asub, row, col);
        Bs[row][col] = GetElement(Bsub, row, col);
        __syncthreads();

        for(int e=0; e<BLOCK_SIZE; ++e)
            Cvalue += As[row][e]*Bs[e][col];
        
        __syncthreads();
        }
    SetElement(Csub, row, col, Cvalue);
}
