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

#define BLOCK_SIZE 32 // block size, each thread to calcualte each block

void initial(float *array, int size){
    for(size_t i=0; i<size; i++){
        array[i] = (float)(rand()%10+1);
    }
    return;
}

void printMatrix(float *array, int row, int col){
    float *p=array;
    for(size_t y=0; y<row; y++){
        for(size_t x=0; x<col; x++){
            printf("%10lf",p[x]);
        }
        p = p + col;
        printf("\n");
    }
    return;
}

void multiplicateMatrixOnHost(float *array_A, 
                              float *array_B, 
                              float *array_C, 
                              int M_p, 
                              int K_p, 
                              int N_p){
    for(size_t i=0; i<M_p; i++){
        for(size_t j=0; j<N_p; j++){
            float sum = 0;
            for(size_t k=0; k<K_p; k++){
                sum += array_A[i*K_p+k]*array_B[k*N_p+j];
            }
            array_C[i*N_p+j] = sum;
        }
    }
    return;
}

__global__ void multiplicateMatrixOnDevice(float *array_A, 
                                           float *array_B, 
                                           float *array_C,
                                           int M_p,
                                           int K_p,
                                           int N_p){
    int ix = threadIdx.x+blockIdx.x*blockDim.x; // row index
    int iy = threadIdx.y+blockIdx.y*blockDim.y; // col index
    if(ix<N_p && iy<M_p){
        float sum=0;
        for(size_t k=0; k<K_p; k++){
            sum += array_A[iy*K_p+k]*array_B[k*N_p+ix];
        }
        array_C[iy*M_p+ix] = sum;
    }
    return;
}

__global__ void matrixMultiplyShared(float *A, 
                                     float *B, 
                                     float *C, 
                                     int numARows,
                                     int numAColumns,
                                     int numBRows,
                                     int numBColumns,
                                     int numCRows,
                                     int numCColumns){

    __shared__ float sharedM[BLOCK_SIZE][BLOCK_SIZE];
    __shared__ float sharedN[BLOCK_SIZE][BLOCK_SIZE];

    int bx = blockidx.x;
    int by = blockidy.y;
    int tx = threadIdx.x;
    int ty = threadIdx.y;
    
    int row = by*BLOCK_SIZE+ty;
    int col = bx*BLOCK_SIZE+tx;

    float Csub = 0.0;

    for(size_t i=0; i<(int)(ceil((float)numAColumns)/BLOCK_SIZE); i++){
        if(i*BLOCK_SIZE + tx < numAColumns && row < numARows)
            sharedM[ty][tx] = A[row*numAColumns + i*BLOCK_SIZE + tx];
        else
            sharedM[ty][tx] = 0.0;
        if(i*BLOCK_SIZE + ty <numBRows && col < numBColumns)
            sharedN[ty][tx] = B[(i*BLOCK_SIZE + ty)*numBColumns + col];
        else
            sharedN[ty][tx] = 0.0;
        
        __syncthreads();

        for(int j=0; j<BLOCK_SIZE; j++)
            Csub += shareM[ty][j]*sharedN[j][tx];
        
        __syncthreads();
    }
    if(row < numCRows && col < numCColumns)
        C[row*numCColumns + col] = Csub;
}


int main(int argc, char **argv){
    clock_t start = 0, finish = 0;
    float time;

    int Axy = M*K;
    int Bxy = K*N;
    int Cxy = M*N;

    float *h_A, *h_B, *hostRef, *deviceRef;
    h_A = (float*)malloc(Axy*sizeof(float));
    h_B = (float*)malloc(Bxy*sizeof(float));

    int nBytes = M*N*sizeof(float);
    hostRef = (float*)malloc(Cxy*sizeof(float));
    deviceRef = (float*)malloc(Cxy*sizeof(float));

    initial(h_A, Axy);
    initial(h_B, Bxy);

    start = clock();
    multiplicateMatrixOnHost(h_A, h_B, hostRef, M, K, N);
    finish = clock();
    printf("\n");
    printf("------------------------------------------------------\n");
    printf("Computing matrix product using multiplicateMatrixOnHost \n");
    printf("------------------------------------------------------\n");
    printf("Matrix_hostRef:(%dx%d) CPU run time is :%lfs\n", M, N, time);

    float *d_A, *d_B, *d_C;

    cudaMalloc((void**)&d_A, Axy*sizeof(float));
    cudaMalloc((void**)&d_B, Bxy*sizeof(float));
    cudaMalloc((void**)&d_C, Cxy*sizeof(float));

    cudaMemcpy(d_A, h_A, Axy*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_A, h_A, Axy*sizeof(float), cudaMemcpyHostToDevice);


}
