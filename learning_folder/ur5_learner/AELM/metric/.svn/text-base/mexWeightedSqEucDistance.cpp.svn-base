#include <math.h>
#include <matrix.h>
#include <mex.h>

//input: vector a, vector b, vector w
//output: vector sum(w .* (a-b).^2)

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    //dx = l.alpha * sum(repmat(l.h', 1, l.visDim) .* (l.center.C - repmat(x, l.hidDim, 1)), 1);
    
    //get dimensions
    int dim = mxGetDimensions(prhs[0])[1]; //row vector
    int numSamples = mxGetDimensions(prhs[0])[0];
    int numSamplesB = mxGetDimensions(prhs[1])[0];
       
    //get inputs
    double *a = mxGetPr(prhs[0]);
    double *b = mxGetPr(prhs[1]);
    double *w = mxGetPr(prhs[2]);
    
    //create output vector
    plhs[0] = mxCreateDoubleMatrix(numSamples,1,mxREAL); //column vector
    double *d = mxGetPr(plhs[0]);
    
    if (numSamples == numSamplesB) {
        for (int i=numSamples-1; i>=0; i--) {
            for (int j=dim-1; j>=0; j--) {
                d[i] += w[j] * pow(a[j*numSamples+i] - b[j*numSamples+i], 2);
            }
        }
    } else {
        for (int i=numSamples-1; i>=0; i--) {
            for (int j=dim-1; j>=0; j--) {
                d[i] += w[j] * pow(a[j*numSamples+i] - b[j], 2);
            }
        }
    }
}
