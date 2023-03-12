/* QP EMBED TEST - MEX Wrapper to Test QP EMBED in MATLAB
 * Copyright (C) Jonathan Currie 2012 (I2C2) 
 */

//Note extra variables are automatically inserted here by embed()

//EMBED START
//External Constants
extern const unsigned int ndec;
extern const unsigned int mc;
extern const unsigned int soft;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //Optional inputs
    int verbosity;    
    //Outputs
    double *norm;
    int *status;
    unsigned int *runiter;
    //Internal vars
    unsigned int i;
    
    //Get Input
    if(nrhs)
        verbosity = (int)*mxGetPr(prhs[0]);
        
    //Create Outputs
	plhs[0] = mxCreateNumericMatrix(1,1,MPC_CLASS,mxREAL);
    norm = mxGetData(plhs[0]);
    plhs[1] = mxCreateNumericMatrix(1,1,mxINT32_CLASS,mxREAL);
    status = (int*)mxGetData(plhs[1]);
    plhs[2] = mxCreateNumericMatrix(1,1,mxUINT32_CLASS,mxREAL);
    runiter = (unsigned int*)mxGetData(plhs[2]);
        
    //Run QP solver with constants
    *status = qpwright_embed(f, b, z, lambda, tslack, 0, runiter);  
    
    //Calculate norm
    for(i = 0; i < (ndec-soft); i++) 
        *norm += (rdel_u[i] - z[i])*(rdel_u[i] - z[i]);
    
    *norm = sqrt(*norm);
    
    //Optionally Print Summary
    if(verbosity) {
        //Print Summary        
        mexPrintf("\nEmbedded MPC QP Testbench Comparison:\n");        
        switch(*status) 
        {
            case  1: mexPrintf(" Successfully Solved [Status %d vs Ref %d]\n",*status,rstatus); break;
            case -1: mexPrintf(" Maximum Iterations Exceeded [Status %d vs Ref %d]\n",*status,rstatus); break;
            case -2: mexPrintf(" Cholesky Failed - Further Progress Is Unlikely [Status %d vs Ref %d]\n",*status,rstatus); break;
            case -3: mexPrintf(" Failed - Numerical Errors Detected [Status %d vs Ref %d]\n",*status,rstatus); break;
            case -4: mexPrintf(" Failed - Problem Looks Infeasible [Status %d vs Ref %d]\n",*status,rstatus); break;
        }
        mexPrintf("Reference   Testbench\n");
        mexPrintf("%d iter       %d iter\n",riter,*runiter);
        for(i = 0; i < (ndec-soft); i++) {
            mexPrintf("%+9.6f    %+9.6f\n",rdel_u[i],z[i]);
        }
        mexPrintf("\n2norm: %1.5g\n\n",*norm);
    }
}