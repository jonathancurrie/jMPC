/*-------------------------------------------------------------------------
//Mex Interface to a C Version of Wright/Mehrotra [Double Precision]
-------------------------------------------------------------------------*/
#include <mex.h>
#include <qp_header.h>
#include <qp_mem.h>


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //Variables are assumed in the following order
	double *H, *f, *A, *b, *z0, *lam0, *t0;	
    int maxiter, verbose;
    double tol;
    //Internal Variables
    double *sol, *lam_sol, *t_sol;
    double *exitflag;
    int *runiter;
    int mc, ndec, warm=0;
    
    //Input Error Check
    if(nrhs < 1) {
        if(nlhs == 1) {
            plhs[0] = mxCreateString("jMPC Toolbox - Copyright Jonathan Currie 2013 (www.i2c2.aut.ac.nz)");
            return;
        }
        #ifdef MEHROTRA               
            #ifdef MKL
                mexPrintf("mquad_mehrotraMKL: High-Speed Interior-Point Predictor-Corrector QP Solver [Double Precision]\n\n  Utilizies Intel MKL %d.%d R%d\n\n",__INTEL_MKL__,__INTEL_MKL_MINOR__,__INTEL_MKL_UPDATE__);
            #else
                mexPrintf("mquad_mehrotra: High-Speed Interior-Point Predictor-Corrector QP Solver [Double Precision]\n\n");
            #endif
            mexPrintf(" Ref: S. Mehrotra, \"On the Implementation of a Primal-Dual Interior Point Method\"\n      SIAM Journal on Optimization 2(4), 1992, pp. 575-601\n\n");
        #else
            #ifdef MKL
                mexPrintf("mquad_wrightMKL: High-Speed Interior-Point QP Solver [Double Precision]\n\n  Utilizies Intel MKL %d.%d R%d\n\n",__INTEL_MKL__,__INTEL_MKL_MINOR__,__INTEL_MKL_UPDATE__);
            #else
                mexPrintf("mquad_wright: High-Speed Interior-Point QP Solver [Double Precision]\n\n");
            #endif
            mexPrintf(" Ref: S. J. Wright, \"Applying New Optimization Algorithms to Model Predictive Control\"\n      in Chemical Process Control-V, CACHE, AIChE Symposium, 1997, pp. 147-155\n\n");
        #endif            
        mexPrintf(" Copyright (C) 2011-2013 Jonathan Currie (I2C2)\n www.i2c2.aut.ac.nz\n\n");        
        #ifdef MEHROTRA
            #ifdef MKL
                mexPrintf("Usage: [z,exitflag,iter,lam,t] = mquad_mehrotraMKL(H,f,A,b,maxiter,tol,verbose,z0,lam0,t0)\n");
            #else
                mexPrintf("Usage: [z,exitflag,iter,lam,t] = mquad_mehrotra(H,f,A,b,maxiter,tol,verbose,z0,lam0,t0)\n");
            #endif
        #else 
            #ifdef MKL
                mexPrintf("Usage: [z,exitflag,iter,lam,t] = mquad_wrightMKL(H,f,A,b,maxiter,tol,verbose,z0,lam0,t0)\n");
            #else
                mexPrintf("Usage: [z,exitflag,iter,lam,t] = mquad_wright(H,f,A,b,maxiter,tol,verbose,z0,lam0,t0)\n");
            #endif        
        #endif    
        mexPrintf("\n");
        return;
    }
    if(nrhs < 4)
        mexErrMsgTxt("You must supply at least 4 arguments to this function!");
    
    //Get pointers to Input variables
	H = mxGetPr(prhs[0]);
	f = mxGetPr(prhs[1]);
    A = mxGetPr(prhs[2]);
    b = mxGetPr(prhs[3]);
    if(nrhs > 4 && !mxIsEmpty(prhs[4]))
        maxiter = (int)*mxGetPr(prhs[4]);
    else
        maxiter = 200;
    if(nrhs > 5 && !mxIsEmpty(prhs[5]))
        tol = *mxGetPr(prhs[5]);
    else
        tol = 1e-6;  
    if(nrhs > 6 && !mxIsEmpty(prhs[6]))
        verbose = (int)*mxGetPr(prhs[6]);
    else
        verbose = 0;   
    //Optional Warm Start Args
    if(nrhs > 9 && !mxIsEmpty(prhs[7]) && !mxIsEmpty(prhs[8]) && !mxIsEmpty(prhs[9])) { //all
        z0 = mxGetPr(prhs[7]);
        lam0 = mxGetPr(prhs[8]);
        t0 = mxGetPr(prhs[9]);
        warm=3;
    }
    else if(nrhs > 8 && !mxIsEmpty(prhs[7]) && !mxIsEmpty(prhs[8])) {
        z0 = mxGetPr(prhs[7]);
        lam0 = mxGetPr(prhs[8]);
        warm=2;
    }
    else if(nrhs > 7 && !mxIsEmpty(prhs[7])) {
        z0 = mxGetPr(prhs[7]);
        warm=1;
    }
    
    //Collect Sizes;
    ndec = (int)mxGetNumberOfElements(prhs[1]);
    mc = (int)mxGetNumberOfElements(prhs[3]);
	
    //Create Outputs
	plhs[0] = mxCreateDoubleMatrix(ndec,1,mxREAL);
    sol = mxGetPr(plhs[0]);
    plhs[1] = mxCreateDoubleMatrix(1,1,mxREAL);
	exitflag = mxGetPr(plhs[1]);
	plhs[2] = mxCreateNumericMatrix(1,1,mxINT32_CLASS,mxREAL);
    runiter = mxGetData(plhs[2]);
    if(nlhs > 3) {
        plhs[3] = mxCreateDoubleMatrix(mc,1,mxREAL);
        lam_sol = mxGetPr(plhs[3]);
        plhs[4] = mxCreateDoubleMatrix(mc,1,mxREAL);
        t_sol = mxGetPr(plhs[4]);
    }
    
    //Get QP Memory
    if(!getQPMemory(mc,ndec))
        mexErrMsgTxt("QP Memory Error");
    
    //Copy in initial vals
    switch(warm)
    {
        case 3:
            memcpy(sol,z0,ndec*sizeof(double));
            memcpy(lambda,lam0,mc*sizeof(double));
            memcpy(tslack,t0,mc*sizeof(double));
            break;
        case 2:
            memcpy(sol,z0,ndec*sizeof(double));
            memcpy(lambda,lam0,mc*sizeof(double));
            break;
        case 1:
            memcpy(sol,z0,ndec*sizeof(double));
            break;
    }
    //Call Routine
    #ifdef MEHROTRA
        exitflag[0] = quad_mehrotra(H,f,A,b,mc,ndec,sol,lambda,tslack,warm,tol,maxiter,verbose,runiter);
    #else 
        exitflag[0] = quad_wright(H,f,A,b,mc,ndec,sol,lambda,tslack,warm,tol,maxiter,verbose,runiter);      
    #endif
            
    //Copy out dual solution
    if(nlhs > 3) {
        memcpy(lam_sol,lambda,mc*sizeof(double));
        memcpy(t_sol,tslack,mc*sizeof(double));
    }
    
    //Free QP Memory
    freeQPMemory();
}