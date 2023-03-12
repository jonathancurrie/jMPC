/*-------------------------------------------------------------------------
//Single Precision Mex Interface to a C Version of Wright Using Simple LA Routines
-------------------------------------------------------------------------*/
#include <mex.h>
#include <qp_header.h>
#include <qp_mem.h>


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //Variables are assumed in the following order
	double *H = NULL, *f = NULL, *A = NULL, *b = NULL, *z0 = NULL, *lam0 = NULL, *t0 = NULL;	    
    int maxiter, verbose;
    float tol;
    //Internal Variables
    float *fH = NULL, *ff = NULL, *fA = NULL, *fb = NULL, *fz0 = NULL, *flam0 = NULL, *ft0 = NULL;
    float *sol, *lam_sol, *t_sol, *exitflag, *mem_ptr;
    int *runiter;
    int mc, ndec, indx, i, warm=0, len;
    
    //Input Error Check
    if(nrhs < 1) {
        if(nlhs == 1) {
            plhs[0] = mxCreateString("jMPC Toolbox - Copyright Jonathan Currie 2013 (www.controlengineering.co.nz)");
            return;
        }
        #ifdef MEHROTRA               
            mexPrintf("msquad_mehrotra: High-Speed Interior-Point Predictor-Corrector QP Solver [Single Precision]\n\n");
        #elif defined NOHEUR
            mexPrintf("msquad_wrightNH: High-Speed Interior-Point QP Solver [Single Precision, No Heuristics]\n\n");
        #else
            mexPrintf("msquad_wright: High-Speed Interior-Point QP Solver [Single Precision]\n\n");
        #endif            
        mexPrintf("  Copyright (C) 2011-2013 Jonathan Currie (Control Engineering)\n  www.controlengineering.co.nz\n\n");
        #ifdef MEHROTRA
            mexPrintf("Usage: [z,exitflag,iter,lam,t] = msquad_mehrotra(H,f,A,b,maxiter,tol,verbose,z0,lam0,t0)\n");
        #elif defined NOHEUR
            mexPrintf("Usage: [z,exitflag,iter,lam,t] = msquad_wrightNH(H,f,A,b,maxiter,tol,verbose,z0,lam0)\n");
        #else
            mexPrintf("Usage: [z,exitflag,iter,lam,t] = msquad_wright(H,f,A,b,maxiter,tol,verbose,z0,lam0,t0)\n");
        #endif
        mexPrintf("\n");
        return;
    }
    if(nrhs < 4)
        mexErrMsgTxt("You must supply at least 4 arguments to this function!");
    
    //For each input check if supplied as a double or single
    if(mxGetClassID(prhs[0])==mxDOUBLE_CLASS)
        H = mxGetPr(prhs[0]);
    else if(mxGetClassID(prhs[0])==mxSINGLE_CLASS)
        fH = (float*)mxGetData(prhs[0]);
    else
        mexErrMsgTxt("H must be a double or single!");
    if(mxGetClassID(prhs[1])==mxDOUBLE_CLASS)
        f = mxGetPr(prhs[1]);
    else if(mxGetClassID(prhs[1])==mxSINGLE_CLASS)
        ff = (float*)mxGetData(prhs[1]);
    else
        mexErrMsgTxt("f must be a double or single!");
    if(mxGetClassID(prhs[2])==mxDOUBLE_CLASS)
        A = mxGetPr(prhs[2]);
    else if(mxGetClassID(prhs[2])==mxSINGLE_CLASS)
        fA = (float*)mxGetData(prhs[2]);
    else
        mexErrMsgTxt("A must be a double or single!");
    if(mxGetClassID(prhs[3])==mxDOUBLE_CLASS)
        b = mxGetPr(prhs[3]);
    else if(mxGetClassID(prhs[3])==mxSINGLE_CLASS)
        fb = (float*)mxGetData(prhs[3]);
    else
        mexErrMsgTxt("H must be a double or single!");
    //Optional Inputs
    if(nrhs > 4 && !mxIsEmpty(prhs[4])) {
        if(mxGetClassID(prhs[4])==mxDOUBLE_CLASS)
            maxiter = (int)*mxGetPr(prhs[4]);
        else if(mxGetClassID(prhs[4])==mxSINGLE_CLASS)
            maxiter = (int)*(float*)mxGetData(prhs[4]);
        else
            mexErrMsgTxt("maxiter must be a double or single!");
    }
    else
        maxiter = 200;
    if(nrhs > 5 && !mxIsEmpty(prhs[5])) {
        if(mxGetClassID(prhs[5])==mxDOUBLE_CLASS)
            tol = (float)*mxGetPr(prhs[5]);
        else if(mxGetClassID(prhs[5])==mxSINGLE_CLASS)
            tol = *(float*)mxGetData(prhs[5]);
        else
            mexErrMsgTxt("tol must be a double or single!");
    }
    else
        tol = 0.0001F;  
    if(nrhs > 6 && !mxIsEmpty(prhs[6])) {
        if(mxGetClassID(prhs[6])==mxDOUBLE_CLASS)
            verbose = (int)*mxGetPr(prhs[6]);
        else if(mxGetClassID(prhs[6])==mxSINGLE_CLASS)
            verbose = (int)*(float*)mxGetData(prhs[6]);
        else
            mexErrMsgTxt("verbose must be a double or single!");
    }
    else
        verbose = 0;
    //Optional Warm Start Vars
    if(nrhs > 9 && !mxIsEmpty(prhs[7]) && !mxIsEmpty(prhs[8]) && !mxIsEmpty(prhs[9])) { //all
        if(mxGetClassID(prhs[7])==mxDOUBLE_CLASS)
            z0 = mxGetPr(prhs[7]);
        else if(mxGetClassID(prhs[7])==mxSINGLE_CLASS)
            fz0 = (float*)mxGetData(prhs[7]);
        else
            mexErrMsgTxt("z0 must be a double or single!");
        if(mxGetClassID(prhs[8])==mxDOUBLE_CLASS)
            lam0 = mxGetPr(prhs[8]);
        else if(mxGetClassID(prhs[8])==mxSINGLE_CLASS)
            flam0 = (float*)mxGetData(prhs[8]);
        else
            mexErrMsgTxt("lam0 must be a double or single!");
        if(mxGetClassID(prhs[9])==mxDOUBLE_CLASS)
            t0 = mxGetPr(prhs[9]);
        else if(mxGetClassID(prhs[9])==mxSINGLE_CLASS)
            ft0 = (float*)mxGetData(prhs[9]);
        else
            mexErrMsgTxt("t0 must be a double or single!");
        warm=3;
    }
    else if(nrhs > 8 && !mxIsEmpty(prhs[7]) && !mxIsEmpty(prhs[8])) {
        if(mxGetClassID(prhs[7])==mxDOUBLE_CLASS)
            z0 = mxGetPr(prhs[7]);
        else if(mxGetClassID(prhs[7])==mxSINGLE_CLASS)
            fz0 = (float*)mxGetData(prhs[7]);
        else
            mexErrMsgTxt("z0 must be a double or single!");
        if(mxGetClassID(prhs[8])==mxDOUBLE_CLASS)
            lam0 = mxGetPr(prhs[8]);
        else if(mxGetClassID(prhs[8])==mxSINGLE_CLASS)
            flam0 = (float*)mxGetData(prhs[8]);
        else
            mexErrMsgTxt("lam0 must be a double or single!");
        warm=2;
    }
    else if(nrhs > 7 && !mxIsEmpty(prhs[7])) {
        if(mxGetClassID(prhs[7])==mxDOUBLE_CLASS)
            z0 = mxGetPr(prhs[7]);
        else if(mxGetClassID(prhs[7])==mxSINGLE_CLASS)
            fz0 = (float*)mxGetData(prhs[7]);
        else
            mexErrMsgTxt("z0 must be a double or single!");
        warm=1;
    }
    
    //Collect Sizes;
    ndec = (int)mxGetNumberOfElements(prhs[1]);
    mc = (int)mxGetNumberOfElements(prhs[3]);
	
    //Create Outputs
	plhs[0] = mxCreateNumericMatrix(ndec,1,mxSINGLE_CLASS,mxREAL);
    sol = (float*)mxGetData(plhs[0]);
    plhs[1] = mxCreateNumericMatrix(1,1,mxSINGLE_CLASS,mxREAL);
	exitflag = (float*)mxGetData(plhs[1]);
	plhs[2] = mxCreateNumericMatrix(1,1,mxINT32_CLASS,mxREAL);
    runiter = (int*)mxGetData(plhs[2]);       
    if(nlhs > 3) {
        plhs[3] = mxCreateNumericMatrix(mc,1,mxSINGLE_CLASS,mxREAL);
        lam_sol = (float*)mxGetData(plhs[3]);
        plhs[4] = mxCreateNumericMatrix(mc,1,mxSINGLE_CLASS,mxREAL);
        t_sol = (float*)mxGetData(plhs[4]);
    }
    //Determine memory requirement for internal single precision arrays
    len = 0;
    if(fH == NULL) len += ndec*ndec;
    if(ff == NULL) len += ndec;
    if(fA == NULL) len += mc*ndec;
    if(fb == NULL) len += mc;
    switch(warm) {
        case 3:
            if(ft0 == NULL) len += mc;
            if(flam0 == NULL) len += mc;
            if(fz0 == NULL) len += ndec;
            break;
        case 2:
            if(flam0 == NULL) len += mc;
            if(fz0 == NULL) len += ndec;
            break;
        case 1:
            if(fz0 == NULL) len += ndec;
            break;
    }
    //Allocate memory
    mem_ptr = (float*)malloc(len*sizeof(float));
    if(mem_ptr == NULL)
        mexErrMsgTxt("Error allocating single precision memory");
    //Distribute Memory and copy/cast in values
    indx = 0;
    if(fH == NULL) {
        fH   = &mem_ptr[indx]; indx += ndec*ndec;
        for(i=0;i<ndec*ndec;i++) {fH[i] = (float)H[i];}
    }
    if(ff == NULL) {
        ff   = &mem_ptr[indx]; indx += ndec;
        for(i=0;i<ndec;i++) {ff[i] = (float)f[i];}
    }
    if(fA == NULL) {
        fA   = &mem_ptr[indx]; indx += mc*ndec;
        for(i=0;i<mc*ndec;i++) {fA[i] = (float)A[i];}
    }
    if(fb == NULL) {        
        fb   = &mem_ptr[indx]; indx += mc;
        for(i=0;i<mc;i++) {fb[i] = (float)b[i];}
    }
    switch(warm) {
        case 3:
            if(ft0 == NULL) {
                ft0   = &mem_ptr[indx]; indx += mc;
                for(i=0;i<mc;i++) {ft0[i] = (float)t0[i];}
            }
            if(flam0 == NULL) {
                flam0   = &mem_ptr[indx]; indx += mc;
                for(i=0;i<mc;i++) {flam0[i] = (float)lam0[i];}
            }
            if(fz0 == NULL) {
                fz0   = &mem_ptr[indx]; indx += ndec;
                for(i=0;i<ndec;i++) {fz0[i] = (float)z0[i];}
            }
            break;
        case 2:
            if(flam0 == NULL) {
                flam0   = &mem_ptr[indx]; indx += mc;
                for(i=0;i<mc;i++) {flam0[i] = (float)lam0[i];}
            }
            if(fz0 == NULL) {
                fz0   = &mem_ptr[indx]; indx += ndec;
                for(i=0;i<ndec;i++) {fz0[i] = (float)z0[i];}
            }
            break;
        case 1:
            if(fz0 == NULL) {
                fz0   = &mem_ptr[indx]; indx += ndec;
                for(i=0;i<ndec;i++) {fz0[i] = (float)z0[i];}
            }
            break;
    }    
    //Get QP Memory
    if(!getQPMemory(mc,ndec))
        mexErrMsgTxt("QP Memory Error");
    
    //Copy in initial vals
    switch(warm)
    {
        case 3:
            memcpy(sol,fz0,ndec*sizeof(float));
            memcpy(lambda,flam0,mc*sizeof(float));
            memcpy(tslack,ft0,mc*sizeof(float));
            break;
        case 2:
            memcpy(sol,fz0,ndec*sizeof(float));
            memcpy(lambda,flam0,mc*sizeof(float));
            break;
        case 1:
            memcpy(sol,fz0,ndec*sizeof(float));
            break;
    }

    //Call Routine
    #ifdef MEHROTRA
        exitflag[0] = (float)quad_mehrotra(fH,ff,fA,fb,mc,ndec,sol,lambda,tslack,warm,tol,maxiter,verbose,runiter);
    #elif defined NOHEUR
        exitflag[0] = (float)squad_wrightNH(fH,ff,fA,fb,mc,ndec,sol,lambda,tslack,warm,tol,maxiter,verbose,runiter);        
    #else
        exitflag[0] = (float)quad_wright(fH,ff,fA,fb,mc,ndec,sol,lambda,tslack,warm,tol,maxiter,verbose,runiter);        
    #endif    
    //Copy out dual solution
    if(nlhs > 3) {
        memcpy(lam_sol,lambda,mc*sizeof(float));
        memcpy(t_sol,tslack,mc*sizeof(float));
    }    
    //Free QP Memory
    freeQPMemory();
    //Free Local Memory
    free(mem_ptr); mem_ptr = NULL;
}