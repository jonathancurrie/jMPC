/* MPC EMBED TEST - MEX Wrapper to Test MPC EMBED in MATLAB
 * Copyright (C) Jonathan Currie 2012 (Control Engineering) 
 */

//Note extra variables are automatically inserted here by embed()

//EMBED START
//External Constants
extern const uintT n_in;
extern const uintT nm_in;
extern const uintT n_out;
extern const uintT nm_out;
extern const uintT nm_dist;
extern const uintT nq_out;
extern const uintT states;

//Main Test Function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //Internal Args
    unsigned int i, k = 0, veclen = T+1;
    
    //Outputs
    const char* fieldnames[8] = {"u","del_u","xm","ym","xp","yp","qpstat","qpiter"};
    double *pu, *pdel_u, *pym, *pxm, *pxp, *pyp;
    double *u, *del_u, *ym, *xm, *setpk, *mdistk;
    int *pqpstat;
    unsigned int *pqpiter;
    
    //Create Outputs
    plhs[0] = mxCreateStructMatrix(1,1,8,fieldnames); 
    mxSetField(plhs[0],0,"u",mxCreateNumericMatrix(veclen,n_in,MPC_CLASS,0));
    mxSetField(plhs[0],0,"del_u",mxCreateNumericMatrix(veclen,n_in,MPC_CLASS,0));
    mxSetField(plhs[0],0,"xm",mxCreateNumericMatrix(veclen,states,MPC_CLASS,0));
    mxSetField(plhs[0],0,"ym",mxCreateNumericMatrix(veclen,n_out,MPC_CLASS,0));
    mxSetField(plhs[0],0,"xp",mxCreateNumericMatrix(veclen,states,MPC_CLASS,0));
    mxSetField(plhs[0],0,"yp",mxCreateNumericMatrix(veclen,nm_out,MPC_CLASS,0));
    mxSetField(plhs[0],0,"qpstat",mxCreateNumericMatrix(veclen,1,mxINT32_CLASS,0));
    mxSetField(plhs[0],0,"qpiter",mxCreateNumericMatrix(veclen,1,mxUINT32_CLASS,0));
    
    //Get Structure Pointers
    pu = mxGetData(mxGetField(plhs[0],0,"u"));
    pdel_u = mxGetData(mxGetField(plhs[0],0,"del_u"));
    pxm = mxGetData(mxGetField(plhs[0],0,"xm"));
    pym = mxGetData(mxGetField(plhs[0],0,"ym"));
    pxp = mxGetData(mxGetField(plhs[0],0,"xp"));
    pyp = mxGetData(mxGetField(plhs[0],0,"yp"));
    pqpstat = (int*)mxGetData(mxGetField(plhs[0],0,"qpstat"));
    pqpiter = (unsigned int*)mxGetData(mxGetField(plhs[0],0,"qpiter"));
    
    //Create Internal Args
    u = mxGetData(mxCreateNumericMatrix(1,n_in,MPC_CLASS,0));
    del_u = mxGetData(mxCreateNumericMatrix(1,n_in,MPC_CLASS,0));
    xm = mxGetData(mxCreateNumericMatrix(1,states,MPC_CLASS,0));
    ym = mxGetData(mxCreateNumericMatrix(1,n_out,MPC_CLASS,0));
    setpk = mxGetData(mxCreateNumericMatrix(1,nq_out,MPC_CLASS,0));
    mdistk = mxGetData(mxCreateNumericMatrix(1,max(nm_dist,1),MPC_CLASS,0));
    
    //Copy Initial u
    memcpy(u,init_u,n_in*sizeof(double)); 
    for(i=0;i<n_in;i++)
        pu[veclen*i] = init_u[i];
    //Copy Initial xp,yp
    for(i=0;i<states;i++)
        pxp[veclen*i] = xp[i];
    for(i=0;i<nm_out;i++)
        pyp[veclen*i] = yp[i];
    
    //Run MPC Simulation    
    for(k = 0; k < T; k++)
    {
        //Allocate Setpoint & Measured Disturbances
        for(i = 0; i < nq_out; i++)
            setpk[i] = setp[k + veclen*i];
        for(i = 0; i < max(nm_dist,1); i++)
            mdistk[i] = mdist[k + veclen*i];
        
        //----- RUN MPC ENGINE -----//
        pqpstat[k+1] = EmbedMPCSolve(yp, setpk, mdistk, u, del_u, ym, xm, &pqpiter[k+1]);  
        
        //----- Process & Save Model Inputs -----//
        for(i = 0; i < n_in; i++) {
			pu[k + 1 + veclen*i] = u[i]; 	   //Saved Input		
        	upp[i] = u[i] + umdist[k + veclen * i]; //Plant Input
        }
        for(i = 0; i < nm_in; i++)
			pdel_u[k + 1 + veclen*i] = del_u[i];        
        
        //----- Process & Save Model States & Outputs -----//
        for(i = 0; i < states; i++) 
            pxm[k + 1 + veclen * i] = xm[i];
        for(i = 0; i < n_out; i++) 
            pym[k + 1 + veclen * i] = ym[i]; 

        //----- Simulate Plant -----//
        jmv(states,states,1.0,plantA,xp,0.0,Kr);
        jmv(states,n_in,1.0,plantB,upp,1.0,Kr);
        memcpy(xp,Kr,states*sizeof(double));
        jmv(nm_out,states,1.0,plantC,xp,0.0,yp);
        //Process & Save Plant States & Outputs
        for(i = 0; i < nm_out; i++)
        {
            yp[i] += ydist[k + veclen * i];
            pyp[k + 1 + veclen * i] = yp[i];
        }
        for(i = 0; i < states; i++)
            pxp[k + 1 + veclen * i] = xp[i];
    }
}