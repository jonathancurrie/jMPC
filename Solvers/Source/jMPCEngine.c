/* jMPC Controller Engine Implemented as a C MEX Function 
 *
 * Copyright (C) 2009-2013 Jonathan Currie (www.i2c2.aut.ac.nz)
 */

#include <mex.h>
#include <windows.h>
#include "jmath.h"
#include "qp_header.h"
#include "qp_mem.h"
#include "mpc_utils.h"
#ifdef SINGLE_PREC
    #define MPC_CLASS mxSINGLE_CLASS
#else
    #define MPC_CLASS mxDOUBLE_CLASS
#endif

// #define DEBUG

//Input Argument Order
enum {eJMPC,eJSIM};         
#define pJMPC    prhs[eJMPC]
#define pJSIM    prhs[eJSIM]

//***** Global Variables *****//
//See getMPCArgs() for a full list of variable descriptions
realT *mpc_ptr; //Dynamic memory pointer
//Model
realT *modelA, *modelB;
//Horizons
unsigned int Np, Nb;
//Sizes
unsigned int n_in, n_out, nm_in, nm_dist, nm_out, nq_out, mc, ndec;                    
unsigned int states, statesn, statesnm, statesnq, Np_out, Np_qout, Nb_in;
//Prediction
realT *F, *Fq, *Phiv, *Phiqv;
//State Estimation
realT *IKC, *Kest;
//QP
realT *PhiTQ, *SE, *H, *R, *Hscale, *Ascale;
//Constraints
realT *A, *Tin, *bcon, *ucon;
//Loop Variables & Initial Values
realT *init_u, *u, *init_up, *del_u;
realT *init_xm, *xm, *init_del_xm, *del_xm, *init_xp, *xp;
realT *init_ym, *ym, *init_yp, *yp;
//Index Vectors
unsigned int *idelu, *iumin, *iumax, *iymin, *iymax;
unsigned int ndelu, numin, numax, nymin, nymax, numeas_out;
unsigned int *iman_u, *imeas_dist, *isq_out, *iumeasp_out, *ismeasp_out;                 
//Linearization Biases
realT *u_op, *y_op;
//Mode & Options
unsigned int look_ahead, soft, uncon, warm, warn, qpsolver, qpverbose, qpmaxiter;      
realT qptol;
//Simulation
realT *plantA, *plantB, *plantC, *udist, *ydist, *mdist, *setp;
unsigned int Tfinal, veclen, setlen;
//Plot Vectors
realT *plot_del_u, *plot_u, *plot_xm, *plot_ym, *plot_xp, *plot_yp;
double *tvec;
int *itervec, *statusvec;
//Engine Variables
realT *K, *Kys, *ys, *y0, *qp_del_xm, *Ssetp, *sp, *x;
realT *fvec, *del, *bvec, *bcheck, *up, *umeas_out, *um, *v, *del_v;    

//***** Function Prototypes *****//
void MPCEngine();
void getMPCArgs(const mxArray *prhs[]);
int getMPCMemory();
void freeMPCMemory();
//Ctrl-C Detection
extern bool utIsInterruptPending();
extern void utSetInterruptPending(bool);

//***** Main MEX Interface *****//
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    const char* fieldnames[9] = {"u","del_u","xm","ym","xp","yp","qpstat","qpiter","time"};
    
    if(nrhs == 0) {
        #ifdef SINGLE_PREC
            mexPrintf("jMPCEngine: Accelerated Linear MPC Simulation Engine [Single Precision]\n\n");
        #else
            mexPrintf("jMPCEngine: Accelerated Linear MPC Simulation Engine [Double Precision]\n\n");
        #endif
        mexPrintf("  Copyright (C) 2011-2013 Jonathan Currie (I2C2)\n  www.i2c2.aut.ac.nz\n\n");
        #ifdef SINGLE_PREC
            mexPrintf("Usage: results = jMPCSEngine(jMPCobj,jSIMobj)\n\n");
        #else
            mexPrintf("Usage: results = jMPCEngine(jMPCobj,jSIMobj)\n\n");
        #endif    
        return;
    }
    
    //Check we have enough input args
    if(nrhs < 2)
        mexErrMsgTxt("This function expects two arguments! jMPCEngine(jMPC,jSIM)");
    
    //Get Passed Arugments
    getMPCArgs(prhs);

    //Create Function Output Structure
    plhs[0] = mxCreateStructMatrix(1,1,9,fieldnames); 
    mxSetField(plhs[0],0,"u",mxCreateNumericMatrix(veclen,n_in,MPC_CLASS,mxREAL));
    mxSetField(plhs[0],0,"del_u",mxCreateNumericMatrix(veclen,n_in,MPC_CLASS,mxREAL));
    mxSetField(plhs[0],0,"xm",mxCreateNumericMatrix(veclen,states,MPC_CLASS,mxREAL));
    mxSetField(plhs[0],0,"ym",mxCreateNumericMatrix(veclen,n_out,MPC_CLASS,mxREAL));
    mxSetField(plhs[0],0,"xp",mxCreateNumericMatrix(veclen,states,MPC_CLASS,mxREAL));
    mxSetField(plhs[0],0,"yp",mxCreateNumericMatrix(veclen,nm_out,MPC_CLASS,mxREAL));
    mxSetField(plhs[0],0,"time",mxCreateDoubleMatrix(veclen,1,mxREAL));
    mxSetField(plhs[0],0,"qpstat",mxCreateNumericMatrix(veclen,1,mxINT32_CLASS,mxREAL));
    mxSetField(plhs[0],0,"qpiter",mxCreateNumericMatrix(veclen,1,mxINT32_CLASS,mxREAL));
    
    //Get Structure Pointers
    plot_u = (realT*)mxGetData(mxGetField(plhs[0],0,"u"));
    plot_del_u = (realT*)mxGetData(mxGetField(plhs[0],0,"del_u"));
    plot_xm = (realT*)mxGetData(mxGetField(plhs[0],0,"xm"));
    plot_ym = (realT*)mxGetData(mxGetField(plhs[0],0,"ym"));
    plot_xp = (realT*)mxGetData(mxGetField(plhs[0],0,"xp"));
    plot_yp = (realT*)mxGetData(mxGetField(plhs[0],0,"yp"));
    tvec = mxGetPr(mxGetField(plhs[0],0,"time")); //must be a double
    statusvec = (int*)mxGetData(mxGetField(plhs[0],0,"qpstat"));
    itervec = (int*)mxGetData(mxGetField(plhs[0],0,"qpiter"));

    //Get Dynamic Memory
    if(!getQPMemory(mc,ndec))
        mexErrMsgTxt("QP Memory Allocation Error");
    if(!getMPCMemory())
        mexErrMsgTxt("MPC Memory Allocation Error");

    //Call Main jMPC Engine
    MPCEngine();
    
    //Free Dynamic Memory
    freeQPMemory();
    freeMPCMemory();    
}

//-- Main MPC Simulation Function --//
void MPCEngine()
{ 
    //Local Variables
    unsigned int i, j, k, index, global_fail = 0, offset = 0, runiter = 0, local_warm = 0;
    int qpstatus;
	realT y0mul = 0.0;
    
    //Timer Variables
    LARGE_INTEGER start, stop, freq;

    //Set Initial Plot Values
    for(i = 0; i < n_in; i++)
        plot_u[i * veclen] = init_up[i] - u_op[i];    
    for(i = 0; i < states; i++) {   
        plot_xm[i * veclen] = init_xm[i];
        plot_xp[i * veclen] = init_xp[i];
    }
    for(i = 0; i < n_out; i++)
        plot_ym[i * veclen] = init_ym[i];
    for(i = 0; i < nm_out; i++)
        plot_yp[i * veclen] = init_yp[i];
    
    //Set Initial Loop Values
    memcpy(u,init_u,nm_in*sizeof(realT));
    memcpy(xm,init_xm,states*sizeof(realT));
    memcpy(del_xm,init_del_xm,statesn*sizeof(realT));
    memcpy(xp,init_xp,states*sizeof(realT));
    memcpy(ym,init_ym,n_out*sizeof(realT));
    memcpy(yp,init_yp,nm_out*sizeof(realT));   
    
    //Initialize Timer
    if(!QueryPerformanceFrequency(&freq))
        mexPrintf("\nTimer Could not retrieve System Frequency!\n"); 
    
    //----- MAIN SIMULATION LOOP -----//
    for(k = 0; k < Tfinal; k++)
    {
        //Check for user exit
        if (utIsInterruptPending()) {
            utSetInterruptPending(false); // clear Ctrl-C status
            mexPrintf("\nCtrl-C Detected. Exiting jMPCEngine...\n\n");
            return;
        }
        //Re-initialize status
        qpstatus = 0;
        //Start Timer
        QueryPerformanceFrequency(&freq);
    	QueryPerformanceCounter(&start);
        
        //----- State Estimator Update -----//
		//K(ismeasp_out) = J.state_est.Kest*(yp-y_op);
		for(i = 0; i < nm_out; i++)
            ys[i] = yp[i]-y_op[i]; 
        jmv(statesnm,nm_out,1.0,Kest,ys,0.0,Kys);		
		for(i = 0; i < (states+nm_out);i++)
			K[ismeasp_out[i]] = Kys[i];
		//K(iumeasp_out) = del_xm(iumeasp_out); 
		for(i = 0; i < (n_out-nm_out); i++)
			K[iumeasp_out[i]] = del_xm[iumeasp_out[i]];		
		//del_xm = J.state_est.IKC*del_xm + K    
        jmv(statesn,statesn,1.0,IKC,del_xm,1.0,K);
        memcpy(del_xm,K,statesn*sizeof(realT));
	
        //----- Update RHS -----//				
		if(nm_dist > 0) {
            //Get current measured disturbance
            for(i = 0; i < nm_dist; i++)
                del_v[i] = mdist[k + veclen * i];
            //Disturbance Prediction = Phiqv * mdist;
            jmv(Np_qout,nm_dist,1.0,Phiqv,del_v,0.0,y0);
			y0mul = 1.0; 
        }
		else
			y0mul = 0.0;
			
        //Use only controlled outputs for QP prediction = del_xm(isq_out)
        for(i = 0; i < statesnq; i++)
            qp_del_xm[i] = del_xm[isq_out[i]]; 
        //y0 = J.Fq*qpdel_xm + Phiqv*mdist'; 
        jmv(Np_qout,statesnq,1.0,Fq,qp_del_xm,y0mul,y0);
        if(!look_ahead) {
            //setp = J.QP.S*setp(k,:)'-y0;
            for(i = 0; i < nq_out; i++) 
                sp[i] = setp[k + setlen*i];          
            memcpy(Ssetp,y0,Np_qout*sizeof(realT));
            jmv(Np_qout,nq_out,1.0,SE,sp,-1.0,Ssetp);
        }
        else { //setpoint look ahead        
            //setp = setp(k:k+J.Np-1,:)'; setp = setp(:);
            for(i = 0, index = 0; i < Np; i++, index += nq_out) {
                for(j = 0; j < nq_out; j++)
                    Ssetp[index+j] = setp[k + i + setlen*j] - y0[index+j];
            }
        }            
        //f = J.QP.PhiTQ*Ssetp;
        jmv(Nb_in,Np_qout,-1.0,PhiTQ,Ssetp,0.0,fvec);
        //Scale f
        for(i=0;i<ndec;i++) 
            fvec[i] *= Hscale[i];           

        //Constrained Problem
        if(!uncon) {  
            //Initialize
            for(i=0;i<mc;i++)
                bvec[i] = 0.0;
            offset = 0;
            if(ndelu) //delu constraints exist
                offset = ndelu * 2; //offset past delu constraints
            if(numin > 0 || numax > 0) { //u constraints exist
                //del = J.constraints.Tin*u;
                jmv(Nb_in,nm_in,1.0,Tin,u,0.0,del);        
                //Pick off finite constraints
                if(numin) { //umin constraints exist                
                    for(i = 0; i < numin; i++)
                        bvec[i+offset] = del[iumin[i]];
                    offset += numin;
                }
                if(numax) { //umax constraints exist                
                    for(i = 0; i < numax; i++)
                        bvec[i+offset] = -del[iumax[i]];
                    offset += numax;
                }                
            }
            if(nymin > 0 || nymax > 0) { //y constraints exist
                if(nm_dist > 0) {
                    //Disturbance Prediction = Phiv * mdist;
                    jmv(Np_out,nm_dist,1.0,Phiv,del_v,0.0,y0);
                    y0mul = 1.0; 
                }
                else
                    y0mul = 0.0;

                //Full prediction for constraints -> y0 = J.F*del_xm + Phiv * mdist;
                jmv(Np_out,statesn,1.0,F,del_xm,y0mul,y0);       
                if(nymin) { //ymin constraints exist
                    for(i = 0; i < nymin; i++)
                        bvec[i+offset] = y0[iymin[i]];
                    offset += nymin;
                }
                if(nymax) { //ymax constraints exist
                    for(i = 0; i < nymax; i++)
                        bvec[i+offset] = -y0[iymax[i]];
                    offset += nymax;
                }
            }            			
            //Add constant & dynamic b while scaling
            for(i = 0; i < mc; i++)
                bvec[i] = (bvec[i] + bcon[i])*Ascale[i];

            //----- Check Unconstrained Optimum -----//
            //x = J.QP.R\(J.QP.R'\f)
            memcpy(x,fvec,ndec*sizeof(realT));
            jtris(R,x,ndec);                    
            jmv(mc,ndec,-1.0,A,x,0.0,bcheck); //Check Ax > b
            global_fail = 0;
            for(i = 0; i < mc; i++) {
                if(bcheck[i] > bvec[i]) {
                    global_fail = 1; 
                    break;
                }
            }        
            //----- Solve Constrained Optimum -----//              
            if(global_fail) {
                if(qpverbose)
                    mexPrintf("Sample %d:\n",k+1);
                //Solve QP
                if(qpsolver==2)
                    qpstatus = quad_mehrotra(H, fvec, A, bvec, mc, ndec, del_u, lambda, tslack, local_warm, qptol, qpmaxiter, qpverbose, &runiter);
                else
                    qpstatus = quad_wright(H, fvec, A, bvec, mc, ndec, del_u, lambda, tslack, local_warm, qptol, qpmaxiter, qpverbose, &runiter);                           
                if(warn && qpverbose==0) {
                    switch(qpstatus)
                    {
                        case -1: mexPrintf("Sample %d: Maximum Iterations Exceeded\n",k+1); break;
                        case -2: mexPrintf("Sample %d: Cholesky Failure\n",k+1); break;
                        case -3: mexPrintf("Sample %d: Numerical Errors Encountered\n",k+1); break;
                        case -4: mexPrintf("Sample %d: No Feasible Solution Found\n",k+1); break;
                        case -5: mexPrintf("Sample %d: Slow Progress - Early Exit\n",k+1); break;
                        case -6: mexPrintf("Sample %d: Unknown QP Exit\n",k+1); break;
                    }
                }
                if(warm) {
                    //Add a little to lambda, t to allow solver to find new constraints easier
                    for(i=0;i<mc;i++) {
                        if(lambda[i] < (realT)0.1)
                            lambda[i] += (realT)0.15;    
                        if(tslack[i] < (realT)0.1)
                            tslack[i] += (realT)0.15; 
                    }
                    //Re-enable full warm starting
                    local_warm = 3;
                }
            }
            else {
                for(i = 0; i < Nb_in; i++)
                    del_u[i] = -x[i]; //Global solution is -x
                //Global solution is used, so disable dual+slack warm starting for now
                if(warm)
                    local_warm = 1;   
                //No QP Iters
                runiter = 0;
            }       
            
            //----- Saturate Inputs -----//
            if(ndelu) {
                for(i = 0, j = 0; i < Nb_in; i++) {        
                    if(del_u[i] > ucon[j+2*nm_in])   //saturate delu at constraints
                        del_u[i] = ucon[j+2*nm_in];
                    else if(del_u[i] < -ucon[j+2*nm_in])
                        del_u[i] = -ucon[j+2*nm_in];                   
                    if(++j >= nm_in)
                        j = 0;
                }
            }            
            for(i = 0; i < nm_in; i++) {
                u[i] += del_u[i];           //sum del_u as we go to get u
                if(numin || numax) {
                    if(u[i] < ucon[i])          //saturate u at constraints
                        u[i] = ucon[i];
                    else if(u[i] > ucon[i+nm_in])
                        u[i] = ucon[i+nm_in];
                }
            }
        }
        //Unconstrained Problem
        else {
            //x = J.QP.R\(J.QP.R'\f)
            memcpy(x,fvec,ndec*sizeof(realT));
            jtris(R,x,ndec); 
            for(i = 0; i < Nb_in; i++)
                del_u[i] = -x[i]; //Global solution is -x
            //Sum del_u to get u
            for(i = 0; i < nm_in; i++) 
                u[i] += del_u[i];           
            runiter = 0;
        }        
        //Record # of Iterations & Status
        itervec[k+1] = runiter;
        statusvec[k+1] = qpstatus;

        //Add Measured Disturbance
		if(nm_dist > 0) {
			for(i = 0; i < nm_in; i++) {
				um[iman_u[i]] = del_u[i]; 		//place del_u in model input
				up[iman_u[i]] = u[i];           //place u in plant input
			}
			for(i = 0; i < nm_dist; i++) {
				um[imeas_dist[i]] = del_v[i]; 	//place del_v in model input		
				v[i] += del_v[i];				//sum del_v
				up[imeas_dist[i]] = v[i]; 		//place v in plant input
			}		
		}
		else {
			for(i = 0; i < n_in; i++) {
				um[i] = del_u[i];
				up[i] = u[i];
			}
		}
		//Save Inputs
        for(i = 0; i < n_in; i++) {
			plot_u[k + 1 + veclen*i] = up[i]; 				 //Saved Input		
        	up[i] = up[i] + udist[k + veclen * i] + u_op[i]; //Plant Input
        }
        for(i = 0; i < nm_in; i++)
			plot_del_u[k + 1 + veclen*i] = del_u[i];

        //----- Simulate Model -----//
        jmv(statesn,statesn,1.0,modelA,del_xm,0.0,K);
        jmv(statesn,n_in,1.0,modelB,um,1.0,K);
        memcpy(del_xm,K,statesn*sizeof(realT));

        //----- Process & Save Model States & Outputs -----//
        for(i = 0; i < states; i++) {
            xm[i] += del_xm[i];
            plot_xm[k + 1 + veclen * i] = xm[i];
        }
        for(i = states, index = 0; i < statesn; i++, index++) 
            plot_ym[k + 1 + veclen * index] = del_xm[i]; 

        //----- Simulate Plant -----//
        jmv(states,states,1.0,plantA,xp,0.0,K);
        jmv(states,n_in,1.0,plantB,up,1.0,K);
        memcpy(xp,K,states*sizeof(realT));
        jmv(nm_out,states,1.0,plantC,xp,0.0,yp);
        
        //Process & Save Plant States & Outputs
        for(i = 0; i < nm_out; i++) {
            yp[i] += ydist[k + veclen * i];
            plot_yp[k + 1 + veclen * i] = yp[i];
        }
        for(i = 0; i < states; i++)
            plot_xp[k + 1 + veclen * i] = xp[i];        
        
        //Record Iteration Elapsed Time
		QueryPerformanceCounter(&stop);
		tvec[k+1] =(double)(stop.QuadPart-start.QuadPart)*1e3/(double)freq.QuadPart;
    }   
}

//-- Save Pointers to the Passed Arguments, extracting from Structures if required --//
void getMPCArgs(const mxArray *prhs[])
{
    //Get The Main Object Properties;
    mxArray *model = getClassProp(pJMPC,"jMPC","Model");
    mxArray *pred = getClassProp(pJMPC,"jMPC","pred");
    mxArray *sizes = getClassProp(pJMPC,"jMPC","sizes");
    mxArray *state_est = getClassProp(pJMPC,"jMPC","state_est");
    mxArray *qpstruct = getClassProp(pJMPC,"jMPC","QP");
    mxArray *indstruct = getClassProp(pJMPC,"jMPC","index");
    mxArray *con = getClassProp(pJMPC,"jMPC","constraints");
    mxArray *initial = getClassProp(pJMPC,"jMPC","initial");
    mxArray *lin = getClassProp(pJMPC,"jMPC","lin");
    mxArray *sinitial = getClassProp(pJSIM,"jSIM","initial");    
    mxArray *mpcopts = getClassProp(pJMPC,"jMPC","mpcopts");
    mxArray *plant = getClassProp(pJSIM,"jSIM","Plant"); 
    
    //Quick Checks
    if(mxIsEmpty(model) || mxIsEmpty(getClassProp(model,"jSS","A")))
        mexErrMsgTxt("MPC Controller Model appears to be empty!");
    if(mxIsClass(plant,"jNL"))
        mexErrMsgTxt("The jMPC MEX Engine is for Linear Plant (jSS) Simulations Only. You have supplied a jNL Nonlinear Plant.");
    if(mxIsEmpty(plant) || mxIsEmpty(getClassProp(plant,"jSS","A")))
        mexErrMsgTxt("Simulation Plant appears to be empty!");    
    #ifdef SINGLE_PREC
        if(mxGetClassID(getClassProp(model,"jSS","A")) == mxDOUBLE_CLASS)
            mexErrMsgTxt("MPC Model is Double Precision - This function is for single precision simulations, please use jMPCEngine for double precision");
        if(mxGetClassID(getClassProp(plant,"jSS","A")) == mxDOUBLE_CLASS)
            mexErrMsgTxt("Simulation Plant is Double Precision - This function is for single precision simulations, please use jMPCEngine for double precision");
    #else
        if(mxGetClassID(getClassProp(model,"jSS","A")) == mxSINGLE_CLASS)
            mexErrMsgTxt("MPC Model is Single Precision - This function is for double precision simulations, please use jMPCSEngine for single precision");
        if(mxGetClassID(getClassProp(plant,"jSS","A")) == mxSINGLE_CLASS)
            mexErrMsgTxt("Simulation Plant is Single Precision - This function is for double precision simulations, please use jMPCSEngine for single precision");
    #endif
    
    //Model Properties
    modelA = (realT*)mxGetData(getClassProp(model,"jSS","A"));                        //Augmented Model A Matrix
    modelB = (realT*)mxGetData(getClassProp(model,"jSS","B"));                        //Augmented Model B Matrix
    //Prediction Numbers
    Np = (unsigned int)*(realT*)mxGetData(getClassProp(pJMPC,"jMPC","Np"));           //Prediction Horizon
    Nb = (unsigned int)*(realT*)mxGetData(getClassProp(pJMPC,"jMPC","Nb"));           //# of Blocking Moves
    //Prediction Matrices
    F = (realT*)mxGetData(getStructArg(pred,"pred","F"));                             //Prediction F Matrix
    Fq = (realT*)mxGetData(getStructArg(pred,"pred","Fq"));                           //Prediction F Matrix (only controlled outputs)
    Phiv = (realT*)mxGetData(getStructArg(pred,"pred","Phiv"));                       //Disturbance Prediction Phiv Matrix
    Phiqv = (realT*)mxGetData(getStructArg(pred,"pred","Phiqv"));                     //Disturbance Prediction Phiqv Matrix (only controlled outputs)
    //Problem Size Information
    n_in = (unsigned int)*(realT*)mxGetData(getStructArg(sizes,"sizes","n_in"));      //# of Inputs
    nm_in = (unsigned int)*(realT*)mxGetData(getStructArg(sizes,"sizes","nm_in"));    //# of Manipulated Inputs
    nm_dist = (unsigned int)*(realT*)mxGetData(getStructArg(sizes,"sizes","nm_dist"));//# of Measured Disturbance Inputs
    n_out = (unsigned int)*(realT*)mxGetData(getStructArg(sizes,"sizes","n_out"));    //# of Outputs
    nm_out = (unsigned int)*(realT*)mxGetData(getStructArg(sizes,"sizes","nm_out"));  //# of Measured Outputs
    nq_out = (unsigned int)*(realT*)mxGetData(getStructArg(sizes,"sizes","nq_out"));  //# of Controlled Outputs
    states = (unsigned int)*(realT*)mxGetData(getStructArg(sizes,"sizes","states"));  //# of States
    //State Estimation
    IKC =  (realT*)mxGetData(getStructArg(state_est,"state_est","IKC"));              //State Pick off Matrix for State Estimator
    Kest =  (realT*)mxGetData(getStructArg(state_est,"state_est","Kest"));            //State Estimation Gain    
    //QP Information
    PhiTQ = (realT*)mxGetData(getStructArg(qpstruct,"QP","PhiTQ"));                   //Prediction Matrix Phi' * Weighting Matrix Q
    SE = (realT*)mxGetData(getStructArg(qpstruct,"QP","S"));                          //Setpoint Expansion Matrix
    H = (realT*)mxGetData(getStructArg(qpstruct,"QP","H"));                           //QP H Matrix
    R = (realT*)mxGetData(getStructArg(qpstruct,"QP","R"));                           //Prefactored Cholesky decomposition of QP H 
    Hscale = (realT*)mxGetData(getStructArg(qpstruct,"QP","Hscale"));                 //Decision variable scaling vector
    Ascale = (realT*)mxGetData(getStructArg(qpstruct,"QP","Ascale"));                 //Constraint scaling vector
    ndec = (unsigned int)mxGetM(getStructArg(qpstruct,"QP","H"));                     //Number of Decision Variables
    qpsolver = (unsigned int)*(realT*)mxGetData(getStructArg(qpstruct,"QP","solver"));//QP Solver
    //Index Information
    idelu = (unsigned int*)mxGetData(getStructArg2(indstruct,"index","mex","idelu"));            //Index of finite delu constraints
    iumin = (unsigned int*)mxGetData(getStructArg2(indstruct,"index","mex","iumin"));            //Index of finite umin constraints
    iumax = (unsigned int*)mxGetData(getStructArg2(indstruct,"index","mex","iumax"));            //Index of finite umax constraints
    iymin = (unsigned int*)mxGetData(getStructArg2(indstruct,"index","mex","iymin"));            //Index of finite ymin constraints 
    iymax = (unsigned int*)mxGetData(getStructArg2(indstruct,"index","mex","iymax"));            //Index of finite ymax constraints 
    ndelu = (unsigned int)mxGetNumberOfElements(getStructArg2(indstruct,"index","mex","idelu")); //Number of finite delu constraints
    numin = (unsigned int)mxGetNumberOfElements(getStructArg2(indstruct,"index","mex","iumin")); //Number of finite umin constraints
    numax = (unsigned int)mxGetNumberOfElements(getStructArg2(indstruct,"index","mex","iumax")); //Number of finite umax constraints
    nymin = (unsigned int)mxGetNumberOfElements(getStructArg2(indstruct,"index","mex","iymin")); //Number of finite ymin constraints
    nymax = (unsigned int)mxGetNumberOfElements(getStructArg2(indstruct,"index","mex","iymax")); //Number of finite ymax constraints    
    iman_u = (unsigned int*)mxGetData(getStructArg2(indstruct,"index","mex","iman_u"));          //Index of manipulated inputs in u
    imeas_dist = (unsigned int*)mxGetData(getStructArg2(indstruct,"index","mex","imeas_dist"));  //Index of measured disturbances in u
    isq_out = (unsigned int*)mxGetData(getStructArg2(indstruct,"index","mex","isq_out"));        //Index of states & controlled outputs in del_xm
    iumeasp_out = (unsigned int*)mxGetData(getStructArg2(indstruct,"index","mex","iumeasp_out"));//Index of umeasured outputs (+ #states) in del_xm
    ismeasp_out = (unsigned int*)mxGetData(getStructArg2(indstruct,"index","mex","ismeasp_out"));//Index of states & measured outputs (+ #states) in del_xm
    numeas_out = (unsigned int)mxGetNumberOfElements(getStructArg2(indstruct,"index","mex","iumeas_out")); //Number of unmeasured outputs       
    //Constraints
    A = (realT*)mxGetData(getStructArg(con,"constraints","A"));                         //QP Constraint Matrix A
    ucon = (realT*)mxGetData(getStructArg(con,"constraints","u"));                      //U constraints
    Tin = (realT*)mxGetData(getStructArg(con,"constraints","Tin"));                     //Lower Triangular Matrix to sum del_u components
    bcon = (realT*)mxGetData(getStructArg(con,"constraints","bcon"));                   //Constant part of QP Constraint vector b  
    uncon = (unsigned int)mxIsLogicalScalarTrue(getStructArg(con,"constraints","uncon")); //Enable unconstrained problem
    mc = (unsigned int)mxGetM(getStructArg(con,"constraints","A"));                     //Number of Constraints                                          
    soft = (unsigned int)*(realT*)mxGetData(getStructArg2(con,"constraints","soft","no")); //Number of soft output constraints
    //Initial Values
    init_u = (realT*)mxGetData(getStructArg(initial,"initial","u"));                  //Initial u (at sample 0 - input)
    init_up = (realT*)mxGetData(getStructArg(initial,"initial","up"));                //Initial u including measured disturbances
    init_xm = (realT*)mxGetData(getStructArg(initial,"initial","xm"));                //Initial xm (model states)
    init_del_xm = (realT*)mxGetData(getStructArg(initial,"initial","del_xm"));        //Initial del_xm (model states increment)    
    init_ym = (realT*)mxGetData(getStructArg(initial,"initial","ym"));                //Initial ym (model output)              
    init_xp = (realT*)mxGetData(getStructArg(sinitial,"initial","xp"));               //Initial xp (plant states)
    init_yp = (realT*)mxGetData(getStructArg(sinitial,"initial","yp"));               //Initial yp (plant output)
    //MPC Options
    look_ahead = (unsigned int)*(realT*)mxGetData(getStructArg(mpcopts,"mpcopts","LookAhead")); //Enable Setpoint lookahead (acausal)
    warm = (unsigned int)*(realT*)mxGetData(getStructArg(mpcopts,"mpcopts","QPWarmStart"));     //Enable QP Warm Starting
    qpverbose = (unsigned int)*(realT*)mxGetData(getStructArg(mpcopts,"mpcopts","QPVerbose"));  //QP Verbosity
    qptol = *(realT*)mxGetData(getStructArg(mpcopts,"mpcopts","QPTol"));                        //QP Termination Tolerance
    qpmaxiter = (unsigned int)*(realT*)mxGetData(getStructArg(mpcopts,"mpcopts","QPMaxIter"));  //Max QP iteratons    
    warn = (unsigned int)mxIsLogicalScalarTrue(getStructArg(mpcopts,"mpcopts","mexWarn"));      //Mex Warning Enable
    //Linearization
    u_op = (realT*)mxGetData(getStructArg(lin,"lin","u_op"));       //U Linearization Point
    y_op = (realT*)mxGetData(getStructArg(lin,"lin","y_op"));       //Y Linearization Point    
    //Simulation Variables
    plantA = (realT*)mxGetData(getClassProp(plant,"jSS","A"));      //Plant A Matrix
    plantB = (realT*)mxGetData(getClassProp(plant,"jSS","B"));      //Plant B Matrix
    plantC = (realT*)mxGetData(getClassProp(plant,"jSS","C"));      //Plant C Matrix
    udist = (realT*)mxGetData(getClassProp(pJSIM,"jSIM","udist"));  //Input Disturbances
    ydist = (realT*)mxGetData(getClassProp(pJSIM,"jSIM","ydist"));  //Output Disturbances
    mdist = (realT*)mxGetData(getClassProp(pJSIM,"jSIM","mdist"));  //Measured Disturbances
    setp = (realT*)mxGetData(getClassProp(pJSIM,"jSIM","setp"));    //Setpoint   
    setlen = (unsigned int)mxGetM(getClassProp(pJSIM,"jSIM","setp"));           //Setpoint vector length
    Tfinal = (unsigned int)*(realT*)mxGetData(getClassProp(pJSIM,"jSIM","T"));  //Simulation Time     
    veclen = Tfinal + 1;  //Plot Vector Length (Tfinal + 1)
    
    #ifdef DEBUG
        mexPrintf("n_in %d\nnm_in %d\nnm_dist %d\nstates %d\nn_out %d\nnm_out %d\nnq_out %d\n",n_in,nm_in,nm_dist,states,n_out,nm_out,nq_out);
        mexPrintf("mc %d\nndec %d\nwarm %d\nsoft %d\nlookahead %d\nuncon %d\nTfinal %d\nveclen %d\n",mc,ndec,warm,soft,look_ahead,uncon,Tfinal,veclen);
    #endif
     
    //Check for valid solver (1 wright, 2 mehrotra)
    if(!qpsolver)
        mexErrMsgTxt("The jMPC MEX Engine Only Supports the Supplied Wright and Mehrotra Solvers");
}

//-- Collect dynamic memory required by the MPC engine --//
int getMPCMemory()
{
    int i, len, indx;
 
    //Determine Sizes
    statesn     = states + n_out;
    statesnm    = states + nm_out;
    statesnq    = states + nq_out;
    Np_out      = Np * n_out;
    Np_qout     = Np * nq_out;
    Nb_in       = Nb * nm_in;

    //Collect Memory for MPC Variables
    len = (mwSize)(nm_in+2*n_in+2*nm_dist+(Nb_in+soft)+2*states+2*statesn+statesnm+3*n_out+Np_out+statesnq+Np_qout+nq_out+2*ndec + numeas_out);    
    if(!uncon)
        len += (mwSize)(Nb_in+2*mc);

    #ifdef DEBUG
        mexPrintf("MPC vec len %d\n",len);
    #endif
    mpc_ptr = (realT*)_aligned_malloc(len*sizeof(realT),16);    
	if(mpc_ptr == NULL)
        return 0;
    
    //Initialise all values to 0
    for(i=0;i<len;i++)
        mpc_ptr[i] = 0.0;    
    
    //Allocate MPC Memory
    indx = 0;
    u               = &mpc_ptr[indx]; indx += nm_in;
    del_u           = &mpc_ptr[indx]; indx += (Nb_in+soft);    
    xm              = &mpc_ptr[indx]; indx += states;
    del_xm          = &mpc_ptr[indx]; indx += statesn;
    ym              = &mpc_ptr[indx]; indx += n_out;
    xp              = &mpc_ptr[indx]; indx += states;
    yp              = &mpc_ptr[indx]; indx += n_out;    
	Kys				= &mpc_ptr[indx]; indx += statesnm;
	K  		        = &mpc_ptr[indx]; indx += statesn;
    ys              = &mpc_ptr[indx]; indx += n_out;
    y0              = &mpc_ptr[indx]; indx += Np_out;
    qp_del_xm       = &mpc_ptr[indx]; indx += statesnq;
    Ssetp           = &mpc_ptr[indx]; indx += Np_qout;
    sp              = &mpc_ptr[indx]; indx += nq_out;
    x               = &mpc_ptr[indx]; indx += ndec;
    fvec            = &mpc_ptr[indx]; indx += ndec;
    up              = &mpc_ptr[indx]; indx += n_in;
    umeas_out       = &mpc_ptr[indx]; indx += numeas_out;
	um				= &mpc_ptr[indx]; indx += n_in;
	v				= &mpc_ptr[indx]; indx += nm_dist;
	del_v			= &mpc_ptr[indx]; indx += nm_dist;
    if(!uncon) {
        del         = &mpc_ptr[indx]; indx += Nb_in;
        bvec        = &mpc_ptr[indx]; indx += mc;
        bcheck      = &mpc_ptr[indx]; indx += mc;               
    }
    
    #ifdef DEBUG
        mexPrintf("statesn %d\nstatesnm %d\nstatesnq %d\nNp_out %d\nNp_qout %d\nNb_in %d\n",statesn,statesnm,statesnq,Np_out,Np_qout,Nb_in);
    #endif

    return 1;
}

//-- Free dynamic memory used by the MPC engine --//
void freeMPCMemory()
{
     _aligned_free(mpc_ptr);
}
