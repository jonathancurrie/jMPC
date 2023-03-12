/* jMPC Controller Engine Implemented as a C S Function 
 *
 * Jonathan Currie (C)
 * Control Engineering 2011-2013 */

#define S_FUNCTION_NAME jMPC_sfcn
#define S_FUNCTION_LEVEL 2

#include <simstruc.h>   //Default Simulink File
#include <windows.h>
#include "qp_header.h"  
#include "qp_mem.h"
#include "jmath.h"
#include "mpc_utils.h"

//#define DEBUG

//***** Parameter Definitions *****//
#define NUM_PARAMS 14
//MPC Standard Args
#define ModelA(S)           ssGetSFcnParam(S, 0)
#define ModelB(S)           ssGetSFcnParam(S, 1)
#define Ts(S)               ssGetSFcnParam(S, 2)
#define Np(S)               ssGetSFcnParam(S, 3)
#define Nb(S)               ssGetSFcnParam(S, 4)
#define pred_struct(S)      ssGetSFcnParam(S, 5)
#define sizes_struct(S)     ssGetSFcnParam(S, 6)
#define stateest_struct(S)  ssGetSFcnParam(S, 7)
#define qp_struct(S)        ssGetSFcnParam(S, 8)
#define index_struct(S)		ssGetSFcnParam(S, 9)
#define cons_struct(S)      ssGetSFcnParam(S, 10)
#define initial_struct(S)   ssGetSFcnParam(S, 11)
#define mpcopts_struct(S)   ssGetSFcnParam(S, 12)
#define lin_struct(S)		ssGetSFcnParam(S, 13)

//***** Global Variables & Pointers *****//
double *mpc_ptr;    //Dynamic memory pointer
//Model
double *modelA, *modelB, Ts;
//Horizons
unsigned int Np, Nb;
//Sizes
unsigned int n_in, n_out, nm_in, nm_dist, nm_out, nq_out, mc, ndec;                    
unsigned int states, statesn, statesnm, statesnq, Np_out, Np_qout, Nb_in;
//Prediction
double *F, *Fq, *Phiv, *Phiqv;
//State Estimation
double *IKC, *Kest;
//QP
double *PhiTQ, *SE, *H, *R, *Hscale, *Ascale;
//Constraints
double *A, *Tin, *bcon, *ucon;
//Loop Variables
double *u, *del_u, *xm, *del_xm;
//Index Vectors
unsigned int *idelu, *iumin, *iumax, *iymin, *iymax;
unsigned int ndelu, numin, numax, nymin, nymax, numeas_out;
unsigned int *iman_u, *imeas_dist, *isq_out, *iumeasp_out, *ismeasp_out;        
//Linearization Biases
double *u_op, *y_op;
//Mode & Options
unsigned int look_ahead, soft, uncon, local_warm, warm, warn, qpsolver, qpverbose, qpmaxiter;      
double qptol;    
//Engine Variables
double *K, *Kys, *ys, *y0, *qp_del_xm, *Ssetp, *x;
double *fvec, *del, *bvec, *bcheck, *up, *umeas_out, *um, *old_v, *del_v;
//Misc
unsigned int stepcount;   //Simulation Step Count
//Timer Variables
LARGE_INTEGER start, stop, freq;

/*====================*
 * S-function methods *
 *====================*/

static void mdlInitializeSizes(SimStruct *S)
{
    unsigned int i = 0, nParam;
    //Collect Problem Sizes
    n_in = (unsigned int)*mxGetPr(getStructArg(sizes_struct(S),"sizes","n_in"));               //# of Inputs
    nm_in = (unsigned int)*mxGetPr(getStructArg(sizes_struct(S),"sizes","nm_in"));             //# of Manipulated Inputs
    nm_dist = (unsigned int)*mxGetPr(getStructArg(sizes_struct(S),"sizes","nm_dist"));         //# of Measured Disturbance Inputs
    n_out = (unsigned int)*mxGetPr(getStructArg(sizes_struct(S),"sizes","n_out"));             //# of Outputs
    nm_out = (unsigned int)*mxGetPr(getStructArg(sizes_struct(S),"sizes","nm_out"));           //# of Measured Outputs
    nq_out = (unsigned int)*mxGetPr(getStructArg(sizes_struct(S),"sizes","nq_out"));           //# of Controlled Outputs
    states = (unsigned int)*mxGetPr(getStructArg(sizes_struct(S),"sizes","states"));           //# of States
    
    #ifdef DEBUG
        ssPrintf("nin %d\nnm_in %d\nnm_dist %d\nnout %d\nnqout %d\nnmout %d\nstates %d\nTs %f\n",n_in,nm_in,nm_dist,n_out,nq_out,nm_out,states,Ts);
    #endif
    
    //Check correct number of parameters passed
    ssSetNumSFcnParams(S, NUM_PARAMS);  /* Number of expected parameters */
    nParam = ssGetNumSFcnParams(S);
    if (nParam != ssGetSFcnParamsCount(S)) {
        return;
    }
    //All parameters are not tunable
    for(i = 0; i < nParam; i++)
       ssSetSFcnParamTunable( S, i, 0 );   
    
    //Internal States
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    //Input Ports
    if (!ssSetNumInputPorts(S, 3)) return;
	ssSetInputPortWidth(S, 0, max(nm_dist,1));  //v
	ssSetInputPortWidth(S, 1, nq_out);   		//setp
    ssSetInputPortWidth(S, 2, nm_out);   		//yp
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
	ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortRequiredContiguous(S, 0, true); //BUG FIX 8/7/13
    ssSetInputPortRequiredContiguous(S, 1, true); 
    ssSetInputPortRequiredContiguous(S, 2, true);

    //Output Ports
    if (!ssSetNumOutputPorts(S, 7)) return;
    ssSetOutputPortWidth(S, 0, nm_in);  //u
    ssSetOutputPortWidth(S, 1, nm_in);  //delu
    ssSetOutputPortWidth(S, 2, n_out);  //ym
    ssSetOutputPortWidth(S, 3, states); //xm
    ssSetOutputPortWidth(S, 4, 1); //qpiter
    ssSetOutputPortWidth(S, 5, 1); //status
    ssSetOutputPortWidth(S, 6, 1); //time    
    ssSetOutputPortDataType(S, 4, SS_UINT32);
    ssSetOutputPortDataType(S, 5, SS_INT32);

    //Number of Sample Times
    ssSetNumSampleTimes(S, 1);
    
    //Work Vectors
    ssSetNumRWork(S, 0);  //won't work with RTW?
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    //No special options
    ssSetOptions(S, 0);
}


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, *mxGetPr(Ts(S)));
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
static void mdlStart(SimStruct *S)
{ 
    unsigned int indx = 0, i = 0, len = 0;
    //Quick Checks
    if(mxIsEmpty(ModelA(S)))
        mexErrMsgTxt("MPC Controller Model appears to be empty!");
    if(mxGetClassID(ModelA(S)) == mxSINGLE_CLASS)
        mexErrMsgTxt("This function is for double precision simulations");
    
    //Standard MPC Parameters
    modelA          = mxGetPr(ModelA(S));
    modelB          = mxGetPr(ModelB(S));
    Np              = (unsigned int)*mxGetPr(Np(S));
    Nb              = (unsigned int)*mxGetPr(Nb(S));
	//Prediction Structure
    F               = mxGetPr(getStructArg(pred_struct(S),"pred","F"));
    Fq              = mxGetPr(getStructArg(pred_struct(S),"pred","Fq"));
	Phiv            = mxGetPr(getStructArg(pred_struct(S),"pred","Phiv"));
	Phiqv           = mxGetPr(getStructArg(pred_struct(S),"pred","Phiqv"));
    //State Estimation Structure
    IKC             = mxGetPr(getStructArg(stateest_struct(S),"state_est","IKC"));
    Kest            = mxGetPr(getStructArg(stateest_struct(S),"state_est","Kest"));    
    //QP Structure
    PhiTQ           = mxGetPr(getStructArg(qp_struct(S),"QP","PhiTQ")); 
    SE              = mxGetPr(getStructArg(qp_struct(S),"QP","S"));
    H               = mxGetPr(getStructArg(qp_struct(S),"QP","H"));
    R               = mxGetPr(getStructArg(qp_struct(S),"QP","R"));
    Hscale          = mxGetPr(getStructArg(qp_struct(S),"QP","Hscale"));                 
    Ascale          = mxGetPr(getStructArg(qp_struct(S),"QP","Ascale"));                
    ndec            = (unsigned int)mxGetM(getStructArg(qp_struct(S),"QP","H"));
    qpsolver        = (unsigned int)*mxGetPr(getStructArg(qp_struct(S),"QP","solver")); 
	//Index Structure
	idelu           = (unsigned int*)mxGetData(getStructArg2(index_struct(S),"index","mex","idelu")); 
    iumin           = (unsigned int*)mxGetData(getStructArg2(index_struct(S),"index","mex","iumin")); 
    iumax           = (unsigned int*)mxGetData(getStructArg2(index_struct(S),"index","mex","iumax")); 
    iymin           = (unsigned int*)mxGetData(getStructArg2(index_struct(S),"index","mex","iymin"));  
    iymax           = (unsigned int*)mxGetData(getStructArg2(index_struct(S),"index","mex","iymax")); 
    ndelu           = (unsigned int)mxGetNumberOfElements(getStructArg2(index_struct(S),"index","mex","idelu")); 
    numin           = (unsigned int)mxGetNumberOfElements(getStructArg2(index_struct(S),"index","mex","iumin")); 
    numax           = (unsigned int)mxGetNumberOfElements(getStructArg2(index_struct(S),"index","mex","iumax")); 
    nymin           = (unsigned int)mxGetNumberOfElements(getStructArg2(index_struct(S),"index","mex","iymin")); 
    nymax           = (unsigned int)mxGetNumberOfElements(getStructArg2(index_struct(S),"index","mex","iymax")); 
    iman_u          = (unsigned int*)mxGetData(getStructArg2(index_struct(S),"index","mex","iman_u"));            
    imeas_dist      = (unsigned int*)mxGetData(getStructArg2(index_struct(S),"index","mex","imeas_dist"));   
    isq_out         = (unsigned int*)mxGetData(getStructArg2(index_struct(S),"index","mex","isq_out"));          
    iumeasp_out     = (unsigned int*)mxGetData(getStructArg2(index_struct(S),"index","mex","iumeasp_out"));  
    ismeasp_out     = (unsigned int*)mxGetData(getStructArg2(index_struct(S),"index","mex","ismeasp_out"));  
    numeas_out      = (unsigned int)mxGetNumberOfElements(getStructArg2(index_struct(S),"index","mex","iumeas_out")); 
    //Constraints Structure
    A               = mxGetPr(getStructArg(cons_struct(S),"constraints","A"));  
    ucon            = mxGetPr(getStructArg(cons_struct(S),"constraints","u"));               
    Tin             = mxGetPr(getStructArg(cons_struct(S),"constraints","Tin"));               
    bcon            = mxGetPr(getStructArg(cons_struct(S),"constraints","bcon"));  
    if(mxIsLogical(getStructArg(cons_struct(S),"constraints","uncon")))
        uncon       = (unsigned int)mxIsLogicalScalarTrue(getStructArg(cons_struct(S),"constraints","uncon")); 
    else
        uncon       = (unsigned int)*mxGetPr(getStructArg(cons_struct(S),"constraints","uncon"));      
    mc              = (unsigned int)mxGetM(getStructArg(cons_struct(S),"constraints","A"));                                                    
    soft            = (unsigned int)*mxGetPr(getStructArg2(cons_struct(S),"constraints","soft","no")); 
    //Initial Value Structure
    u               = mxGetPr(getStructArg(initial_struct(S),"initial","u"));                       
    xm              = mxGetPr(getStructArg(initial_struct(S),"initial","xm"));              
    del_xm          = mxGetPr(getStructArg(initial_struct(S),"initial","del_xm"));
    //MPC Options
    look_ahead      = (unsigned int)*mxGetPr(getStructArg(mpcopts_struct(S),"mpcopts","LookAhead")); 
    warm            = (unsigned int)*mxGetPr(getStructArg(mpcopts_struct(S),"mpcopts","QPWarmStart"));   
    qpverbose       = (unsigned int)*mxGetPr(getStructArg(mpcopts_struct(S),"mpcopts","QPVerbose")); 
    qptol           = *mxGetPr(getStructArg(mpcopts_struct(S),"mpcopts","QPTol"));              
    qpmaxiter       = (unsigned int)*mxGetPr(getStructArg(mpcopts_struct(S),"mpcopts","QPMaxIter"));  
    warn            = (unsigned int)mxIsLogicalScalarTrue(getStructArg(mpcopts_struct(S),"mpcopts","mexWarn"));        
    //Linearization Structure  
    u_op            = mxGetPr(getStructArg(lin_struct(S),"lin","u_op"));
    y_op            = mxGetPr(getStructArg(lin_struct(S),"lin","y_op")); 
    
    //Sizes
    statesn         = states+n_out;
	statesnm		= states+nm_out;
	statesnq		= states+nq_out;
    Nb_in           = Nb*nm_in;
    Np_out          = Np*n_out;
    Np_qout         = Np*nq_out;
    stepcount       = 0;
    local_warm      = 0;
    
    #ifdef DEBUG
        ssPrintf("statesn %d\nstatesnm %d\nstatesnq %d\nNp_out %d\nNp_qout %d\nNb_in %d\n",statesn,statesnm,statesnq,Np_out,Np_qout,Nb_in);
    #endif  
    
    //***** Collect MPC Memory *****//
    len = (statesn + statesnm + n_out + Np_out + statesnq + Np_qout + 3*ndec + numeas_out + 2*n_in + 2*nm_dist);
    if(!uncon)
        len += Nb_in + 2*mc;
    if((len > 1e8) || (len < 1)) {
        ssPrintf("MPC vector length = %d\n",len);
        ssSetErrorStatus(S,"Memory vector length calculated as impossible size!");
        return;
    }
    #ifdef DEBUG
        ssPrintf("MPC vector length = %d\n",len);
    #endif
    
    mpc_ptr = (double*)_aligned_malloc(len*sizeof(double),16);
    if(mpc_ptr == NULL) {
        ssSetErrorStatus(S,"malloc returned NULL");
        return;
    }
    for(i=0;i<len;i++)
        mpc_ptr[i] = 0.0;
    //Allocate MPC Memory
    indx = 0;
    Kys				= &mpc_ptr[indx]; indx += statesnm;
	K           	= &mpc_ptr[indx]; indx += statesn;
    ys              = &mpc_ptr[indx]; indx += n_out;
    y0              = &mpc_ptr[indx]; indx += Np_out;
    qp_del_xm		= &mpc_ptr[indx]; indx += statesnq;
    Ssetp           = &mpc_ptr[indx]; indx += Np_qout;
    x               = &mpc_ptr[indx]; indx += ndec;
    del_u           = &mpc_ptr[indx]; indx += ndec;
    fvec            = &mpc_ptr[indx]; indx += ndec;	
    umeas_out       = &mpc_ptr[indx]; indx += numeas_out;
	um				= &mpc_ptr[indx]; indx += n_in;
	up				= &mpc_ptr[indx]; indx += n_in;
	old_v           = &mpc_ptr[indx]; indx += nm_dist;
    del_v           = &mpc_ptr[indx]; indx += nm_dist;
    if(!uncon) {    
        del             = &mpc_ptr[indx]; indx += Nb_in;
        bvec            = &mpc_ptr[indx]; indx += mc;
        bcheck          = &mpc_ptr[indx]; indx += mc;
    }
    
    for(i = 0; i < nm_dist; i++){
        old_v[i] = u_op[imeas_dist[i]];
    }            
    if(!getQPMemory(mc,ndec)) {
        ssSetErrorStatus(S,"Could not allocate QP memory");
        return;
    }       
    //Initialize Timer
    if(!QueryPerformanceFrequency(&freq))
        ssPrintf("\nTimer Could not retrieve System Frequency!\n");
}
#endif /*  MDL_START */

static void mdlOutputs(SimStruct *S, int tid)
{   
    //I/O Signals
	double              *v		    = (double*)ssGetInputPortSignal(S,0);
    double              *setp       = (double*)ssGetInputPortSignal(S,1);
    double              *yp         = (double*)ssGetInputPortSignal(S,2);
    double              *Ou         = (double*)ssGetOutputPortSignal(S,0);
    double              *Odel_u     = (double*)ssGetOutputPortSignal(S,1);
    double              *Oym        = (double*)ssGetOutputPortSignal(S,2);
    double              *Oxm        = (double*)ssGetOutputPortSignal(S,3);
    unsigned int        *Oqpiter    = (unsigned int*)ssGetOutputPortSignal(S,4);
    int                 *Ostatus    = (int*)ssGetOutputPortSignal(S,5);
    double              *Otime      = (double*)ssGetOutputPortSignal(S,6);
    //Internal Variables
    unsigned int i, j, index, offset = 0, global_fail;
	double y0mul = 0.0;

    //Re-initialize status
    *Oqpiter = 0; *Ostatus = 0;
    //Start Timer
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&start);   
    
    //----- State Estimator Update -----//
    //K(ismeasp_out) = J.state_est.Kest*(yp-y_op);
    for(i = 0; i < nm_out; i++)
        ys[i] = yp[i]-y_op[i]; 
    jmv(statesnm,nm_out,1.0,Kest,ys,0.0,Kys);	
    for(i = 0; i < statesnm; i++)
        K[ismeasp_out[i]] = Kys[i];
    //K(iumeasp_out) = del_xm(iumeasp_out); 
    for(i = 0; i < (n_out-nm_out); i++)
        K[iumeasp_out[i]] = del_xm[iumeasp_out[i]];		
    //del_xm = J.state_est.IKC*del_xm + K        
    jmv(statesn,statesn,1.0,IKC,del_xm,1.0,K);
    memcpy(del_xm,K,statesn*sizeof(double));

    //----- Update RHS -----//				
    if(nm_dist > 0) {
        //Difference measured disturbance
        for(i = 0; i < nm_dist; i++) {            
            del_v[i] = v[i] - old_v[i];
            old_v[i] = v[i];
        }
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
    //setp = J.QP.S*setp(k,:)'-y0;           
    memcpy(Ssetp,y0,Np_qout*sizeof(double));
    jmv(Np_qout,nq_out,1.0,SE,setp,-1.0,Ssetp);
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
        memcpy(x,fvec,ndec*sizeof(double));
        jtris(R,x,ndec);        
        //Check Ax > b
        jmv(mc,ndec,-1.0,A,x,0.0,bcheck);
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
                ssPrintf("Sample %d:\n",stepcount+1);
            //Solve QP
            if(qpsolver==2)
                *Ostatus = quad_mehrotra(H, fvec, A, bvec, mc, ndec, del_u, lambda, tslack, local_warm, qptol, qpmaxiter, qpverbose, Oqpiter);
            else
                *Ostatus = quad_wright(H, fvec, A, bvec, mc, ndec, del_u, lambda, tslack, local_warm, qptol, qpmaxiter, qpverbose, Oqpiter);
            if(warn && qpverbose==0) {
                switch(*Ostatus)
                {
                    case -1: ssPrintf("Sample %d: Maximum Iterations Exceeded\n",stepcount+1); break;
                    case -2: ssPrintf("Sample %d: Cholesky Failure\n",stepcount+1); break;
                    case -3: ssPrintf("Sample %d: Numerical Errors Encountered\n",stepcount+1); break;
                    case -4: ssPrintf("Sample %d: No Feasible Solution Found\n",stepcount+1); break;
                    case -5: ssPrintf("Sample %d: Slow Progress - Early Exit\n",stepcount+1); break;
                    case -6: ssPrintf("Sample %d: Unknown QP Exit\n",stepcount+1); break;
                }
            }
            if(warm) {
                //Add a little to lambda, t to allow solver to find new constraints easier
                for(i=0;i<mc;i++) {
                    if(lambda[i] < 0.1)
                        lambda[i] += 0.15;    
                    if(tslack[i] < 0.1)
                        tslack[i] += 0.15; 
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
        }
        
        //----- Saturate Inputs -----//
        if(ndelu) {
            for(i = 0, j = 0; i < Nb_in; i++) {        
                if(del_u[i] > ucon[j+2*nm_in])   //saturate delu at constraints
                    del_u[i] = ucon[j+2*nm_in];
                else if(del_u[i] < -ucon[j+2*nm_in])
                    del_u[i] = -ucon[j+2*nm_in];
                j++;
                if(j >= nm_in)
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
            Odel_u[i] = del_u[i];
        }
    }
    //Unconstrained Problem
    else {
        //x = J.QP.R\(J.QP.R'\f)
        memcpy(x,fvec,ndec*sizeof(double));
        jtris(R,x,ndec); 
        for(i = 0; i < Nb_in; i++)
            del_u[i] = -x[i]; //Global solution is -x
        //Sum del_u to get u
        for(i = 0; i < nm_in; i++) {
            u[i] += del_u[i];
            Odel_u[i] = del_u[i];
        }
    }
	
	//Add Measured Disturbance
    if(nm_dist > 0) {
        for(i = 0; i < nm_in; i++) {
            um[iman_u[i]] = del_u[i];       //place del_u in model input
            up[i] = u[i];                   //place u in plant input
        }
        for(i = 0; i < nm_dist; i++)
            um[imeas_dist[i]] = del_v[i]; 	//place del_v in model input				
    }
    else {
        for(i = 0; i < n_in; i++) {
            um[i] = del_u[i];
            up[i] = u[i];
        }
    }

    //----- Simulate Model -----//
    jmv(statesn,statesn,1.0,modelA,del_xm,0.0,K);
    jmv(statesn,n_in,1.0,modelB,um,1.0,K);
    memcpy(del_xm,K,statesn*sizeof(double));   
	
	//Process Inputs
	for(i = 0; i < nm_in; i++)
		Ou[i] = up[i] + u_op[iman_u[i]]; 
    //----- Process Outputs -----// 
	for(i = 0; i < states; i++) {
		xm[i] += del_xm[i];
		Oxm[i] = (xm[i]);
    }
    for(i = states, index = 0; i < statesn; i++, index++)
        Oym[index] = del_xm[i];
    
    //Record Iteration Elapsed Time
    QueryPerformanceCounter(&stop);
    *Otime =(double)(stop.QuadPart-start.QuadPart)*1e3/(double)freq.QuadPart;
    //Increment Simulation Counter
    stepcount++; 
}      

static void mdlTerminate(SimStruct *S)
{
    //Free Allocated Memory
    _aligned_free(mpc_ptr);
    freeQPMemory();
}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif