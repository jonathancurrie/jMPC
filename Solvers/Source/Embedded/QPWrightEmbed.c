/* QPWRIGHT EMBED - Implementation of Wright's Method to Solve Embedded QPs
 * Copyright (C) Jonathan Currie 2012-2013 (I2C2) 
 */

//EMBED START
//External Constants
extern const realT H[], A[], tol, MACH_EPS, pHAmax;
extern const intT maxiter, ndec, mc;
#if ACCELQP > 0
extern const realT At[];
#if ACCELQP > 1
extern const realT AtA[];
extern const intT AtAidx[];
#endif
#endif

//Constants Used in the Algorithm
#define AMAX 0.9995
#define AOKMAX 0.999995
#define ADEC 0.1 
#define SIGMAX 0.99999 
#define CHOLMAX 2

//Local Functions
realT solveStepLength(realT *lam, realT *del_lam, realT *t, realT *del_t, intT mc);

//QP Solver
intT qpwright_embed(const realT *f, const realT *b, realT *z, realT *lam, realT *t, intT warm, uintT *runiter)
{      
    //Internal Variables
    intT i,j, iter, cholfail = 0, status=1;    
    //Calculation Variables
    realT alpha = 1.0, alpham1, ascale = 1.0, inftol = tol*10.0;
    realT sigma = 0.1, phi, mu, mu_old, musig, WARMVAL;
    realT mr2_2, mr2_1 = 100.0, mr2 = 10.0, mr1, pmax;
    
    //Determine max problem data value
    pmax = max(vecmax(f,ndec),pHAmax);
    pmax = max(vecmax(b,mc),pmax);
    //Set Heuristic Warm Start Value
    WARMVAL = pmax > 1 ? sqrt(pmax) : 0.5;
    
    //Initialize Variables Based on Warm Start Mode
    switch(warm) {
        case 0: //none
            for(i=0;i<ndec;i++)
                z[i] = 0.0;
            for(i=0;i<mc;i++) {
                lam[i] = WARMVAL;
                t[i] = WARMVAL;
            }            
            break;
        case 1: //primal only
            for(i=0;i<mc;i++) {
                lam[i] = WARMVAL;
                t[i] = WARMVAL;
            }
            break;
        case 2: //primal and dual
            for(i=0;i<mc;i++)
                t[i] = WARMVAL;
            break;
        case 3: //primal, dual and slack
            break;
    }            
    //Calculate Initial Residuals
    memcpy(r1,f,ndec*sizeof(realT));
    memcpy(r2,b,mc*sizeof(realT));
    //r1 = -H*z-f-A'*lam
    jmv(ndec,ndec,-1.0,H,z,-1.0,r1);
    jtmv(ndec,mc,-1.0,A,lam,1.0,r1);
    //r2 = -A*z + b
    #if ACCELQP > 0
        jtmv(mc,ndec,-1.0,At,z,1.0,r2);
    #else
        jmv(mc,ndec,-1.0,A,z,1.0,r2);
    #endif
    
    //Starting mu
    mu = jdot(mc,t,lam)/mc; //t'*lam/mc    
    //Begin Searching
    for(iter = 1; iter <= maxiter; iter++) 
    {    
		j = 0; musig = mu*sigma;
		//Copy Values
        memcpy(del_z,r1,ndec*sizeof(realT));
		memcpy(RQP,H,ndec*ndec*sizeof(realT));	        
        for(i = 0; i < mc; i++) {
            ilamt[i] = t[i]/lam[i]; //ilam.*t
            lamt[i] = lam[i]/t[i]; //lam./t
            mesil[i] = musig/lam[i]; //mu*sigma/lam
        }

        //Form Linear System (H+A'*diag(lam/.t)*A)
        #if ACCELQP > 1
            jtmvs(ndec*ndec,mc,AtAidx,1.0,AtA,lamt,1.0,RQP);            
        #else
            formLinSys(A,lamt,ndec,mc,RQP);
        #endif
        //lamt.*(r2+b-mesil)
		for(i = 0; i < mc; i++) {
			r2[i] = lamt[i] *(r2[i] - mesil[i]);
			del_lam[i] = -r2[i];
		}
        //A'*IGR2+r1
        jtmv(ndec,mc,1.0,A,r2,1.0,del_z);

		//RQP\del_z
        if(cholSolve(RQP, del_z, ndec) < 1) {
            cholfail++; 
            //Check for max cholesky failures
            if(cholfail > CHOLMAX) {
                status = -2;//indicate failure
                break; //give up
            }
            //Pull back max step size
            ascale -= 0.1;
        }
		//-IGA*del_z+IGR2
        for(i=0;i<mc;i++) {
            #if ACCELQP > 0
                realT *Aptr = (realT*)&At[i*ndec];
                for(j=0;j<ndec;j++)                           
                    del_lam[i] += *Aptr++*lamt[i]*del_z[j];
            #else
                for(j=0;j<ndec;j++)                           
                    del_lam[i] += A[i + j*mc]*lamt[i]*del_z[j];
            #endif
            //-t+MeSIL-ilamt.*del_lam
            del_t[i] = -t[i]+mesil[i]-ilamt[i]*del_lam[i];           
        }
        //Solve for step-length
        alpha = solveStepLength(lam,del_lam,t,del_t,mc);
        //Check for numerical problems (alpha will go negative)
        if(alpha < MACH_EPS) {status = -3; break;}
        //Local Scaling
        alpha *= ascale; alpham1 = (1.0 - alpha);                         
        //Increment iterates
        for(i=0;i<ndec;i++) {
            z[i] += alpha*del_z[i];
            r1[i] *= alpham1;
        }
        for(i=0;i<mc;i++) {
            lam[i] += alpha*del_lam[i];
            t[i] += alpha*del_t[i];
        }
        //Update Residuals
        memcpy(r2,b,mc*sizeof(realT));
        //r2 = -A*z + b
        #if ACCELQP > 0
            jtmv(mc,ndec,-1.0,At,z,1.0,r2);                 
        #else
            jmv(mc,ndec,-1.0,A,z,1.0,r2);
        #endif
        
        //Complementarity Gap mu
        mu_old = mu;
		mu = jdot(mc,t,lam)/mc; //t'*lam/mc   
        //Infeasibility phi
        mr2_2 = mr2_1; mr2_1 = mr2;
        mr1 = vecmax(r1,ndec); mr2 = vecmmax(r2,t,mc);
        phi = (max(mr1,mr2) + mu*mc)/pmax;
        //Check for NaN
        if(mu != mu || phi != phi) {status = -3; break;}
        //Check Convergence
		if(mu <= tol && phi <= tol) {
            runiter[0] = iter;
			return 1;
		}
        //Check for Primal Infeasible
        if(iter > 6 && mr2/pmax > tol/10.0) {
            if(fabs(mr2-mr2_1)/mr2 < inftol && fabs(mr2_1-mr2_2)/mr2_1 < inftol) { 
                status = -4;
                break;
            }
        } 
        //Solve for new sigma
        mu_old = mu/mu_old;
        sigma = mu_old*mu_old*mu_old;
        if(sigma > SIGMAX)
            sigma = SIGMAX;    
    }
	//Check for expired iterations
    if(status==1) {
        status = -1;
        iter--;
    }
    //Return Error
    runiter[0] = iter;
    return status;     
}