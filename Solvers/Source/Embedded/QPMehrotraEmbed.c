/* QPWRIGHT EMBED - Implementation of Wright's Method to Solve Embedded QPs
 * Copyright (C) Jonathan Currie 2012-2013 (Control Engineering) 
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
intT qpmehrotra_embed(const realT *f, const realT *b, realT *z, realT *lam, realT *t, intT warm, uintT *runiter)
{      
    //Internal Variables
    intT i,j, iter, cholfail = 0, status=1;    
    //Calculation Variables
    realT Aztemp, Azsum, alpha = 1.0, alpham1, ascale = 1.0, inftol = tol*10.0;
    realT sigma = 0.1, phi = 1e30, phi1 = 1e30, phi2, mu, mu1, musig, WARMVAL;
    realT mr2_2, mr2_1 = 100.0, mr2 = 10.0, mr1, pmax = -1e20;
    
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
        }

        //Form Linear System (H+A'*diag(lam/.t)*A)
        #if ACCELQP > 1
            jtmvs(ndec*ndec,mc,AtAidx,1.0,AtA,lamt,1.0,RQP);            
        #else
            formLinSys(A,lamt,ndec,mc,RQP);
        #endif
        //lamt.*(r2+b-mesil)
		for(i = 0; i < mc; i++) {
			igr2[i] = r2[i]*lamt[i];
			del_lam[i] = -igr2[i];
		}
        //A'*IGR2+r1
        jtmv(ndec,mc,1.0,A,igr2,1.0,del_z);
        //Save RHS for use later
        memcpy(rhs,del_z,ndec*sizeof(realT));

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
            del_t[i] = -t[i]-ilamt[i]*del_lam[i];           
        }
        //Solve for step-length
        alpha = solveStepLength(lam,del_lam,t,del_t,mc);
        //Check for numerical problems (alpha will go negative)
        if(alpha < MACH_EPS) {status = -3; break;}
        //Local Scaling
        alpha *= ascale;

        //Solve for centering parameter dot(lam+alpha*del_lam,t+alpha*del_t)/mc
        mu1 = 0;
        for(i=0;i<mc;i++)
            mu1 += (lam[i]+alpha*del_lam[i])*(t[i]+alpha*del_t[i]);
        mu1 = mu1/mc/mu;
        //min((mu1/mu)^3,0.99999)
        sigma = mu1*mu1*mu1;
        if(sigma > SIGMAX)
            sigma = SIGMAX; 
        //Solve for correction term = (sigma*mu - del_lam.*del_t)./t (saved as mesil)
        musig = mu*sigma;
        for(i=0;i<mc;i++)
            mesil[i] = (musig - del_lam[i]*del_t[i])/t[i];
        //RHS = RHS - A'*term
        jtmv(ndec,mc,-1.0,A,mesil,1.0,rhs);
        memcpy(del_z,rhs,ndec*sizeof(realT));
        //Solve system with new rhs
        jtris(RQP,del_z,ndec);                     
        //-igr2+IGA*del_z + term
        for(i=0;i<mc;i++) {            
            #if ACCELQP > 0
                realT *Aptr = (realT*)&At[i*ndec];
                Azsum = 0.0; del_lam[i] = -igr2[i];
                for(j=0;j<ndec;j++) {
                    Aztemp = *Aptr++*del_z[j];
                    Azsum += Aztemp;
                    del_lam[i] += Aztemp*lamt[i];
                }
            #else
                Azsum = 0.0; del_lam[i] = -igr2[i];
                for(j=0;j<ndec;j++) {
                    Aztemp = A[i + j*mc]*del_z[j];
                    Azsum += Aztemp;
                    del_lam[i] += Aztemp*lamt[i];
                }                
            #endif
            del_lam[i] += mesil[i];
            //-A*del_z + r2 - t;
            del_t[i] = -Azsum + r2[i] - t[i];
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
		mu = jdot(mc,t,lam)/mc; //t'*lam/mc   
        //Infeasibility phi
        mr2_2 = mr2_1; mr2_1 = mr2;
        mr1 = vecmax(r1,ndec); mr2 = vecmmax(r2,t,mc);
        phi2 = phi1, phi1 = phi; phi = (max(mr1,mr2) + mu*mc)/pmax;
        //Check for NaN
        if(mu != mu || phi != phi) {status = -3; break;}
        //Check Convergence
		if(mu <= tol && phi <= tol) {
            runiter[0] = iter;
			return 1;
		}
        //Check for Primal Infeasible
        if(iter > 4 && mr2/pmax > tol/10.0) {
            if((phi > phi1 && phi1 > phi2) || (fabs(mr2-mr2_1)/mr2 < inftol && fabs(mr2_1-mr2_2)/mr2_1 < inftol)) { 
                status = -4;
                break;
            }
        }   
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