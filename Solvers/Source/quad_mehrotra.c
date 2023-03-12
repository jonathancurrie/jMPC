//***** Solve QP Using Mehrotra's Method *****//

// Copyright (C) 2009-2013 Jonathan Currie (www.i2c2.aut.ac.nz)

#include <qp_header.h>
#include <qp_mem.h>
#include <mex.h>
#include <jmath.h>
#include <math.h>
#ifdef SINGLE_PREC
    #define sqrt(x) sqrtf(x)
    #define fabs(x) fabsf(x)
#endif

//EMBED START
int quad_mehrotra(const realT *H, const realT *f, const realT *A, 
        const realT *b, int mc, int ndec, realT *sol, realT *lam, realT *t,
        int warm, realT tol, int maxiter, int verbose, int *runiter)
{      
    //Internal Variables
    int i,j, iter, status = 1, cholfail=0;
    //Calculation Variables
    realT Aztemp, Azsum, alpha = (realT)1.0, alpham1, ascale = (realT)1.0, inftol = (realT)tol*10;
    realT sigma = (realT)0.1, phi = (realT)1e30, phi1 = (realT)1e30, phi2, mu, mu1, mu_old, musig, WARMVAL;
    realT mr2_2, mr2_1 = (realT)100.0, mr2 = (realT)10.0, mr1, pmax = (realT)-1e20;
    
    //Determine max problem data value
    pmax = vecmax(H,ndec*ndec);
    pmax = max(vecmax(f,ndec),pmax);
    pmax = max(vecmax(A,mc*ndec),pmax);
    pmax = max(vecmax(b,mc),pmax);
    //Set Heuristic Warm Start Value
    WARMVAL = pmax > (realT)1.0 ? min(sqrt(pmax),1e6) : (realT)0.5; 
    
    //Initialize Variables Based on Warm Start Mode
    switch(warm)
    {
        case 0: //none
            for(i=0;i<ndec;i++)
                z[i] = (realT)0.0;
            for(i=0;i<mc;i++) {
                lam[i] = WARMVAL;
                t[i] = WARMVAL;
            }            
            break;
        case 1: //primal only
            for(i=0;i<ndec;i++)
                z[i] = sol[i];
            for(i=0;i<mc;i++) {
                lam[i] = WARMVAL;
                t[i] = WARMVAL;
            }
            break;
        case 2: //primal and dual
            for(i=0;i<ndec;i++)
                z[i] = sol[i];
            for(i=0;i<mc;i++)
                t[i] = WARMVAL;
            break;
        case 3: //primal, dual and slack
            for(i=0;i<ndec;i++)
                z[i] = sol[i];
            break;
    }
    //Calculate Initial Residuals
    memcpy(r1,f,ndec*sizeof(realT));
    memcpy(r2,b,mc*sizeof(realT));
    //r1 = -H*z-f-A'*lam
    jmv(ndec,ndec,-1.0,H,z,-1.0,r1);
    jtmv(ndec,mc,-1.0,A,lam,1.0,r1);
    //r2 = -A*z + b
    jmv(mc,ndec,-1.0,A,z,1.0,r2);
    
    //Optional Extra Info
    if(verbose) {
        mexPrintf("-----------------------------------------------\n");
        if(sizeof(realT)==4)
            mexPrintf("QuadMehrotra QP Solver [MEX Single Version]\n");
        else
            mexPrintf("QuadMehrotra QP Solver [MEX Double Version]\n");
        switch(warm) {
            case 3: mexPrintf(" Warm Start: Primal + Dual + Slack\n"); break;
            case 2: mexPrintf(" Warm Start: Primal + Dual\n"); break;
            case 1: mexPrintf(" Warm Start: Primal\n"); break;
        }
        if(verbose > 1)
            mexPrintf(" %4d Decision Variables\n %4d Inequality Constraints\n",ndec,mc);         
        mexPrintf("-----------------------------------------------\n");
        mexPrintf("iter           phi             mu        sigma        alpha       max(r1)       max(r2)\n");
        mexEvalString("drawnow;");
    }
    
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
        formLinSys(A,lamt,ndec,mc,RQP);
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
            if(verbose)
                mexPrintf("\b (Cholesky Failed)\n"); //may need corrective action here
            cholfail++;             
            //Check for max cholesky failures
            if(cholfail > CHOLMAX) {
                status = -2;//indicate failure
                break; //give up
            }
            //Reduce maximum step size
            ascale -= ADEC;
        }
		//-IGA*del_z+IGR2
        for(i=0;i<mc;i++) {
            for(j=0;j<ndec;j++)                           
                del_lam[i] += A[i + j*mc]*lamt[i]*del_z[j];
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
            Azsum = 0.0; del_lam[i] = -igr2[i];
            for(j=0;j<ndec;j++) {
                Aztemp = A[i + j*mc]*del_z[j];
                Azsum += Aztemp;
                del_lam[i] += Aztemp*lamt[i];
            }
            del_lam[i] += mesil[i];
            //-A*del_z + r2 - t;
            del_t[i] = -Azsum + r2[i] - t[i];
        }
        //Solve for step-length
        alpha = solveStepLength(lam,del_lam,t,del_t,mc);
        //Check for numerical problems (alpha will go negative)
        if(alpha < MACH_EPS) {status = -3; break;}   
        //Local Scaling
        alpha *= ascale; alpham1 = (realT)1.0 - alpha;        
        //Increment Iterates
        for(i=0;i<ndec;i++) {
            z[i] += alpha*del_z[i];
            r1[i] *= alpham1; //equiv to -H*z-f-A'*lam
        }
        for(i=0;i<mc;i++) {
            lam[i] += alpha*del_lam[i];
            t[i] += alpha*del_t[i];
        }
        //Update Residuals
        memcpy(r2,b,mc*sizeof(realT));
        jmv(mc,ndec,-1.0,A,z,1.0,r2);
        //Complementarity Gap mu
        mu_old = mu;
		mu = jdot(mc,t,lam)/mc; //t'*lam/mc
        //Infeasibility phi
        mr2_2 = mr2_1; mr2_1 = mr2;
        mr1 = vecmax(r1,ndec); mr2 = vecmmax(r2,t,mc);
        phi2 = phi1, phi1 = phi; phi = (max(mr1,mr2) + mu*mc)/pmax;
        if(verbose) {
            mexPrintf("%3d  %13.5g  %13.5g  %11.5g  %11.5g   %11.5g   %11.5g\n",iter,phi,mu,sigma,alpha,mr1,mr2); 
            mexEvalString("drawnow;");
        }        
        //Check for NaN
        if(mu != mu || phi != phi) {status = -3; break;}
        //Check Convergence
		if(mu <= tol && phi <= tol) {
            memcpy(sol,z,ndec*sizeof(realT));
            runiter[0] = iter;
            //Optional Output
            if(verbose) {
                mexPrintf("-----------------------------------------------\n");
                mexPrintf(" Successfully solved in %d Iterations\n",iter);
                if(verbose > 1)
                    mexPrintf(" Final phi: %g, mu %g [tol %g]\n",phi,mu,tol);
                mexPrintf("-----------------------------------------------\n");
            }
			return 1;
		}
        //Check for Primal Infeasible
        if(iter > 4 && mr2/pmax > tol/10.0) {
            if((phi > phi1 && phi1 > phi2) || (fabs(mr2-mr2_1)/mr2 < inftol && fabs(mr2_1-mr2_2)/mr2_1 < inftol)) {
                if(verbose)
                    mexPrintf("\b (Primal Infeasibility Detected)\n");  
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
    //Optional Output
    if(verbose) {
        mexPrintf("-----------------------------------------------\n");
        switch(status) 
        {
            case -1: mexPrintf(" Maximum Iterations Exceeded\n"); break;
            case -2: mexPrintf(" Cholesky Failed - Further Progress Is Unlikely\n"); break;
            case -3: mexPrintf(" Failed - Numerical Errors Detected\n"); break;
            case -4: mexPrintf(" Failed - Problem Looks Infeasible\n"); break;
        }
        if(verbose > 1)
            mexPrintf(" Final phi: %g, mu %g [tol %g]\n",phi,mu,tol);
        mexPrintf("-----------------------------------------------\n");
    }  
    //Copy best solution
    memcpy(sol,z,ndec*sizeof(realT));
    //Return Error
    runiter[0] = iter;
    return status;   
}