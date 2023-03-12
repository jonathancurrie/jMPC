// Quadratic Programming Solvers Header File
// Jonathan Currie 09-13

#ifdef SINGLE_PREC
    typedef float realT;
    #define AMAX 0.9995F       
    #define AOKMAX 0.999995F 
    #define ADEC 0.1F
    #define SIGMAX 0.99999F     
    #define CHOLMAX 2             
    #define MACH_EPS 0.0000001192F
#else
    typedef double realT;
    #define AMAX 0.9995         //step-size factor for >= lam, t
    #define AOKMAX 0.999995     //step-size for no breaking lam,t
    #define ADEC 0.1            //decrement for Cholesky Failure step size
    #define SIGMAX 0.99999      //maximum value of sigma
    #define CHOLMAX 2           //maximum number of cholesky failures
    #define MACH_EPS 2.220446049250313e-16
#endif

//Quad Wright
int quad_wright(const realT *H, const realT *f, const realT *A, 
        const realT *b, int mc, int ndec, realT *sol, realT *lam, realT *t,
        int warm, realT tol, int maxiter, int verbose, int *runiter);
//Quad Mehrotra
int quad_mehrotra(const realT *H, const realT *f, const realT *A, 
        const realT *b, int mc, int ndec, realT *sol, realT *lam, realT *t,
        int warm, realT tol, int maxiter, int verbose, int *runiter);
//Solve Step-Length
realT solveStepLength(realT *lam, realT *del_lam, realT *t, realT *del_t, int mc);

