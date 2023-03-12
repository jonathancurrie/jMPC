// QP Memory Allocation Routines
// Jonathan Currie 2011

//Collect QP Memory
int getQPMemory(int mc, int ndec);
//Free QP Memory
void freeQPMemory(void);

#ifdef SINGLE_PREC
    #define PREC float
#else
    #define PREC double
#endif

//Global Variables Declared in qp_mem.c
extern PREC *qp_ptr;     //Dynamic Memory Pointer
extern PREC *lambda;     //dual vector
extern PREC *tslack;     //slack vector
extern PREC *z;          //primal vector
extern PREC *del_lam;    //lambda increment
extern PREC *del_t;      //t increment
extern PREC *del_z;      //z increment      
extern PREC *ilamt;      //t/lambda
extern PREC *lamt;       //lambda/t
extern PREC *mesil;      //mu*sigma*e
extern PREC *r1;         //dual residual
extern PREC *r2;         //primal residual
extern PREC *RQP;        //Cholesky Matrix R
 
#ifdef MEHROTRA
extern PREC *rhs;        //Corrector rhs
extern PREC *igr2;       //lambda/t*r2
#endif

#ifdef MKL
#include <mkl.h>
extern MKL_INT *ipiv;
#endif