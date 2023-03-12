// QP Memory Allocation Routines
// Jonathan Currie 2011-2013

#include <qp_mem.h>
#include <stdlib.h>

//Global Variables
PREC *qp_ptr;     
PREC *lambda;     
PREC *tslack;          
PREC *z;          
PREC *del_lam;    
PREC *del_t;      
PREC *del_z;       
PREC *ilamt;      
PREC *lamt;      
PREC *mesil;
PREC *r1;         
PREC *r2;         
PREC *RQP;        
#ifdef MEHROTRA
    PREC *rhs;
    PREC *igr2;
#endif
#ifdef MKL
    MKL_INT *ipiv;
#endif

int getQPMemory(int mc, int ndec)
{
    int len, indx, i;
    
    //Collect Memory for QP Variables
    len = 8*mc+3*ndec+ndec*ndec;
    #ifdef MEHROTRA
        len += ndec + mc;
    #endif
    #ifdef MKL
        qp_ptr = (PREC*)mkl_malloc(len*sizeof(PREC),16);
    #else
        qp_ptr = (PREC*)_aligned_malloc(len*sizeof(PREC),16);
    #endif
    if(qp_ptr == NULL)
        return 0;
    
    //Initialise all values to something known (problems with NaNs propogating!)
    for(i = 0; i < len; i++)
        qp_ptr[i] = 0.0;
    
    //Allocate QP Memory
    indx = 0;
    lambda          = &qp_ptr[indx]; indx += mc;
    tslack          = &qp_ptr[indx]; indx += mc;
    z               = &qp_ptr[indx]; indx += ndec;
    del_lam         = &qp_ptr[indx]; indx += mc;
	del_t           = &qp_ptr[indx]; indx += mc;    
    del_z           = &qp_ptr[indx]; indx += ndec;
    ilamt           = &qp_ptr[indx]; indx += mc;
    lamt            = &qp_ptr[indx]; indx += mc;
    mesil           = &qp_ptr[indx]; indx += mc;
	r1              = &qp_ptr[indx]; indx += ndec;
	r2              = &qp_ptr[indx]; indx += mc;
	RQP             = &qp_ptr[indx]; indx += (ndec*ndec);
    #ifdef MEHROTRA
        rhs         = &qp_ptr[indx]; indx += ndec;
        igr2        = &qp_ptr[indx]; indx += mc;
    #endif
        
    //Collect MKL Specific Memory
    #ifdef MKL
        ipiv = (MKL_INT*)mkl_malloc(ndec*sizeof(MKL_INT),16);
        if(ipiv == NULL)
            return 0;
    #endif
    
    //Successful
    return 1;
}

void freeQPMemory(void)
{             
    #ifdef MKL
        mkl_free(qp_ptr); 
        mkl_free(ipiv);
    #else
        _aligned_free(qp_ptr); 
    #endif
}