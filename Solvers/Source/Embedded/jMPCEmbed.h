/* JMPC EMBED - Header file for Embedded jMPC Controllers
 * Copyright (C) Jonathan Currie 2012-2013 (I2C2) 
 */

//EMBED START
//-- MATHEMATIC ROUTINES --//
//Dot Product
realT jdot(intT r, const realT* vecA, const realT* vecB);
//Matrix * Vector (non-transposed)
void jmv(const intT rows, const intT cols, const realT a, const realT *matA, const realT *vecA, const realT b, realT *y);
//Matrix * Vector (transposed)
void jtmv(const intT rows, const intT cols, const realT a, const realT *matA, const realT *vecA, const realT b, realT *y);
void jtmvs(const intT rows, const intT cols, const intT *idx, const realT a, const realT *matA, const realT *vecA, const realT b, realT *y);
//Form Linear System
void formLinSys(const realT *A, const realT *lamt, const intT ndec, const intT mc, realT *RQP);
//Solve Linear System using Cholesky Decomposition
intT cholSolve(realT *R, realT *x, const intT n);
//Cholesky Decomposition
intT jchol(realT *A, const intT n);
//Forward Substitution
void jfsub(const realT *A,  realT *x, const intT n);
//Backward Substitution
void jbsub(const realT *A,  realT *x, const intT n);
//Triangular Solver
void jtris(const realT *R, realT *x, const intT n);
//Vector abs max
realT vecmax(const realT *vec, intT len);
realT vecmmax(const realT *vec, realT *t, intT len);

//-- QP ROUTINES --//
intT qpwright_embed(const realT *f, const realT *b, realT *sol, realT *lam, realT *t, intT warm, uintT *runiter);
intT qpmehrotra_embed(const realT *f, const realT *b, realT *sol, realT *lam, realT *t, intT warm, uintT *runiter);

//-- MPC ROUTINES --//
intT EmbedMPCSolve(realT *yp, realT *setp, realT *v, realT *Ou, realT *Odel_u, realT *Oym, realT *Oxm, uintT *QPiter);

//-- PIL ROUTINES --//
//PIL Simulation
void PILSim();
void PILQP();
//Measure (from serial port)
void Serial_Measure(realT *yp, realT *mdist, realT *setp);
//Output (to serial port)
void Serial_Output(intT n_in, realT *u, realT *del_u, intT nstates, realT *xm, intT n_out, realT *ym, ulongT T, uintT iter, intT status);
//Transmit array of floating point numbers
void tx_data(intT no, realT *data);
//Transmit a 16bit integer
void tx_int(uintT t);
//Transmit a 32bit integer
void tx_long(ulongT t);
//Transmit string
void tx_msg(charT * msg);
//Receive array of floating point numbers
intT rx_data(realT *data);
//Transmit Byte (User to Implement)
void serial_tx(ucharT a);
//Receive Byte (User to Implement)
ucharT serial_rx(void);
//Start uS Timer (User to Implement)
void timer_start();
//Stop uS Timer (User to Implement)
ulongT timer_stop();

//-- USEFUL MACROS --//
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

