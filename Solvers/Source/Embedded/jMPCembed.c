/* MPC EMBED - jMPC Engine for Embedded MPC
 * Copyright (C) Jonathan Currie 2012-2013 (I2C2) 
 */

//EMBED START
//External Constants
extern const intT ndec, mc;
extern const intT n_in, Nb_in, nm_in, n_out, nm_out, nm_dist, nq_out, Np_out, Np_qout;
extern const intT states, statesn, statesnq, statesnm;
extern const intT ndelu, numin, numax, nymin, nymax;
extern const uintT iumeasp_out[], ismeasp_out[], isq_out[], imeas_dist[], iman_u[];
extern const uintT idelu[], iumin[], iumax[], iymin[], iymax[];
extern const realT Kest[], IKC[];
extern const realT A[], R[], F[], SE[], Tin[], PhiTQ[], Hscale[], Ascale[];
extern const realT delumax[], bcon[], umin[], umax[], u_op[], y_op[];
extern const intT uncon, soft, warm;
extern const realT modelA[], modelB[];
#ifdef MEAS_DIST
extern const realT Phiv[];
extern const realT Phiqv[];
#endif 
#ifdef UNCON_OUT
extern const realT Fq[];
#endif

//-- Main MPC Engine --//
intT EmbedMPCSolve(realT *yp, realT *setp, realT *v, realT *Ou, realT *Odel_u, realT *Oym, realT *Oxm, uintT *QPiter)
{ 
    //Internal Variables
    intT i, index, QPstatus = 0;
    #ifdef CON_MPC
        intT j, global_fail, offset=0;
        realT atemp;
    #endif
    //Initialize
    *QPiter=0;
    
    //---- State Estimator Update ----//
	//K(ismeasp_out) = J.state_est.Kest*(yp-y_op);
	for(i = 0; i < nm_out; i++)
		ys[i] = yp[i]-y_op[i]; 	
    jmv(statesnm,nm_out,1.0,Kest,ys,0.0,Kys);
	for(i = 0; i < (states+nm_out);i++)
		K[ismeasp_out[i]] = Kys[i];
    #ifdef UNMEAS_OUT
        //K(iumeasp_out) = del_xm(iumeasp_out); 
        for(i = 0; i < (n_out-nm_out); i++)
            K[iumeasp_out[i]] = del_xm[iumeasp_out[i]];		
    #endif
	//del_xm = J.state_est.IKC*del_xm + K        
    jmv(statesn,statesn,1.0,IKC,del_xm,1.0,K);
    memcpy(del_xm,K,statesn*sizeof(realT));
				
    //---- Update RHS ----//
	//y0 = J.F*del_xm + Phiv * mdist;
    #ifdef UNCON_OUT
       //Use only controlled outputs for QP prediction = del_xm(isq_out)
        for(i = 0; i < statesnq; i++)
            qp_del_xm[i] = del_xm[isq_out[i]];
       //y0 = J.F*del_xm(iout) + Phiqv*mdist';
        jmv(Np_qout,statesnq,1.0,Fq,qp_del_xm,0.0,y0);
    #else
        jmv(Np_out,statesn,1.0,F,del_xm,0.0,y0);
    #endif       
    #ifdef MEAS_DIST	
        //Disturbance Prediction = Phiqv * mdist;        
        for(i = 0; i < nm_dist; i++) {  //Difference measured disturbance         
            del_v[i] = v[i] - old_v[i];
            old_v[i] = v[i];
        }       
        jmv(Np_qout,nm_dist,1.0,Phiqv,del_v,1.0,y0); 
    #endif		         
    memcpy(Ssetp,y0,Np_qout*sizeof(realT));
    //setp = J.QP.S*setp(k,:)'-y0;
    jmv(Np_qout,nq_out,1.0,SE,setp,-1.0,Ssetp);
    //f = J.QP.PhiTQ*Ssetp;
    jmv(Nb_in,Np_qout,-1.0,PhiTQ,Ssetp,0.0,fvec);
    //Scale f
    for(i=0;i<ndec;i++)
        fvec[i] *= Hscale[i];
    //Constrained Problem
    #ifdef CON_MPC    
        //Initialize
        for(i=0;i<mc;i++)
            bvec[i] = 0.0;
        offset = 0;
        #ifdef DELU_CON //delu constraints exist
            offset = ndelu * 2; //offset past delu constraints
        #endif
        #ifdef U_CON //u constraints exist
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
        #endif
        #ifdef Y_CON //y constraints exist
            //If no uncontrolled outputs we already have prediction in y0, otherwise required
            #ifdef UNCON_OUT
                //Full prediction for constraints -> y0 = J.F*del_xm + Phiv * mdist;
                jmv(Np_out,statesn,1.0,F,del_xm,0.0,y0);            
                #ifdef MEAS_DIST                            
                    //Disturbance Prediction = Phiv * mdist; 
                    jmv(Np_out,nm_dist,1.0,Phiv,del_v,1.0,y0);
                #endif	
            #endif
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
        #endif
        //Add constant & dynamic b while scaling      
        for(i = 0; i < mc; i++)
            bvec[i] = (bvec[i] + bcon[i])*Ascale[i];
        
        //---- Check Unconstrained Optimum ----//
        //x = -J.QP.R\(J.QP.R'\f); % -H\f; 
        memcpy(x,fvec,ndec*sizeof(realT));
        jtris(R,x,ndec); 
        //Check for any Ax > b
        global_fail = 0;
        for(i=0;i<mc;i++) {
            atemp = 0.0;
            for(j=0;j<ndec;j++) //column dot product
                atemp += -A[i + j*mc]*x[j];            
            if(atemp > bvec[i]) {
                global_fail = 1;
                break;
            }
        }
        //---- Solve Constrained Optimum ----//
        if(global_fail) {            
            //Solve QP using selected solver
            QPstatus = qpwright_embed(fvec, bvec, del_u, lambda, tslack, local_warm, QPiter);            
            if(warm) {
                //Add a little to lambda to allow solver to find new constraints easier
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
                del_u[i] = -x[i];
            //Global solution is used, so disable dual+slack warm starting for now
            if(warm)
                local_warm = 1;   
        }
		
        //---- Saturate Inputs ----//
        #ifdef DELU_CON
            for(i = 0, j = 0; i < Nb_in; i++) {        
                if(del_u[i] > delumax[j]) //saturate delu at constraints
                    del_u[i] = delumax[j];
                else if(del_u[i] < -delumax[j])
                    del_u[i] = -delumax[j];                
                if(++j >= nm_in)
                    j = 0;
            }
        #endif        
        for(i = 0; i < nm_in; i++)  {
            u[i] += del_u[i]; 
            if(numin || numax) {
                if(u[i] < umin[i]) //saturate u at constraints
                    u[i] = umin[i];
                else if(u[i] > umax[i])
                    u[i] = umax[i];            
            }
            Odel_u[i] = del_u[i];
        }
        
    //Unconstrained Problem
	#else
        //x = J.QP.R\(J.QP.R'\f)
        memcpy(x,fvec,ndec*sizeof(realT));
        jtris(R,x,ndec); 
        for(i = 0; i < Nb_in; i++)
            del_u[i] = -x[i]; //Global solution is -x
        //Sum del_u to get u
        for(i = 0; i < nm_in; i++) {
            u[i] += del_u[i];
            Odel_u[i] = del_u[i];
        }
    #endif
	
	//---- Add Measured Disturbance ----//
    #ifdef MEAS_DIST
		for(i = 0; i < nm_in; i++) {
			um[iman_u[i]] = del_u[i];      //place del_u in model input
			up[iman_u[i]] = u[i];          //place u in plant input
		}        
		for(i = 0; i < nm_dist; i++)       
			um[imeas_dist[i]] = del_v[i];  //place del_v in model input	
	#else
		for(i = 0; i < n_in; i++) {
			um[i] = del_u[i];
			up[i] = u[i];
		}
	#endif
	
    //---- Simulate Model ----//
    jmv(statesn,statesn,1.0,modelA,del_xm,0.0,K);
    jmv(statesn,n_in,1.0,modelB,um,1.0,K);
    memcpy(del_xm,K,statesn*sizeof(realT));
	
	//Process Inputs
	for(i = 0; i < nm_in; i++)
		Ou[i] = up[i]+u_op[iman_u[i]]; 
    //---- Process Outputs ----//
	for(i = 0; i < states; i++) {
		xm[i] += del_xm[i];
		Oxm[i] = xm[i];
    }
    for(i = states, index = 0; i < statesn; i++, index++)
        Oym[index] = del_xm[i];
    
    //Return Status (global or qp)
    return QPstatus;
}
