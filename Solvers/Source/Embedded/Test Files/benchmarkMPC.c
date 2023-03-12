/* BENCHMARK MPC - A C File For Benchmarking MPC Performance
 * Copyright (C) Jonathan Currie 2014 (I2C2) 
 */

//Note extra variables are automatically inserted here by embed()

//EMBED START
//External Constants
extern const uintT n_in;
extern const uintT nm_in;
extern const uintT n_out;
extern const uintT nm_out;
extern const uintT nm_dist;
extern const uintT nq_out;
extern const uintT states;
extern const uintT Np_qout;
extern const uintT Nb_in;
extern const uintT ndec;
extern const uintT mc;

//Main Benchmark Function
int main()
{
    //Internal Args
    unsigned int i, k = 0, veclen = T+1;
	drealT tav = 0, tmx = 0, tto = 0;
	uintT qpt = 0, qpm = 0, qpf = 0;
	//Initialize ms Timer
	#ifdef _INC_WINDOWS
		LARGE_INTEGER tic, toc, frequency;
		if(!QueryPerformanceFrequency(&frequency)) {
			printf("Failed to retrieve system frequency for timing");
			return 0;
		}
	#else
		struct timespec tic, toc, temp;
	#endif
	
	//Print Run Specs
	printf("*------------------------------------------*\n\n");
	printf(" jMPC Benchmark (jMPC v%s, Embed v%s)\n",JMPCVER,EMBEDVER);
	printf("  - Solver: %s [%s Precision]\n",QPSOLVER,PRECISION);
	printf("  - Np: %d, Nb: %d, ndec: %d, ncon: %d\n",Np_qout/nq_out,Nb_in,ndec,mc);
	printf("  - In: %d, Out: %d, States: %d, Mdist: %d\n",nm_in,nq_out,states,nm_dist);
	
	printf("\n Running...");
    
    //Copy Initial u
    for(i=0;i<n_in;i++)
        pu[veclen*i] = init_u[i];
    //Copy Initial xp,yp
    for(i=0;i<states;i++)
        pxp[veclen*i] = xp[i];
    for(i=0;i<nm_out;i++)
        pyp[veclen*i] = yp[i];
    
    //Run MPC Simulation    
    for(k = 0; k < T; k++)
    {
        //Allocate Setpoint & Measured Disturbances
        for(i = 0; i < nq_out; i++)
            setpk[i] = setp[k + veclen*i];
        for(i = 0; i < max(nm_dist,1); i++)
            mdistk[i] = mdist[k + veclen*i];
			
		//Start Timer
		#ifdef _INC_WINDOWS
			QueryPerformanceFrequency(&frequency);
			QueryPerformanceCounter(&tic);
		#else
			clock_gettime(CLOCK_MONOTONIC,&tic);
		#endif
        
        //----- RUN MPC ENGINE -----//
        qpstat[k] = EmbedMPCSolve(yp, setpk, mdistk, mpc_u, mpc_del_u, mpc_ym, mpc_xm, &qpiter[k]);  
        
		//Stop Timer & Record Time
		#ifdef _INC_WINDOWS
			QueryPerformanceCounter(&toc);
			telaps[k] = (double)(toc.QuadPart-tic.QuadPart)*1e6/(double)frequency.QuadPart;
		#else
			clock_gettime(CLOCK_MONOTONIC,&toc);
			//From Forces Code
			if ((toc.tv_nsec - tic.tv_nsec)<0) {
				temp.tv_sec = toc.tv_sec - tic.tv_sec-1;
				temp.tv_nsec = 1000000000+toc.tv_nsec - tic.tv_nsec;
			} else {
				temp.tv_sec = toc.tv_sec - tic.tv_sec;
				temp.tv_nsec = toc.tv_nsec - tic.tv_nsec;
			}
			telaps[k] = (double)temp.tv_sec + (double)temp.tv_nsec / 1000;
		#endif
		
        //----- Process & Save Model Inputs -----//
        for(i = 0; i < n_in; i++) {
			pu[k + 1 + veclen*i] = mpc_u[i]; 	   //Saved Input		
        	upp[i] = mpc_u[i] + umdist[k + veclen * i]; //Plant Input
        }              

        //----- Simulate Plant -----//
        jmv(states,states,1.0,plantA,xp,0.0,Kr);
        jmv(states,n_in,1.0,plantB,upp,1.0,Kr);
        memcpy(xp,Kr,states*sizeof(double));
        jmv(nm_out,states,1.0,plantC,xp,0.0,yp);
        //Process & Save Plant States & Outputs
        for(i = 0; i < nm_out; i++)
        {
            yp[i] += ydist[k + veclen * i];
            pyp[k + 1 + veclen * i] = yp[i];
        }
        for(i = 0; i < states; i++)
            pxp[k + 1 + veclen * i] = xp[i];
    }

	//Calculate Stats
	for(i = 0; i < T; i++){
		tto += telaps[i];
		if(telaps[i] > tmx)
			tmx = telaps[i];
		qpt += qpiter[i];
		if(qpiter[i] > qpm)
			qpm = qpiter[i];
		if(qpstat[i] < 0)
			qpf++;			
	}
	tav = tto/T;
	
	//Print Summary
	printf("Done!\n\n Sampling Statistics: \n");
	printf("  - Average: %5.3f us (%1.3f kHz)\n",tav,1./(tav/1000.));
	printf("  - Maximum: %5.3f us (%1.3f kHz)\n",tmx,1./(tmx/1000.));
	printf("  - Total:   %5.3f ms\n",tto/1000.0);
	printf(" Solver Statistics:\n");
	printf("  - Total QP Iters: %d (Max %d)\n",qpt,qpm);
	printf("  - Total QP Failures: %d \n",qpf);
	printf("\n*------------------------------------------*\n\n");
}