/* SERIAL EMBED - Serial Port Routine for Processor In the Loop (PIL) Simulation
 * Copyright (C) Jonathan Currie 2012 (Control Engineering) 
 */

//EMBED START
//External Constants
extern const intT nm_in, n_out, states, ndec, mc;

//Processor In The Loop MPC Simulation
void PILSim()
{
    intT status = 0;
    uintT iter = 0;
    ulongT Te = 0;
    
    //Enter while 1 loop for MPC control
    while(1) {
		//Get Inputs
		rx_data(yp); rx_data(mdist); rx_data(setp);
        //Start uS Timer
        timer_start();
		//MPC Calculation
		status = EmbedMPCSolve(yp, setp, mdist, mpc_u, mpc_del_u, mpc_ym, mpc_xm, &iter);
        //Stop uS Timer
		Te = timer_stop();
		//Send Outputs
        tx_data(nm_in,mpc_u); tx_data(nm_in,mpc_del_u); tx_data(states,mpc_xm); tx_data(n_out,mpc_ym);
        tx_long(Te); tx_int(iter); tx_int(status);
	}    
}

#ifdef CON_MPC
//Processor In The Loop QP Solver
void PILQP()
{
    intT status = 0;
    uintT iter = 0, warm = 0;
    ulongT Te = 0;
    
    //Enter while 1 loop for multiple QPs
    while(1) {   
        //Get Variables + Mode
        warm = (uintT)serial_rx();
        rx_data(QPf); rx_data(QPb); rx_data(_z); rx_data(_lambda); rx_data(_tslack);   
        //Start uS Timer
        timer_start();
        //Call QP Solver
        status = qpwright_embed(QPf,QPb,_z,_lambda,_tslack,warm,&iter);
        //Stop uS Timer
        Te = timer_stop();
        //Transmit Results
        tx_data(ndec,_z); tx_data(mc,_lambda); tx_data(mc,_tslack);
        tx_long(Te); tx_int(iter); tx_int(status);
    }
}
#endif
#endif 

//Transmit array of floating point numbers
void tx_data(intT no, realT *data)
{
	intT i;
    #ifdef DOUBLE_PREC
    ulongT *ptr = (ulongT *)data;
    #else
	uintT *ptr = (uintT *)data;
    #endif

	for(i = 0; i < no; i++)
	{		
		#ifdef DOUBLE_PREC
		serial_tx(ptr[i*2]);
		serial_tx(ptr[i*2]>>8);
		serial_tx(ptr[i*2]>>16);
		serial_tx(ptr[i*2]>>24);
		serial_tx(ptr[i*2+1]);
		serial_tx(ptr[i*2+1]>>8);
		serial_tx(ptr[i*2+1]>>16);
		serial_tx(ptr[i*2+1]>>24);
        #else
        serial_tx(ptr[i*2]);
		serial_tx(ptr[i*2]>>8);
		serial_tx(ptr[i*2+1]);
		serial_tx(ptr[i*2+1]>>8);
		#endif
	}
}

//Transmit a 16 bit integer
void tx_int(uintT t)
{
    serial_tx(t);
    serial_tx(t>>8);
}

//Transmit a 32 bit integer
void tx_long(ulongT t)
{
    serial_tx(t);
    serial_tx(t>>8);
    serial_tx(t>>16);
    serial_tx(t>>24);
}

//Transmit string
void tx_msg(charT * msg)
{
    intT i = 0;
    while(msg[i] != '\0')
		serial_tx((ucharT)msg[i++]);
}

//Receive array of floating point numbers
intT rx_data(realT *data)
{
	ucharT rxchar;
	uintT rxstatus = 0, done = 0, max = 1000, rxcount = 0, bytecount = 0;
	#ifdef DOUBLE_PREC
	ullongT fdata[8];
	uintT bytecomp = 7;
	#else
	ulongT fdata[4];
	uintT bytecomp = 3;
	#endif
	enum{rx_idle,no_rx,data_rx};
	intT no;

	while(done < max)
	{
		//Collect byte from Serial Port
		rxchar = serial_rx();
		//Switch based on state
		switch(rxstatus)
		{
			case rx_idle:
			{
				if(rxchar == 0xCA) //expected header
				{
					rxstatus = no_rx;
					rxcount = 0;
				}
				break;
			}
			case no_rx:
			{
				no = (intT)rxchar;
				rxstatus = data_rx;
				break;
			}
			case data_rx:
			{
				//Need to build up floats/doubles
				if(bytecount == bytecomp)
				{					
					//Shift in
					#ifdef DOUBLE_PREC
					fdata[bytecount] = (ullongT)rxchar;
					fdata[0] |= fdata[7] << 56 | fdata[6] << 48 | fdata[5] << 40 | fdata[4] << 32 | fdata[3] << 24 | fdata[2] << 16 | fdata[1] << 8;				
					#else
					fdata[bytecount] = (ulongT)rxchar;
					fdata[0] |= fdata[3] << 24 | fdata[2] << 16 | fdata[1] << 8;
					#endif
					memcpy(&data[rxcount++], fdata, sizeof(realT));
					bytecount=0;
				}
				else {
					#ifdef DOUBLE_PREC
					fdata[bytecount++] = (ullongT)rxchar;
					#else
					fdata[bytecount++] = (ulongT)rxchar;
					#endif
				}
				
				if(rxcount == no)
					return rxcount;

				break;
			}

		}
		//Increment loop counter
		done++;
	}
	return rxcount;
}