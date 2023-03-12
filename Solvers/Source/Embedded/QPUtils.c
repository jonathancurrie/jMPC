//EMBED START
//Solve Step Length
realT solveStepLength(realT *lam, realT *del_lam, realT *t, realT *del_t, intT mc)
{
    realT minval = 1.0;
    intT i, KKTOK = 1;
    for(i = 0; i < mc; i++) {
        realT alam = lam[i]+del_lam[i];
        realT at = t[i]+del_t[i];
        if(alam < MACH_EPS) {            
            realT lam_temp = 1.0-(alam/del_lam[i]);
            minval = lam_temp < minval ? lam_temp : minval; 
            KKTOK = 0;
        }
        if(at < MACH_EPS) {            
            realT t_temp = 1.0-(at/del_t[i]); 
            minval = t_temp < minval ? t_temp : minval;
            KKTOK = 0;
        }	
    }
    if(KKTOK)
        return AOKMAX;
    else
        return AMAX*minval;
}