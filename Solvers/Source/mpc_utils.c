//MPC Utility Functions
#include "mpc_utils.h"

//Check and return structure field
mxArray *getStructArg(const mxArray *prhs, const char *structname, const char *fieldname)
{
    char errmsg[256];
    int index;
    
    if(!mxIsStruct(prhs)) {
        sprintf(errmsg,"jMPC %s Input Error: The specified structure '%s' is not a MATLAB structure!\n",BUILD_TYPE,structname);
        mexErrMsgTxt(errmsg);
        return NULL;
    }    
    index = mxGetFieldNumber(prhs,fieldname);    
    if(index < 0) {
        sprintf(errmsg,"jMPC %s Input Error: Cannot find field '%s' in passed structure '%s'!\n",BUILD_TYPE,fieldname,structname);
        mexErrMsgTxt(errmsg);
        return NULL;
    }   
    return mxGetFieldByNumber(prhs,0,index);
}

//Check and return structure field (2 deep)
mxArray *getStructArg2(const mxArray *prhs, const char *structname1, const char *structname2, const char *fieldname)
{
    return getStructArg(getStructArg(prhs,structname1,structname2),structname2,fieldname);
}

//Check and return class property
mxArray *getClassProp(const mxArray *prhs, const char *classname, const char *propname)
{
    char errmsg[256];
    mxArray *data;
    
    if(!mxIsClass(prhs,classname)) {
        sprintf(errmsg,"jMPC %s Input Error: The specified class object is not a '%s' object!\n",BUILD_TYPE,classname);
        mexErrMsgTxt(errmsg);
        return NULL;
    }    
    data = mxGetProperty(prhs,0,propname);
    if(data == NULL) {
        sprintf(errmsg,"jMPC %s Input Error: Cannot find property '%s' in passed '%s' object!",BUILD_TYPE,propname,classname);
        mexErrMsgTxt(errmsg);
        return NULL;
    }    
    return data;
}