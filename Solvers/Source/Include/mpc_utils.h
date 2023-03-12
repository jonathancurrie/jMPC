//MPC Utility Functions
#include <mex.h>

#ifdef MEX
    #define BUILD_TYPE "Engine"
#elif defined SFUN
    #define BUILD_TYPE "S Function"
#endif

//Check and return structure field
mxArray *getStructArg(const mxArray *prhs, const char *structname, const char *fieldname);
//Check and return structure field (2 deep)
mxArray *getStructArg2(const mxArray *prhs, const char *structname1, const char *structname2, const char *fieldname);
//Check and return class property
mxArray *getClassProp(const mxArray *prhs, const char *classname, const char *propname);