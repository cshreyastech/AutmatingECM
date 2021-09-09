//#include "ecm_manupulation/Dynamics.h"
//#include "ecm_manupulation/Kinematics.h"
#include "ecm_manupulation/AMBFClient.h"

int main(int argc, char* argv[])
{
    //Kinematics kinematics;
//    kinematics.AMBFIK();
    AMBFClient ambfclient = new AMBFClient();
    ClientPtr client = ambfclient.getInstance();
    return 0;
}
