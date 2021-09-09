//#include "ecm_manupulation/Dynamics.h"
//#include "ecm_manupulation/Kinematics.h"
#include "ecm_manupulation/AMBFClient.h"

int main(int argc, char* argv[])
{
    //Kinematics kinematics;
//    kinematics.AMBFIK();

    AMBFClient *client;
    ClientPtr ambfClient = client->getInstance();
    ambfClient->connect();
    ambfClient->cleanUp();
    return 0;
}
