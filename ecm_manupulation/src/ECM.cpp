#include "ecm_manupulation/ECM.h"

ECM::ECM(){

}


void ECM::cleanup() {
    for(DH *dh : DH_Vector)
        dh->~DH();
}

ECM::~ECM(void){
    cleanup();
}

