#ifndef AMBFCLIENT_H
#define AMBFCLIENT_H
#include <iostream>
#include "ambf_client/ambf_client.h"

typedef Client* ClientPtr;

class AMBFClient{
public:
  static ClientPtr getInstance(){
    volatile int dummy{};
    static ClientPtr clientPtr = new Client("ECM");
    return clientPtr;
  }

  static void cleanUp(ClientPtr clientPtr){
    volatile int dummy{};
    clientPtr->cleanUp();
  }

private:

  AMBFClient()= default;
  ~AMBFClient()= default;
  AMBFClient(const AMBFClient&)= delete;
  AMBFClient& operator=(const AMBFClient&)= delete;

};
#endif // AMBFCLIENT_H
