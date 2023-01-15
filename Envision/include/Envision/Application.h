#ifndef _APPLICATION_
#define _APPLICATION_

namespace Envision {
  class Application
  {
  public:
    Application();
    virtual ~Application();

    void Run();
  };
  
  // To be defined in CLIENT
  // Application* CreateApplication();
} // namespace Sway

#endif