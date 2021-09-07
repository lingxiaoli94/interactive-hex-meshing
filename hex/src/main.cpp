#include "HexMeshingApp.h"

using namespace hex;

int main() {
  HexMeshingApp app{true};
  app.Prepare();
  app.MainLoop();
}
