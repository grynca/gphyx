#include "base.h"
using namespace grynca;
#include "test_dynamics.h"
#include "test_collisions.h"

int main(int argc, char* argv[]) {
    srand(time(NULL));

    SDLTestBenchSton::create(1024, 768, true);
    SDLTestBench& testbench = SDLTestBenchSton::get();

    TestCollisions tcoll;
    testbench.addTest("Test Collisions", &tcoll);

    TestDynamics tdyn;
    testbench.addTest("Test Dynamics", &tdyn);

    testbench.runTest(0);
    return 0;
}