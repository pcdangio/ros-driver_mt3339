#include "driver.h"

int main(int argc, char **argv)
{
    // Create the node.
    driver node(argc, argv);

    // Run the node.
    node.run();
}
