#ifndef BUNDLEPARAMS_H
#define BUNDLEPARAMS_H

#include <string>
#include "../../include/common/command_args.h"

using namespace std;

struct BundleParams{
public:
    BundleParams(int argc, char** argv);
    virtual ~BundleParams(){};

public:
    string input;
    string output;

    CommandArgs arg;

};

 BundleParams::BundleParams(int argc, char** argv)
 {  
    arg.param("input", input, "", "file which will be processed.");
    arg.param("output", output, "", "file which will be writen result.");
    arg.parseArgs(argc, argv);
 }

#endif