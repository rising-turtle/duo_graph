#include <iostream>
#include <fstream>
#include "SubmapReader.h"

using namespace std;

int main(int argc, char* argv[])
{
    string fname = "./submaps/0submap.map";
    if(argc >= 2)
    {
        fname = argv[1];
    }
    string oufname = "./submaps/postfile.log";
    resolve::SubmapHeader h;
    ifstream inf(fname.c_str());
    inf>>h;
    print(cout, h);
    CSubmapReader submap;
    submap.read(fname);
    submap.print(oufname);
    return 0;
}
