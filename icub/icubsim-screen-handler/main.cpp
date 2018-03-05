//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%              A Portable Bio-Inspired Architecture for              %%%%
//%%%%                  Efficient Robotic Vergence Control                %%%%
//%%%%                                                                    %%%%
//%%%%                                                                    %%%%
//%%%%  Copyright (c) Sep. 2017                                           %%%%
//%%%%  All rights reserved.                                              %%%%
//%%%%                                                                    %%%%
//%%%%  Authors: Agostino Gibaldi, Andrea Canessa, Silvio P. Sabatini     %%%%
//%%%%                                                                    %%%%
//%%%%  PSPC-lab - Department of Informatics, Bioengineering,             %%%%
//%%%%  Robotics and Systems Engineering - University of Genoa            %%%%
//%%%%                                                                    %%%%
//%%%%  The code is released for free use for SCIENTIFIC RESEARCH ONLY.   %%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include <cstdlib>
#include <string>
#include <cmath>

#include "opencv2/opencv.hpp"

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;


class Handler : public RFModule
{
    RpcClient rpcPortWorld;
    Port oPortTexture;
        
    string img_filename;
    double screen_period;
    double screen_amplitude;

    Vector x0;
    double t0;
    
public:
    bool configure(ResourceFinder &rf) override
    {
        string stemName=rf.check("name", Value("iCubSimScreenHandler")).asString();
        img_filename=rf.check("img",Value("../../../../data/images/Lmed.png")).asString();
        screen_period=rf.check("screen-period",Value(30.0)).asDouble();
        screen_amplitude=rf.check("screen-amplitude",Value(0.5)).asDouble();

        Mat img=imread(img_filename.c_str(),IMREAD_COLOR);
        if (img.empty())
        {
            yError()<<"Unable to locate"<<img_filename;
            return false;
        }

        rpcPortWorld.open(("/" + stemName + "/world:rpc").c_str());
        oPortTexture.open(("/" + stemName + "/texture:o").c_str());

        if (!Network::connect(rpcPortWorld.getName(),"/icubSim/world") ||
            !Network::connect(oPortTexture.getName(),"/icubSim/texture/screen"))
        {
            yError()<<"Unable to connect to icubSim";
            rpcPortWorld.close();
            oPortTexture.close();
            return false;
        }

        Bottle cmd,rep;
        cmd.addString("world");
        cmd.addString("get");
        cmd.addString("screen");
        rpcPortWorld.write(cmd,rep);
        x0.resize(3);
        for (int i=0; i<3; i++)
            x0[i]=rep.get(i).asDouble();

        ImageOf<PixelRgb> texture;
        texture.resize(img.cols,img.rows);
        for (int j=0; j<texture.height(); j++)
            for (int i=0; i<texture.width(); i++)
                texture(i,j)=PixelRgb(img.at<Vec3b>(j,i)[2],img.at<Vec3b>(j,i)[1],img.at<Vec3b>(j,i)[0]);
        oPortTexture.write(texture);
     
        t0=Time::now();
        return true;
    }

    double getPeriod() override
    {
        return 0.1;
    }

    bool updateModule() override
    {
        double t=Time::now()-t0;
        Vector x=x0; x[2]+=0.5*screen_amplitude*(1.0-cos((2.0*M_PI/screen_period)*t));

        Bottle cmd,rep;
        cmd.addString("world");
        cmd.addString("set");
        cmd.addString("screen");
        cmd.addDouble(x[0]);
        cmd.addDouble(x[1]);
        cmd.addDouble(x[2]);
        rpcPortWorld.write(cmd,rep);

        return true;
    }

    bool close() override
    {
        rpcPortWorld.close();
        oPortTexture.close();
        return true;
    }
};



int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "YARP seems unavailable";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Handler handler;
    return handler.runModule(rf);
}