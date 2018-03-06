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

#include "opencv2/opencv.hpp"

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

#include "VergenceControl/VergenceControl.h"

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace vergencecontrol;


class Controller : public RFModule
{
    PolyDriver driver;
    IControlMode2 *ictrl;
    IPositionControl2 *ipos;
    IVelocityControl2 *ivel;

    BufferedPort<ImageOf<PixelRgb>> iPortL;
    BufferedPort<ImageOf<PixelRgb>> iPortR;
    BufferedPort<ImageOf<PixelRgb>> oPortAnaglyph;
    
    string ini_filename, weights_filename;
    VergenceControl *population;
    double scale;

public:
    bool configure(ResourceFinder &rf) override
    {
        string stemName = rf.check("name", Value("iCubVergenceControl")).asString();
        string robot =rf.check("robot", Value("icubSim")).asString();
        ini_filename = rf.check("ini", Value("../../../../data/Gt43B0.0208f0.063ph7.ini")).asString();
        weights_filename = rf.check("weights", Value("../../../../data/vergence-weights.bin")).asString();
        scale = rf.check("scale", Value(1.0)).asDouble();

        Property options;
        options.put("device", "remote_controlboard");
        options.put("remote", ("/" + robot + "/head").c_str());
        options.put("local", "/" + stemName + "/head");
        if (!driver.open(options)) {
            yError() << "Unable to open" << options.find("device").asString();
            return false;
        }

        driver.view(ictrl);
        driver.view(ipos);
        driver.view(ivel);

        // attain a starting vergence prior to
        // enabling the control  
        ictrl->setControlMode(5,VOCAB_CM_POSITION);
        ipos->setRefSpeed(5,10.0);
        ipos->positionMove(5,5.0);

        bool done=false;
        double t0=Time::now();
        while (!done && (Time::now()-t0<5.0)) {
            ipos->checkMotionDone(5,&done);
            Time::yield();
        }

        ictrl->setControlMode(5,VOCAB_CM_VELOCITY);
        ivel->stop(5);

        iPortL.open(("/" + stemName + "/image/left:i").c_str());
        iPortR.open(("/" + stemName + "/image/right:i").c_str());
        oPortAnaglyph.open(("/" + stemName + "/image/anaglyph:o").c_str());

        population = nullptr;
        return true;
    }

    double getPeriod() override
    {
        // synch with incoming images
        return 0.0;
    }

    void resizeImage(const ImageOf<PixelRgb> &rgb, ImageOf<PixelMono> &mono) const
    {
        ImageOf<PixelMono> tmp; tmp.resize(rgb);
        mono.resize((int)(scale*tmp.width()), (int)(scale*tmp.height()));

        Mat rgbMat = cvarrToMat((IplImage*)rgb.getIplImage());
        Mat tmpMat = cvarrToMat((IplImage*)tmp.getIplImage());
        Mat monoMat = cvarrToMat((IplImage*)mono.getIplImage());

        cvtColor(rgbMat, tmpMat, CV_RGB2GRAY);
        resize(tmpMat, monoMat, monoMat.size());
    }

    void createAnaglyh(const ImageOf<PixelMono> &left, const ImageOf<PixelMono> &right,
                       ImageOf<PixelRgb> &anaglyph)
    {
        yAssert((left.width() == right.width()) && (left.height() == right.height()));
        anaglyph.resize(left.width(), left.height());

        Mat leftMat = cvarrToMat((IplImage*)left.getIplImage());
        Mat rightMat = cvarrToMat((IplImage*)right.getIplImage());
        Mat anaglyphMat = cvarrToMat((IplImage*)anaglyph.getIplImage());

        Mat planes[3];
        leftMat.copyTo(planes[0]);
        leftMat.copyTo(planes[1]);
        rightMat.copyTo(planes[2]);
        merge(planes, 3, anaglyphMat);
    }

    bool updateModule() override
    {
        // Capture a pair of new images (left and right) (blocking calls)
        ImageOf<PixelRgb> *iLeftRgb = iPortL.read();
        ImageOf<PixelRgb> *iRightRgb = iPortR.read();
        if ((iLeftRgb == nullptr) || (iRightRgb == nullptr)) {
            return false;
        }

        // Init vergence control
        if (population == nullptr) {
            population = new VergenceControl((int)(scale*iLeftRgb->width()), (int)(scale*iLeftRgb->height()),
                                             ini_filename, weights_filename, 3);
        }

        ImageOf<PixelMono> iLeftMono;
        resizeImage(*iLeftRgb, iLeftMono);
        Mat iMatLeftMono = cvarrToMat((IplImage*)iLeftMono.getIplImage());

        ImageOf<PixelMono> iRightMono;
        resizeImage(*iRightRgb, iRightMono);
        Mat iMatRightMono = cvarrToMat((IplImage*)iRightMono.getIplImage());

        // Compute vergence control
        population->loadImg(iMatLeftMono, 'L');
        population->loadImg(iMatRightMono, 'R');

        float gain[2] = { 10.0, -1 };
        population->setVergenceGAIN(gain);
        population->computeVergenceControl();
        population->printVergence();
        
        ivel->velocityMove(5,population->getVergenceH());

        createAnaglyh(iLeftMono, iRightMono, oPortAnaglyph.prepare());
        oPortAnaglyph.write();

        return true;
    }

    bool interruptModule() override
    {
        iPortL.interrupt();
        iPortR.interrupt();
        
        return true;
    }

    bool close() override
    {
        delete population;

        iPortL.close();
        iPortR.close();
        oPortAnaglyph.close();

        ivel->stop(5);
        ictrl->setControlMode(5,VOCAB_CM_POSITION);
        driver.close();
        
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

    Controller controller;
    return controller.runModule(rf);
}