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

//OpenCV
#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#include <ctype.h>
#endif


#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <D:/robotology/yarp-2.3.6/include/yarp/dev/ControlBoardInterfaces.h>
#include <D:/robotology/yarp-2.3.6/include/yarp/dev/ControlBoardPid.h>
//#include <yarp/dev/all.h>

//POPULATION
#include "PopCod_Disp2D.h"

//IPP
#include <ipp.h>

//NAMESPACE
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

void create_anaglyph(IplImage *dest,IplImage *left,IplImage *right)
{
	for(int i=0; i<dest->width*dest->height; i++)
	{
		dest->imageData[3*i]= left->imageData[i];
		dest->imageData[3*i+1]= left->imageData[i];
		dest->imageData[3*i+2]= right->imageData[i];
	}
}

void Gray2RGB(IplImage *dest,IplImage *src)
{
	for(int i=0; i<dest->width*dest->height; i++)
	{
		dest->imageData[3*i]=src->imageData[i];
		dest->imageData[3*i+1]=src->imageData[i];
		dest->imageData[3*i+2]=src->imageData[i];
	}
}

int main(int argc, char *argv[]) 
{
	//YARP INIT
    Network yarp;

	BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortL;  // make a port for reading images
	BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortR;  
	BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outPortL;  // make a port for reading images
	BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outPortR;  

    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outPortA;  // make a port for sending images
    BufferedPort<yarp::os::Bottle> targetPort;  // make a port for sending control

	imagePortL.open("/vergControl/image/in_l");  // give the port a name
	imagePortR.open("/vergControl/image/in_r");  // give the port a name
    outPortL.open("/vergControl/image/grayL");
	outPortR.open("/vergControl/image/grayR");
	outPortA.open("/vergControl/image/anaglyph");
    targetPort.open("/vergControl/target");

	//POPULATION INIT
	ImageOf<PixelRgb> *imageLeft;
	imageLeft = imagePortL.read();  // read an image to get the size

	int cart_width = imageLeft->width();
	int cart_height = imageLeft->height();
	int cart_width_res = 80;
	int cart_height_res = 60;

	const IppLibraryVersion* libver = ippiGetLibVersion();
	printf("%s %s  %s (%c%c%c%c)\n\n", libver->Name, libver->Version,  libver->BuildDate,
		libver->targetCpu[0],libver->targetCpu[1],libver->targetCpu[2],libver->targetCpu[3]);

	char filter[25]="Gt43B0.0208f0.063.bin";  float dmax=6.0f;         //max detectable disparity value
	int n_scale=1;           //number of scales
	int n_phase=7;	         //number of phase shifts
	float energy_th=0.0001f;  //energy threshold
	int ori_thr=0;           //threshold on orientation (0 if not used)

	//CONSTRUCTOR
	PopCod_Disp2D population_response(filter,cart_width_res,cart_height_res,n_scale,n_phase,dmax,energy_th,ori_thr);

	//Image allocation
	IplImage *img_left = cvCreateImage( cvSize(cart_width, cart_height), 8, 3);
	IplImage *img_right = cvCreateImage( cvSize(cart_width, cart_height), 8, 3);
	IplImage *left_gray = cvCreateImage( cvSize(cart_width, cart_height), 8, 1);
	IplImage *right_gray = cvCreateImage( cvSize(cart_width, cart_height), 8, 1);
	IplImage *left_gray_res = cvCreateImage( cvSize(cart_width_res, cart_height_res), 8, 1);
	IplImage *right_gray_res = cvCreateImage( cvSize(cart_width_res, cart_height_res), 8, 1);
	//IplImage *img_left_resize = cvCreateImage( cvSize(cart_width_res, cart_height_res), 8, 3);
	//IplImage *img_right_resize = cvCreateImage( cvSize(cart_width_res, cart_height_res), 8, 3);
	IplImage *img_left_out = cvCreateImage( cvSize(cart_width_res, cart_height_res), 8, 3);
	IplImage *img_right_out = cvCreateImage( cvSize(cart_width_res, cart_height_res), 8, 3);
	IplImage *anaglyph = cvCreateImage( cvSize(cart_width_res, cart_height_res), 8, 3);

	//PID
	//Pid *pid_verg;
	//getPidRaw(4, pid_verg);

	//cout<<pid_verg.

	double d_verg, gain = 800.0;
	int counter = 0;
	char name_image[100];

	//resetTorquePid(4)=0;
	//resetTorquePid(5)=0;

	//MAIN LOOP
	while(1)
	{
		//Preparing the output
		ImageOf<PixelBgr> outImageA;// = outPortA.prepare(); //set an output image
		outImageA.wrapIplImage(anaglyph); //wrap outImage on anaglyph (do it once??)

		ImageOf<PixelBgr> outImageL;// = outPortL.prepare(); //set an output image
		outImageL.wrapIplImage(img_left_out); //wrap outImage on anaglyph (do it once??)

		ImageOf<PixelBgr> outImageR;// = outPortR.prepare(); //set an output image
		outImageR.wrapIplImage(img_right_out); //wrap outImage on anaglyph (do it once??)
		
		//Capturing a new image (left and right)
		ImageOf<PixelRgb> *imageLeft = imagePortL.read();  // read an image
		ImageOf<PixelRgb> *imageRight = imagePortR.read();  // read an image

		//CASTING from ImageOf to Ipl
		img_left=(IplImage *)imageLeft->getIplImage();
		img_right=(IplImage *)imageRight->getIplImage();

		//RESIZING
		cvCvtColor( img_left, left_gray, CV_RGB2GRAY );
		cvCvtColor( img_right, right_gray, CV_RGB2GRAY );

		cvResize(left_gray, left_gray_res, 1);
		//sprintf(name_image, "D:/ICUB/vergence/data_save/left_%d.bmp", counter);
		//cvSaveImage(name_image ,left_gray_res);
		cvResize(right_gray, right_gray_res, 1);

		//Casting to grey level float
		//population_response.Load_Img_RGB((unsigned char*)img_left->imageData, 'L', left_gray->imageData);
		population_response.Load_Img_Gray((unsigned char*)left_gray_res->imageData, 'L');
		//sprintf(name_image, "D:/ICUB/vergence/data_save/left_%d.bmp", counter);
		//cvSaveImage(name_image ,left_gray);
		//population_response.Load_Img_RGB((unsigned char*)img_right->imageData, 'R', right_gray->imageData);
		population_response.Load_Img_Gray((unsigned char*)right_gray_res->imageData, 'R');
		//sprintf(name_image, "D:/ICUB/vergence/data_save/right_%d.bmp", counter);
		//cvSaveImage(name_image ,right_gray);
		/*population_response.Load_Img_RGB_yarp(imageLeft->getPixelAddress(0,0), 'L', left_gray->imageData);
		population_response.Load_Img_RGB_yarp(imageRight->getPixelAddress(0,0), 'R', right_gray->imageData);*/

		//Computing vergence control
		population_response.Compute_vergence();
						
		d_verg=gain*population_response.GetVergence();

		//if(abs(d_verg)<0.2)
		//	d_verg=0;
		//else 
		//if(abs(d_verg)>1)
		//	d_verg=(int)(d_verg*2.0);

		printf("CONTROLLI: %f \t \n",d_verg);

		//Sending the control to headTracker (??)
		Bottle &target=targetPort.prepare();
		target.clear();
		target.addDouble(d_verg);

		targetPort.write();

		//Creating an Sending ANAGLYPH
		create_anaglyph(anaglyph,left_gray_res,right_gray_res);
		Gray2RGB(img_left_out,left_gray_res);
		Gray2RGB(img_right_out,right_gray_res);
		//sprintf(name_image, "D:/ICUB/vergence/data_save/anaglyph_%d.bmp", counter);
		//cvSaveImage(name_image ,anaglyph);

		outPortA.prepare() = outImageA;
		outPortL.prepare() = outImageL;
		outPortR.prepare() = outImageR;

		outPortA.write();
		outPortL.write();
		outPortR.write();
	
		counter++;
	}//end MAIN LOOP

    return 0;
}