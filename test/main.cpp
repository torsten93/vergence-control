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

#include "opencv2/core.hpp"
//#include "opencv2/core/utility.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui.hpp"

#include <ctime>
#include <cstdio>
#include <iostream>

#include "VergenceControl/VergenceControl.h"

using namespace std;
using namespace cv;

static void help()
{
    printf("\nThis program demonstrated the use of a bioinspired VERGENCE CONTROL based on the binocular energy model\n"
           "The stereo image is filtered with a Gabor filter bank, to compute binocular energy, and the control seeks\n"
           "to maximize such energy.\n\n"
           "Usage: VergenceControl [image_left image_right]\n\n\n");
}


bool STOP = false, LOOP = false;
Point2d C;

void on_mouse(int event, int x, int y, int d, void *ptr)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        C.x = x;
        C.y = y;

        LOOP = true;
    }
    if  ( event == EVENT_RBUTTONDOWN )
    {
        STOP = true;
    }
}


void meshgrid(int maxX, int maxY, Mat &X, Mat &Y)
{
  for (int i = 0; i <= maxX-1; i++)
    for (int j = 0; j <= maxY-1; j++){
        X.at<float>(i,j) = (float)j;
        Y.at<float>(i,j) = (float)i;
    }
}


int test_single_image(const string &left_filename, const string &right_filename)
{
    clock_t start_time, end_time;

    Mat L = imread(left_filename, IMREAD_GRAYSCALE);
    if( L.empty() )
    {

        printf("Cannot read image file: %s\n", left_filename.c_str());
        return -1;
    }

    Mat R = imread(right_filename, IMREAD_GRAYSCALE);
    if( R.empty() )
    {
        printf("Cannot read image file: %s\n", right_filename.c_str());
        return -1;
    }

    int SX,SY;
    MatSize sz = R.size;
    SY = sz[0]; SX = sz[1];

    VergenceControl POPULATION(SX, SY, "../Gt43B0.0208f0.063ph7.ini", "../vergence-weights.bin", 3);

    float GAIN[2] = {10.0,-1};
    POPULATION.setVergenceGAIN(GAIN);

    POPULATION.loadImg(L, 'L');
    POPULATION.loadImg(R, 'R');

    start_time = clock();
    int d;

    for(d=0;d<10;d++){

        POPULATION.computeVergenceControl();

        Scalar* VC = POPULATION.getVergence();

        POPULATION.printVergence();
    }
    end_time = clock();

    float seconds = (end_time-start_time)/1000.0;

    printf("\n Number of images: %d \n Elaboration time %2.2f sec\n FPS: %2.2f\n", d, seconds, float(d)/seconds);
}


int test_mouse(const string &left_filename, const string &right_filename)
{
    Mat L = imread(left_filename, IMREAD_GRAYSCALE);

    if( L.empty() )
    {
        printf("Cannot read image file: %s\n", left_filename.c_str());
        return -1;
    }

    Mat Lint;
    L.copyTo(Lint);

    Mat R = imread(right_filename, IMREAD_GRAYSCALE);
    if( R.empty() )
    {
        printf("Cannot read image file: %s\n", right_filename.c_str());
        return -1;
    }
    Mat Rint;
    R.copyTo(Rint);

    if(R.size() != L.size())
    {
        printf("Left and right images have different size");
        return -1;
    }

    int SX,SY;
    MatSize sz = R.size;
    SX = sz[0]; SY = sz[1];

    Mat ANAG(SX,SY,CV_8UC3,Scalar(0,0,0));
    Mat planes[3] = {Lint,Lint,Rint};

    Mat map_x(SX, SY, CV_32FC1), map_y(SX, SY, CV_32FC1), map_xw(SX, SY, CV_32FC1), map_yw(SX, SY, CV_32FC1);;
    meshgrid(SX, SY, map_x, map_y);

    VergenceControl POPULATION(SY, SX, "../Gt43B0.0208f0.063ph7.ini", "../vergence-weights.bin", 3);

    float GAIN[2] = {5.0,-0.5};
    POPULATION.setVergenceGAIN(GAIN);

    // Create Anaglyph image
    Lint.copyTo(planes[0]);
    Lint.copyTo(planes[1]);
    Rint.copyTo(planes[2]);

    merge(planes, 3, ANAG);

    imshow("ANAGLYPH",ANAG);

    setMouseCallback("ANAGLYPH", on_mouse, NULL);

    float STATE_H = 0.0, STATE_V = 0.0;
    while (!STOP)
    {
        imshow("ANAGLYPH",ANAG);
        int KEY = waitKey(1);

        if(KEY == 27)
            break;

        if (LOOP){
            cout << "Left button of the mouse is clicked - position (" << C.x << ", " << C.y << ")" << endl;
            //STOP = true;

            POPULATION.setCenter(C);

            for(int i=0; i< 12; i++){
                POPULATION.loadImg(Lint, 'L');
                POPULATION.loadImg(Rint, 'R');

                POPULATION.computeVergenceControl();

                float VH = POPULATION.getVergenceH();
                float VV = POPULATION.getVergenceV();

                POPULATION.printVergence();

                STATE_H += VH; STATE_V += VV;
                map_xw = map_x - STATE_H;
                map_yw = map_y - STATE_V;
                remap(L, Lint, map_xw, map_yw, INTER_LINEAR, BORDER_CONSTANT);

                map_xw = map_x + STATE_H;
                map_yw = map_y + STATE_V;
                remap(R, Rint, map_x, map_yw, INTER_LINEAR, BORDER_CONSTANT);

                Lint.copyTo(planes[0]);
                Lint.copyTo(planes[1]);
                Rint.copyTo(planes[2]);

                merge(planes, 3, ANAG);


                imshow("ANAGLYPH",ANAG);
                int KEY = waitKey(1);

                }
            }

        LOOP = false;
    }
}


int test_mouse_scale(const string &left_filename, const string &right_filename)
{
    Mat L = imread(left_filename, IMREAD_GRAYSCALE);

    if( L.empty() ) {
        printf("Cannot read image file: %s\n", left_filename.c_str());
        return -1;
    }
    else
        printf("Left image %s loaded, image size %dx%d\n",left_filename.c_str(),L.rows,L.cols);

    Mat Lint;
    L.copyTo(Lint);

    float SCALE_FACTOR = 0.25;

    Mat Lscale;
    resize(Lint, Lscale, Size(), SCALE_FACTOR, SCALE_FACTOR);

    Mat LscaleInt;
    Lscale.copyTo(LscaleInt);

    Mat R = imread(right_filename, IMREAD_GRAYSCALE);
    if( R.empty() ) {
        printf("Cannot read image file: %s\n", right_filename.c_str());
        return -1;
    }
    else
        printf("Right image %s loaded, image size %dx%d\n",left_filename.c_str(),R.rows,R.cols);

    Mat Rint;
    R.copyTo(Rint);

    Mat Rscale;
    resize(Rint, Rscale, Size(), SCALE_FACTOR, SCALE_FACTOR);

    Mat RscaleInt;
    Rscale.copyTo(RscaleInt);

    if(R.size() != L.size())
    {
        printf("Left and right images have different size");
        return -1;
    }

    int SX,SY;
    MatSize sz = R.size;
    SX = sz[1]; SY = sz[0];

    int SXscale,SYscale;
    MatSize sz_scale = Rscale.size;
    SXscale = sz_scale[1]; SYscale = sz_scale[0];

    VergenceControl POPULATION(SXscale, SYscale, "../Gt43B0.0208f0.063ph7.ini", "../vergence-weights.bin", 3);

    Mat ANAG(SY,SX,CV_8UC3,Scalar(0,0,0));
    Mat planes[3] = {LscaleInt,LscaleInt,RscaleInt};

    Mat map_x(SY, SX, CV_32FC1), map_y(SY, SX, CV_32FC1), map_xw(SY, SX, CV_32FC1), map_yw(SY, SX, CV_32FC1);;
    meshgrid(SY, SX, map_x, map_y);

    Mat map_x_scale(SYscale, SXscale, CV_32FC1), map_y_scale(SYscale, SXscale, CV_32FC1), map_xw_scale(SYscale, SXscale, CV_32FC1), map_yw_scale(SYscale, SXscale, CV_32FC1);;
    meshgrid(SYscale, SXscale, map_x_scale, map_y_scale);

    float GAIN[2] = {5.0,-1.0};
    POPULATION.setVergenceGAIN(GAIN);

    // Create Anaglyph image
    Lint.copyTo(planes[0]);
    Lint.copyTo(planes[1]);
    Rint.copyTo(planes[2]);

    merge(planes, 3, ANAG);

    int SSX, SSY;
    POPULATION.getSubImgSize(&SSX, &SSY);

    C.x = SX/2; C.y = SY/2;
    Point2d F1(C.x - SSX * SCALE_FACTOR, C.y - SSY * SCALE_FACTOR);
    Point2d F2(C.x + SSX * SCALE_FACTOR, C.y + SSY * SCALE_FACTOR);

    rectangle(ANAG, F1, F2, Scalar(0,255,0), 2);

    imshow("ANAGLYPH",ANAG);
    setMouseCallback("ANAGLYPH", on_mouse, NULL);

    float STATE_H = 0.0, STATE_V = 0.0;
    while (!STOP)
    {
        imshow("ANAGLYPH",ANAG);
        int KEY = waitKey(1);

        if(KEY == 27)
            break;

        if (LOOP){
            cout << "\nLeft button of the mouse is clicked - position (" << C.x << ", " << C.y << ")" << endl;
            //STOP = true;

            F1.x = C.x - SSX * SCALE_FACTOR; F1.y = C.y - SSY * SCALE_FACTOR;
            F2.x = C.x + SSX * SCALE_FACTOR; F2.y = C.y + SSY * SCALE_FACTOR;

            C.x = C.x * SCALE_FACTOR;
            C.y = C.y * SCALE_FACTOR;
            POPULATION.setCenter(C);

            for(int i=0; i< 12; i++){

                POPULATION.loadImg(LscaleInt, 'L');
                POPULATION.loadImg(RscaleInt, 'R');

                POPULATION.computeVergenceControl();

                float VH = POPULATION.getVergenceH();
                float VV = POPULATION.getVergenceV();

                POPULATION.printVergence();

                // IMAGE FOR COMPUTATION
                STATE_H += VH; STATE_V += VV;

                map_xw_scale = map_x_scale - STATE_H;
                map_yw_scale = map_y_scale - STATE_V;
                remap(Lscale, LscaleInt, map_xw_scale, map_yw_scale, INTER_LINEAR, BORDER_CONSTANT);

                map_xw_scale = map_x_scale + STATE_H;
                map_yw_scale = map_y_scale + STATE_V;
                remap(Rscale, RscaleInt, map_x_scale, map_yw_scale, INTER_LINEAR, BORDER_CONSTANT);

                // IMAGE FOR DISPLAY
                map_xw = map_x - STATE_H / SCALE_FACTOR;
                map_yw = map_y - STATE_V / SCALE_FACTOR;
                remap(L, Lint, map_xw, map_yw, INTER_LINEAR, BORDER_CONSTANT);

                map_xw = map_x + STATE_H / SCALE_FACTOR;
                map_yw = map_y + STATE_V / SCALE_FACTOR;
                remap(R, Rint, map_x, map_yw, INTER_LINEAR, BORDER_CONSTANT);

                Lint.copyTo(planes[0]);
                Lint.copyTo(planes[1]);
                Rint.copyTo(planes[2]);

                merge(planes, 3, ANAG);

                rectangle(ANAG, F1, F2, Scalar(0,255,0), 2);

                imshow("ANAGLYPH",ANAG);
                int KEY = waitKey(1);

                }
            }

        LOOP = false;
    }
}


int main(int argc, const char *argv[])
{
    help();
    string left_filename, right_filename;

    if (argc < 2) {
        left_filename  = "../images/Lmed.png";
        right_filename = "../images/Rmed.png";
    }
    else {
        left_filename = argv[0];
        right_filename = argv[1];
    }

    return test_mouse_scale(left_filename,right_filename);
}
