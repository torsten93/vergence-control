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

#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iterator>
#include <sstream>
#include <iostream>

#include "VergenceControl/VergenceControl.h"
#include "VergenceControl/private/INIReader.h"

using namespace std;
using namespace cv;

VergenceControl::VergenceControl(int width, int height, const string &filter_filename,
                                 const string &verg_weight, int vergence_channels)
{
    //------------------- DEFAULT CONSTRUCTOR -------------------//
    M=height;  N=width;

    // INIT RGB IMAGE TO 0
    L =  Mat(height, width, CV_8UC3, Scalar(0,0,0));
    R =  Mat(height, width, CV_8UC3, Scalar(0,0,0));

    //------------------- VERGENCE GAIN -------------------//
    Vergence.GAIN[0] = 1.0;
    Vergence.GAIN[1] = 0.2;

    //------------------- CREATE GABOR FILTERS AND GAUSSIAN ENVELOPE -------------------//
    loadGaborParams(filter_filename);

    Mcut = Gfilt.taps;
    Ncut = Gfilt.taps;

    /*Mpatch = getOptimalDFTSize(2*Gfilt.taps);
    Npatch = getOptimalDFTSize(2*Gfilt.taps);*/
    Mpatch = 2*Gfilt.taps;
    Npatch = 2*Gfilt.taps;

    Mpad = Mcut + 2*Gfilt.taps;
    Npad = Mcut + 2*Gfilt.taps;

    createGaborFilters();

    create_Gaussian(filter_filename);
    loadVergenceW(verg_weight);

    //------------------- INIT PATCH IMAGE -------------------//
    Lpatch = Mat(Mpatch, Npatch, CV_8UC1, Scalar(0));
    Rpatch = Mat(Mpatch, Npatch, CV_8UC1, Scalar(0));

    Lpatch32F = Mat(Mpatch, Npatch, CV_32FC1, Scalar(0));
    Rpatch32F = Mat(Mpatch, Npatch, CV_32FC1, Scalar(0));

    //------------------- INIT PADDED IMAGE -------------------//
    Lpad =  Mat(height + 2*Gfilt.taps, width + 2*Gfilt.taps, CV_8UC3, Scalar(0,0,0));
    Rpad =  Mat(height + 2*Gfilt.taps, width + 2*Gfilt.taps, CV_8UC3, Scalar(0,0,0));

    //------------------- INIT FFT IMAGE (further versions) -------------------//
    Lfft = Mat(Mpatch, Npatch, CV_32FC2, Scalar(0,0,0));
    Rfft = Mat(Mpatch, Npatch, CV_32FC2, Scalar(0,0,0));

    //------------------- SET ROI TO IMAGE CENTER -------------------//
    Point2d C(N/2,M/2);
    setCenter(C);

    //------------------- INIT FILTERED IMAGE -------------------//
    Lfilt[0] = new Mat1f [Gfilt.Nori]; // Even Filters
    Lfilt[1] = new Mat1f [Gfilt.Nori]; // Odd Filters
    Rfilt[0] = new Mat1f [Gfilt.Nori]; // Even  Filters
    Rfilt[1] = new Mat1f [Gfilt.Nori]; // Odd  Filters
    for (int P=0;P<2;P++) //EVEN/ODD
        for (int T=0;T<Gfilt.Nori;T++){
            Lfilt[P][T] =  Mat(Gfilt.taps, Gfilt.taps, CV_32FC1);
            Rfilt[P][T] =  Mat(Gfilt.taps, Gfilt.taps, CV_32FC1);
        }

    //------------------- INIT FILTERED AND PHASE SHIFTED IMAGE + ENERGY MATRIX -------------------//
    for (int P=0;P<Gfilt.Nph;P++){ //phases
        RShiftE[P] = new Mat1f [Gfilt.Nori]; // Even  Filters
        RShiftO[P] = new Mat1f [Gfilt.Nori]; // Odd  Filters
        Energy[P] = new Mat1f [Gfilt.Nori]; // Binocular Energy
        for (int T=0;T<Gfilt.Nori;T++){
            RShiftE[P][T] =  Mat(Gfilt.taps, Gfilt.taps, CV_32FC1);
            RShiftO[P][T] =  Mat(Gfilt.taps, Gfilt.taps, CV_32FC1);
            Energy[P][T] =  Mat(Gfilt.taps, Gfilt.taps, CV_32FC1);
        }
    }

    SUM_H = Mat(Mpatch,Npatch,CV_32FC1,Scalar(0.0));
    SUM_V = Mat(Mpatch,Npatch,CV_32FC1,Scalar(0.0));
    SUM_N = Mat(Mpatch,Npatch,CV_32FC1,Scalar(0.0));

    planes[0] = Mat(Gfilt.taps, Gfilt.taps, CV_32FC1, Scalar(0.0));
    planes[1] = Mat(Gfilt.taps, Gfilt.taps, CV_32FC1, Scalar(0.0));

    tmpEven = Mat(Gfilt.taps, Gfilt.taps, CV_32FC1, Scalar(0.0));
    tmpOdd  = Mat(Gfilt.taps, Gfilt.taps, CV_32FC1, Scalar(0.0));
    tmpSum  = Mat(Gfilt.taps, Gfilt.taps, CV_32FC1, Scalar(0.0));

    tmpSpectrum = Mat(Mpatch, Npatch, CV_32FC2, Scalar(0.0,0.0));
    tmpIFT = Mat(Mpatch, Npatch, CV_32FC1, Scalar(0.0));
    tmpF = Mat(Mpatch, Npatch, CV_32FC1, Scalar(0.0));

    //------------------- PRINT INFO -------------------//
    printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
    printf("%%%%    A Portable Bio-Inspired Architecture for \n");
    printf("%%%%       Efficient Robotic Vergence Control    \n");
    printf("%%%%                                             \n");
    printf("%%%%                                             \n");
    printf("%%%%    Copyright (c) Sep. 2017                  \n");
    printf("%%%%    All rights reserved.                     \n");
    printf("%%%%                                             \n");
    printf("%%%%    Authors: Agostino Gibaldi, Andrea Canessa, Silvio P. Sabatini\n");
    printf("%%%%                                             \n");
    printf("%%%%    PSPC-lab - Department of Informatics, Bioengineering, \n");
    printf("%%%%    Robotics and Systems Engineering - University of Genoa \n");
    printf("%%%%                                             \n");
    printf("%%%%    The code is released for free use for SCIENTIFIC RESEARCH ONLY. \n");
    printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
    printf("\n\nImages: width=%d, height=%d;\n\n", M, N);
    printf("Gabor Filters: orientations=%d; phases+%d; \n",Gfilt.Nori,Gfilt.Nph);
    printf("               fo=%f; sigma+%f; taps=%d\n",Gfilt.f,Gfilt.sigma,Gfilt.taps);
    printf("\n");
}


VergenceControl::~VergenceControl()
{
    //-------------------DISTRUCTOR-------------------//
    L.release();
    R.release();
    Lgray.release();
    Rgray.release();
    Lpatch.release();
    Rpatch.release();
}


void VergenceControl::loadGaborParams(const string &ini_filter_file)
{
    //------------------- LOAD PARAMETERS FOR GABOR FILTERS -------------------//
    char * cstr = new char [ini_filter_file.length()+1];
    strcpy (cstr, ini_filter_file.c_str());

    // LOAD FILTER PARAMS
    INIReader iniReader(cstr);
    Gfilt.Nori = iniReader.GetInteger("Gabor", "Nori", 0);
    Gfilt.Nph = iniReader.GetInteger("Gabor", "Nph", 0);
    Gfilt.sigma = iniReader.GetReal("Gabor", "sigma", 0.0);
    Gfilt.taps = iniReader.GetInteger("Gabor", "taps", 0);
    Gfilt.B = iniReader.GetReal("Gabor", "B", 0.0);
    Gfilt.f = iniReader.GetReal("Gabor", "f", 0.0);
    string phase = iniReader.Get("Gabor", "phases", "0.0 0.0 0.0 0.0 0.0 0.0 0.0");
    vector<float> v;
    for (string::size_type sz = 0; sz < phase.size();) {
        v.push_back(stof(phase.substr(sz), &sz));
    }
    Gfilt.phase = new float[v.size()];
    for (unsigned int i = 0; i < v.size(); i++) {
        Gfilt.phase[i] = v[i];
    }
    fovea_size = iniReader.GetReal("Gaussian", "fovea_size", 0.0);
}


void VergenceControl::createGaborFilters()
{
    //------------------- CREATE GABOR FILTERS -------------------//
    //Gfilt.taps = getOptimalDFTSize(2*Gfilt.taps);
    Gfilt.taps = Mpatch;

    Gfilt.HalfSize = (Gfilt.taps - Gfilt.taps%2)/2;
    Gfilt.theta = (float*) malloc(sizeof(float)*Gfilt.Nori);
    for (int i=0;i<Gfilt.Nori;i++)
        Gfilt.theta[i] = i*M_PI/Gfilt.Nori;
    Gfilt.phi[0] = M_PI/2;
    Gfilt.phi[1] = 0.0;

    meshgridTest(Range(-Gfilt.HalfSize,Gfilt.HalfSize-1), Range(-Gfilt.HalfSize,Gfilt.HalfSize-1), Gfilt.X, Gfilt.Y);

    Gfilt.Filters[0] = new Mat1f [Gfilt.Nori]; // Even Filters
    Gfilt.Filters[1] = new Mat1f [Gfilt.Nori]; // Odd  Filters

    Gfilt.FiltersFFT[0] = new Mat2f [Gfilt.Nori]; // Even Filters
    Gfilt.FiltersFFT[1] = new Mat2f [Gfilt.Nori]; // Odd  Filters

    for (int P=0;P<2;P++) //EVEN/ODD
        for (int T=0;T<Gfilt.Nori;T++){
            Gfilt.Filters[P][T] = Mat(Gfilt.taps, Gfilt.taps, CV_32FC1);
            Gfilt.FiltersFFT[P][T] = Mat(Gfilt.taps, Gfilt.taps, CV_32FC2);

            Gabor2D(Gfilt.X, Gfilt.Y, Gfilt.Filters[P][T], Gfilt.f, Gfilt.theta[T], Gfilt.sigma, Gfilt.phi[P]);
            Gfilt.Filters[P][T] = Gfilt.Filters[P][T] - mean(Gfilt.Filters[P][T]);

            dft(Gfilt.Filters[P][T], Gfilt.FiltersFFT[P][T], DFT_COMPLEX_OUTPUT);

            split(Gfilt.FiltersFFT[P][T], planes);

        }
}


void VergenceControl::Gabor2D(const Mat &X, const Mat &Y, Mat &G,
                              float f, float theta, float sigma, float phi)
{
    //------------------- 2D GABOR FUNCTION -------------------//
    float Xrot, Yrot;

    int count = 0;
    for(int r=0; r<X.rows; r++)
        for(int c=0; c<X.cols; c++){
            Xrot =  X.at<int>(r,c)*cos(theta)+Y.at<int>(r,c)*sin(theta);
            Yrot = -X.at<int>(r,c)*sin(theta)+Y.at<int>(r,c)*cos(theta);

            G.at<float>(r,c) = exp(-(Xrot*Xrot + Yrot*Yrot)/(2*sigma*sigma))*sin(2*M_PI*f*Xrot + phi);
        }
}


void VergenceControl::create_Gaussian(const string &ini_filter_file)
{
    //------------------- CREATE 2D GAUSSIAN FUNCTION -------------------//
    char * cstr = new char [ini_filter_file.length()+1];
    strcpy (cstr, ini_filter_file.c_str());

    INIReader iniReader(cstr);
    fovea_size = iniReader.GetReal("Gaussian", "fovea_size", 0.0);

    Gaussian = Mat(Gfilt.taps, Gfilt.taps, CV_32FC1);
    Gaussian2D(Gfilt.X, Gfilt.Y, Gaussian, fovea_size);
}


void VergenceControl::Gaussian2D(const Mat &X, const Mat &Y, Mat &G, float sigma)
{
    //------------------- 2D GAUSSIAN FORMULA -------------------//
    for(int r=0; r<X.rows; r++)
        for(int c=0; c<X.cols; c++)
            G.at<float>(r,c) = exp(-(X.at<int>(r,c)*X.at<int>(r,c) + Y.at<int>(r,c)*Y.at<int>(r,c))/(2*sigma*sigma));
}


void VergenceControl::meshgrid(const Mat &xgv, const Mat &ygv, Mat1i &X, Mat1i &Y)
{
    //------------------- MESHGRID UTILITY -------------------//
    repeat(xgv.reshape(1,1), ygv.total(), 1, X);
    repeat(ygv.reshape(1,1).t(), 1, xgv.total(), Y);
}


// helper function (maybe that goes somehow easier)
void VergenceControl::meshgridTest(const Range &xgv, const Range &ygv, Mat1i &X, Mat1i &Y)
{
    //------------------- MESHGRID UTILITY -------------------//
    vector<int> t_x, t_y;
    for (int i = xgv.start; i <= xgv.end; i++) t_x.push_back(i);
        for (int i = ygv.start; i <= ygv.end; i++) t_y.push_back(i);
            meshgrid(cv::Mat(t_x), cv::Mat(t_y), X, Y);
}


void VergenceControl::loadImgFile(const string &img_filename, char c)
{
    //------------------- LOAD IMAGE INTO CLASS FROM FILE -------------------//
    if(c=='L' | c =='l'){
        //------------------- LOAD IMAGE -------------------//
        L = imread(img_filename, IMREAD_GRAYSCALE);

        if( L.empty() )
        {
            printf("Cannot read image file: %s\n", img_filename.c_str());
        }

        //------------------- PAD IMAGE -------------------//
        copyMakeBorder(L, Lpad, Gfilt.taps, Gfilt.taps, Gfilt.taps, Gfilt.taps, BORDER_REFLECT_101);

        //------------------- PATCH IMAGE -------------------//
        Lpatch = Lpad(ROI);

        Lpatch.convertTo(Lpatch32F, CV_32FC1);
        Lpatch32F = Lpatch32F / 255.0;
        Lpatch32F = Lpatch32F - mean(Lpatch32F);

        //------------------- PATCH DFT -------------------//
        dft(Lpatch32F, Lfft, DFT_COMPLEX_OUTPUT);

    }
    else if(c=='R' | c =='r'){
        //------------------- LOAD IMAGE -------------------//
        R = imread(img_filename, IMREAD_GRAYSCALE);

        if(R.empty() )
        {
            printf("Cannot read image file: %s\n", img_filename.c_str());
        }

        //------------------- PAD IMAGE -------------------//
        copyMakeBorder(R, Rpad, Gfilt.taps, Gfilt.taps, Gfilt.taps, Gfilt.taps, BORDER_REFLECT_101);

        //------------------- PATCH IMAGE -------------------//
        Rpatch = Rpad(ROI);

        Rpatch.convertTo(Rpatch32F, CV_32FC1);
        Rpatch32F = Rpatch32F / 255.0;
        Rpatch32F = Rpatch32F - mean(Rpatch32F);

        //------------------- PATCH DFT -------------------//
        dft(Rpatch32F, Rfft, DFT_COMPLEX_OUTPUT);
    }
}


void VergenceControl::loadImg(const Mat &img, char c)
{
    //------------------- LOAD IMAGE INTO CLASS -------------------//
    if(c=='L' | c =='l'){

        img.copyTo(L);

        copyMakeBorder(L, Lpad, Gfilt.taps, Gfilt.taps, Gfilt.taps, Gfilt.taps, BORDER_REFLECT_101);

        Lpatch = Lpad(ROI);

        Lpatch.convertTo(Lpatch32F, CV_32FC1);
        Lpatch32F = Lpatch32F / 255.0;
        dft(Lpatch32F, Lfft, DFT_COMPLEX_OUTPUT);

    }
    else if(c=='R' | c =='r'){

        img.copyTo(R);

        copyMakeBorder(R, Rpad, Gfilt.taps, Gfilt.taps, Gfilt.taps, Gfilt.taps, BORDER_REFLECT_101);

        Rpatch = Rpad(ROI);

        Rpatch.convertTo(Rpatch32F, CV_32FC1);
        Rpatch32F = Rpatch32F / 255.0;
        dft(Rpatch32F, Rfft, DFT_COMPLEX_OUTPUT);
    }
}


void VergenceControl::loadVergenceW(const string &verg_filename)
{
    //------------------- LOAD VERGENCE WEIGHTS INTO CLASS FROM FILE -------------------//
    FILE *fp;
    float tmpf[1];

    fp=fopen(verg_filename.c_str(),"rb");
    if (fp==NULL){
        printf("Error opening file: %s\n",verg_filename.c_str());
        exit(1);
    }

    fread(&Vergence.Nori,sizeof(int),1,fp);
    fread(&Vergence.Nph,sizeof(int),1,fp);
    fread(&Vergence.Nch,sizeof(int),1,fp);

    Vergence.VergW = new Mat1f [Vergence.Nch]; // Even Filters

    for(int c=0; c<Vergence.Nch; c++){
        Vergence.VergW[c] = Mat(Vergence.Nph,Vergence.Nori,CV_32FC1,Scalar(0.0));
        for( int p=0; p<Vergence.Nph; p++)
            for( int o=0; o<Vergence.Nori; o++){
                fread(tmpf,sizeof(float),1,fp);
                Vergence.VergW[c].at<float>(p,o) = tmpf[0];
            }
    }

    fclose(fp);
}


void VergenceControl::computeVergenceControl()
{
    //------------------- COMPUTE VERGENCE CONTROL -------------------//
    start_time = clock();
    //imageFFT();

    //filtGaborBankFFT();
    filtGaborBank();
    shiftGaborBank();
    computeEnergy();
    computeVergence();
    end_time = clock();
}


void VergenceControl::imageFFT()
{
    //------------------- COMPUTE DFT OF IMAGE -------------------//
    Lpatch = L(ROI);

    Lpatch.convertTo(Lpatch32F, CV_32FC1);
    Lpatch32F = Lpatch32F / 255.0;
    Lpatch32F = Lpatch32F - mean(Lpatch32F);
    dft(Lpatch32F, Lfft, DFT_COMPLEX_OUTPUT);

    Rpatch = R(ROI);

    Rpatch.convertTo(Rpatch32F, CV_32FC1);
    Rpatch32F = Rpatch32F / 255.0;
    Rpatch32F = Rpatch32F - mean(Rpatch32F);
    dft(Rpatch32F, Rfft, DFT_COMPLEX_OUTPUT);
}


void VergenceControl::filtGaborBankFFT()
{
    //------------------- COMPUTE CONVOLUTION USING DFT -------------------//
    for (int T=0;T<Gfilt.Nori;T++) //EVEN/ODD
        for (int P=0;P<2;P++){
            mulSpectrums(Lfft,Gfilt.FiltersFFT[P][T],tmpSpectrum, 0);
            dft(tmpSpectrum,tmpIFT, DFT_INVERSE);

            split(tmpIFT, planes);
            planes[0].copyTo(Lfilt[P][T]);

            mulSpectrums(Rfft,Gfilt.FiltersFFT[P][T],tmpSpectrum, 0);
            dft(tmpSpectrum,tmpIFT, DFT_INVERSE);

            split(tmpIFT, planes);
            planes[0].copyTo(Rfilt[P][T]);
        }
}


void VergenceControl::filtGaborBank()
{
    //------------------- COMPUTE CONVOLUTION -------------------//
    for (int T=0;T<Gfilt.Nori;T++) //EVEN/ODD
        for (int P=0;P<2;P++){

            filter2D(Lpatch32F, tmpF, 1, Gfilt.Filters[P][T]);
            tmpF.copyTo(Lfilt[P][T]);

            filter2D(Rpatch32F, tmpF, 1, Gfilt.Filters[P][T]);
            tmpF.copyTo(Rfilt[P][T]);

        }
}


void VergenceControl::shiftGaborBank()
{
    //------------------- SHIFT IN PHASE GABOR RESPONSE -------------------//
    for (int P=0;P<int(Gfilt.Nph);P++) //EVEN/ODD
        for (int T=0;T<Gfilt.Nori;T++){
            RShiftE[P][T] = cos(Gfilt.phase[P])*Rfilt[0][T] - sin(Gfilt.phase[P])*Rfilt[1][T];
            RShiftO[P][T]  = cos(Gfilt.phase[P])*Rfilt[1][T] + sin(Gfilt.phase[P])*Rfilt[0][T];
        }
}


void VergenceControl::computeEnergy()
{
    //------------------- COMPUTE BINOCULAR ENERGY -------------------//
    for (int P=0;P<int(Gfilt.Nph);P++)
        for (int T=0;T<Gfilt.Nori;T++){
            tmpEven = Lfilt[0][T] + RShiftE[P][T];
            tmpEven = tmpEven.mul(tmpEven);

            tmpOdd = Lfilt[1][T] + RShiftO[P][T];
            tmpOdd = tmpOdd.mul(tmpOdd);

            tmpSum = tmpEven+tmpOdd;

            sqrt(tmpSum,tmpSum);
            tmpSum.copyTo(Energy[P][T]);
        }
}


void VergenceControl::computeVergence()
{
    //------------------- COMPUTE VERGENCE CONTROL -------------------//
    SUM_H = 0.0*SUM_H;
    SUM_V = 0.0*SUM_V;
    SUM_N = 0.0*SUM_N;

    for(int P=0; P<Vergence.Nph; P++)
        for( int T=0; T<Vergence.Nori; T++){
            SUM_H = SUM_H +  Vergence.VergW[0].at<float>(P,T) * Energy[P][T].mul(Gaussian);
            SUM_V = SUM_V + Vergence.VergW[1].at<float>(P,T) * Energy[P][T].mul(Gaussian);
            SUM_N = SUM_N + (1.0/(Vergence.Nori*Vergence.Nph)) * Energy[P][T].mul(Gaussian);
        }

        Vergence.NORM  = sum(SUM_N)[0];
        Vergence.VC[0] = Vergence.GAIN[0] * sum(SUM_H)[0] / Vergence.NORM[0];
        Vergence.VC[1] = Vergence.GAIN[1] * sum(SUM_V)[0] / Vergence.NORM[0];
}


float VergenceControl::getVergenceH()
{
    //------------------- GET HORIZONTAL VERGENCE CONTROL -------------------//
    return Vergence.VC[0][0];
}


float VergenceControl::getVergenceV()
{
    //------------------- GET VERTICAL VERGENCE CONTROL -------------------//
    return Vergence.VC[1][0];
}


Scalar* VergenceControl::getVergence()
{
    //------------------- GET  VERGENCE CONTROL -------------------//
    return Vergence.VC;
}


void VergenceControl::printVergence()
{
    //------------------- PRINT VERGENCE CONTROL -------------------//
    printf("VC H: %2.2f \t VC V: %2.2f \t FPS: %2.2f \n", Vergence.VC[0].val[0], Vergence.VC[1].val[0], 1000.0/(end_time-start_time));
}


void VergenceControl::setCenter(const Point2d &cntr)
{
    //------------------- SET IMAGE LOCATION TO COMPUTE VERGENCE -------------------//
    // SET IMAGE CENTER AND ROI
    Center.x = cntr.x + Gfilt.taps;
    Center.y = cntr.y + Gfilt.taps;
    ROI = Rect(Center.x - Mpatch/2,Center.y - Npatch/2, Mpatch, Npatch); // X Y width height
}


void VergenceControl::getVergenceGAIN(float *GAIN)
{
    //------------------- GET VERGENCE GAIN -------------------//
    GAIN = Vergence.GAIN;
}


void VergenceControl::setVergenceGAIN(float *GAIN)
{
    //------------------- SET VERGENCE GAIN -------------------//
    Vergence.GAIN[0] = GAIN[0];
    Vergence.GAIN[1] = GAIN[1];
}


void VergenceControl::getSubImgSize(int *M, int *N)
{
    //------------------- GET SIZE OF IMAGE PATCH -------------------//
    M[0] = Mpatch;
    N[0] = Npatch;
}
