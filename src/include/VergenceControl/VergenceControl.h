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

#ifndef LIB_VERGENCECONTROL_H
#define LIB_VERGENCECONTROL_H

#include <ctime>
#include <string>

#include <opencv2/opencv.hpp>

//----------------------------------------------------------
//A simple wrapper class for the population coding model
//of vergence control computation, developed in C on FFT convolution
//----------------------------------------------------------
class VergenceControl {
    int width;        //image width
    int height;       //image height

public:
    VergenceControl(int width, int height, const std::string &filter_filename,
                    const std::string &verg_weight, int vergence_channels);
    ~VergenceControl();

    //Read left and right images from file Load_Img_File
    void loadImgFile(const std::string &img_filename, char c);
    void loadImg(const cv::Mat &img, char c);
    void Load_Img_RGB(unsigned char *img, char c, char *gray);
    void Load_Img_GREY(unsigned char *img, char c);

    // Compute vergence control
    void computeVergenceControl();
    void setCenter(const cv::Point2d &cntr);

    float getVergenceH();
    float getVergenceV();
    cv::Scalar* getVergence();
    void printVergence();
    void getVergenceGAIN(float* GAIN);
    void setVergenceGAIN(float* GAIN);
    void getSubImgSize(int *M, int *N);

private:
    // GABOR FILTERS
    typedef struct
    {
        int Nori, Nph;
        float *theta;
        float *phase;
        float phi[2];

        float sigma;
        float f;
        int taps;
        int HalfSize;
        float B;

        //cv::Mat* EvenFilters;
        //cv::Mat* OddFilters;
        cv::Mat1f* Filters[2];
        cv::Mat2f* FiltersFFT[2];
        cv::Mat1i X, Y;
    } Gabor;

    typedef struct
    {
        int Nori, Nph, Nch; // Orientations, Phases, Channels
        cv::Mat1f *VergW; //vergence weights
        cv::Scalar VC[2], NORM;
        float GAIN[2];

    } Verg;

    Gabor Gfilt;
    Verg Vergence;
    int M, N, Mcut, Ncut;
    int Mpatch, Npatch;
    int Mpad, Npad;
    float Verg_control[2];
    cv::Mat L, R, Lpad, Rpad, Lpatch, Rpatch;
    cv::Mat1f Lpatch32F, Rpatch32F;
    cv::Mat Lfft, Rfft;
    cv::Mat Lgray, Rgray;
    cv::Mat1f *Lfilt[2], *Rfilt[2]; //Even/Odd filtered images
    cv::Mat1f *RShiftE[7], *RShiftO[7], *Energy[7]; // Even/Odd filtered and phase shifted right image
    cv::Mat1f tmpEven, tmpOdd, tmpSum;
    cv::Mat SUM_H, SUM_V, SUM_N;
    float fovea_size;
    cv::Mat Gaussian;
    cv::Point2d Center;
    cv::Rect_<double> ROI;
    float testVW;

    std::clock_t start_time, end_time;

    cv::Mat planes[2];
    cv::Mat tmpSpectrum;
    cv::Mat tmpIFT, tmpF;

    void createGaborFilters();
    void loadGaborParams(const std::string &ini_filter_file);
    void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv, cv::Mat1i &X, cv::Mat1i &Y);
    void meshgridTest(const cv::Range &xgv, const cv::Range &ygv, cv::Mat1i &X, cv::Mat1i &Y);
    void Gabor2D(const cv::Mat &X, const cv::Mat &Y, cv::Mat &G,
                 float f, float theta, float sigma, float phi);

    void create_Gaussian(const std::string &ini_filter_file);
    void Gaussian2D(const cv::Mat &X, const cv::Mat &Y, cv::Mat &G, float sigma);

    void imageFFT();
    void filtGaborBankFFT();
    void filtGaborBank();
    void shiftGaborBank();
    void computeEnergy();
    void computeVergence();
    void loadVergenceW(const std::string &verg_filename);
};

#endif
