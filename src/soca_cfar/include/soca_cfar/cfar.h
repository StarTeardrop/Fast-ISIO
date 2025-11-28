#pragma once

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

#include <armadillo>
#include <boost/math/special_functions/binomial.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/photo.hpp>

class CFAR
{
private:
    int train_cells; 
    int guard_cells; 
    int total_train_cells;
    int train_hs;
    int guard_hs;
    int total_hs;
    float Pfa;
    int rank;


    float threshold_mul; 
public:
    CFAR();
    CFAR(int train_cells, int guard_cells, float Pfa);
    ~CFAR();

    double calcMultiplier();
    int getTrainCells();
    int getGuardCells();
    float getPfa();
    float getThresholdMultiplier();

    cv::Mat soca_1d_naive(cv::Mat &img);
    cv::Mat soca_2d_naive(cv::Mat &img);

    cv::Mat soca_1d(cv::Mat &img);
    cv::Mat soca_2d(cv::Mat &img);
    cv::Mat soca_vert(cv::Mat &img);
    float calc_rect_sum(cv::Mat &img, int x, int y, int w, int h);
};
