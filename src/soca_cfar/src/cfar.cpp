#include "cfar.h"

CFAR::CFAR()
{
}

CFAR::CFAR(int train_cells, int guard_cells, float Pfa)
{
    assert(train_cells % 2 == 0);
    assert(guard_cells % 2 == 0);
    this->train_cells = train_cells;
    this->guard_cells = guard_cells;
    this->Pfa = Pfa;

    this->train_hs = this->train_cells / 2;
    this->guard_hs = this->guard_cells / 2;
    this->total_hs = this->train_hs + this->guard_hs;

    this->threshold_mul = CFAR::calcMultiplier();

    this->rank = this->train_cells / 2;
    this->total_train_cells = this->train_hs * (2 * this->train_hs + 2 * this->guard_hs + 1);
}

CFAR::~CFAR()
{
}

double CFAR::calcMultiplier()
{
    arma::vec coeffs(this->train_cells, arma::fill::zeros);
    coeffs.front() = -this->Pfa / 2;
    for (int i = 0; i <= (this->train_cells / 2 - 1); i++)
    {
        int index = this->train_cells / 2 + i;
        double coeff = boost::math::binomial_coefficient<double>(this->train_cells / 2 - 1 + i, i);
        coeffs(index) = coeff;
    }

    arma::cx_vec roots = arma::roots(arma::reverse(coeffs));

    double x = -1;
    std::for_each(roots.begin(), roots.end(), [&x](arma::cx_double root)
                  {
        if ((root.imag() == 0) && (root.real() > 0.0) && (root.real() < 0.5))
        {
            if (root.real() > x) x = root.real();
        } });
    if (x == -1)
    {
        // std::cerr << "Real root not found\n";
        return -1;
    }
    return (1 / x - 2) * this->train_cells / 2;
}

int CFAR::getTrainCells()
{
    return this->train_cells;
}

int CFAR::getGuardCells()
{
    return this->guard_cells;
}

float CFAR::getPfa()
{
    return this->Pfa;
}

float CFAR::getThresholdMultiplier()
{
    return this->threshold_mul;
}

cv::Mat CFAR::soca_1d_naive(cv::Mat &img)
{
    cv::Mat img_gray;
    if (img.channels() == 3)
    {
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        img_gray = img.clone();
    }

    int rows = img_gray.rows;
    int cols = img_gray.cols;
    cv::Mat res = cv::Mat::zeros(img_gray.size(), CV_32F);

    for (int row = this->train_hs + this->guard_hs; row < rows - this->train_hs - this->guard_hs; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            float leading_sum = 0.0, lagging_sum = 0.0;
            for (int i = row - this->train_hs - this->guard_hs; i < row + this->train_hs + this->guard_hs + 1; ++i)
            {
                if ((i - row) > this->guard_hs)
                    lagging_sum += img_gray.at<uchar>(i, col);
                else if ((i - row) < -this->guard_hs)
                    leading_sum += img_gray.at<uchar>(i, col);
            }
            float sum_train = std::min(leading_sum, lagging_sum);
            float num = (this->threshold_mul * sum_train / (2 * this->train_hs));
            res.at<float>(row, col) = (img_gray.at<uchar>(row, col) > num) ? img_gray.at<uchar>(row, col) : 0.0f;
        }
    }

    return res;
}

cv::Mat CFAR::soca_2d_naive(cv::Mat &img)
{
    cv::Mat img_gray;
    if (img.channels() == 3)
    {
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        img_gray = img.clone();
    }

    int rows = img_gray.rows;
    int cols = img_gray.cols;
    cv::Mat res = cv::Mat::zeros(img_gray.size(), CV_32F);

    for (int row = (this->train_hs + this->guard_hs) + 1; row < (rows - this->train_hs - this->guard_hs); row++)
    {
        for (int col = (this->train_hs + this->guard_hs) + 1; col < (cols - this->train_hs - this->guard_hs); col++)
        {
            float leading_sum = 0.0, lagging_sum = 0.0;
            for (int i = row - this->train_hs - this->guard_hs; i <= row + this->train_hs + this->guard_hs; ++i)
            {
                for (int j = col - this->train_hs - this->guard_hs; j <= col + this->train_hs + this->guard_hs; ++j)
                {
                    if (std::abs(i - row) <= this->guard_hs && std::abs(j - col) <= this->guard_hs)
                    {
                        continue;
                    }
                    if ((i < row && j < col) || (i < row && j == col) || (i == row && j < col))
                    {
                        leading_sum += img_gray.at<uchar>(i, j);
                    }
                    else
                    {
                        lagging_sum += img_gray.at<uchar>(i, j);
                    }
                }
            }
            float sum_train = std::min(leading_sum, lagging_sum);
            float num = (this->threshold_mul * sum_train / this->total_train_cells);
            res.at<float>(row, col) = (img_gray.at<uchar>(row, col) > num) ? img_gray.at<uchar>(row, col) : 0.0f;
        }
    }

    return res;
}

// Main functions
cv::Mat CFAR::soca_1d(cv::Mat &img)
{
    cv::Mat img_gray;
    if (img.channels() == 3)
    {
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        img_gray = img;
    }

    cv::Mat integral_image;
    cv::integral(img_gray, integral_image, CV_32F);
    cv::Mat trimmed_image = integral_image(cv::Rect(1, 1, integral_image.cols - 1, integral_image.rows - 1));
    cv::Mat res = cv::Mat::zeros(img_gray.size(), CV_32F);

    int rows = trimmed_image.rows;
    int cols = trimmed_image.cols;

    for (int row = this->total_hs; row < rows - this->total_hs; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            float leading_sum = calc_rect_sum(trimmed_image, row - this->guard_hs - this->train_hs, col, 1, this->train_hs);
            float lagging_sum = calc_rect_sum(trimmed_image, row + this->guard_hs, col, 1, this->train_hs);
            float sum_train = std::min(leading_sum, lagging_sum);
            float num = (this->threshold_mul * sum_train / (2 * this->train_hs));
            res.at<float>(row, col) = (img_gray.at<uchar>(row, col) > num) ? img_gray.at<uchar>(row, col) : 0.0f;
        }
    }

    return res;
}

cv::Mat CFAR::soca_2d(cv::Mat &img)
{
    cv::Mat img_gray;
    if (img.channels() == 3)
    {
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        img_gray = img.clone();
    }

    cv::Mat integral_image;
    cv::integral(img_gray, integral_image, CV_32F);
    cv::Mat trimmed_image = integral_image(cv::Rect(1, 1, integral_image.cols - 1, integral_image.rows - 1));
    cv::Mat res = cv::Mat::zeros(img_gray.size(), CV_32F);

    int rows = trimmed_image.rows;
    int cols = trimmed_image.cols;

    for (int row = this->total_hs + 1; row < rows - this->total_hs; row++)
    {
        for (int col = this->total_hs + 1; col < cols - this->total_hs; col++)
        {
            float leading_guard = calc_rect_sum(trimmed_image, row - this->guard_hs, col - this->guard_hs, this->guard_hs, 2 * this->guard_hs + 1);
            float leading_sum = calc_rect_sum(trimmed_image, row - this->total_hs, col - this->total_hs, this->total_hs, (2 * this->total_hs + 1));
            float leading_train = (leading_sum - leading_guard) / this->total_train_cells;

            float lagging_guard = calc_rect_sum(trimmed_image, row - this->guard_hs, col + 1, this->guard_hs, (2 * this->guard_hs + 1));
            float lagging_sum = calc_rect_sum(trimmed_image, row - this->total_hs, col + 1, this->total_hs, (2 * this->total_hs + 1));
            float lagging_train = (lagging_sum - lagging_guard) / this->total_train_cells;

            float num = (this->threshold_mul * std::min(leading_train, lagging_train));
            res.at<float>(row, col) = (img_gray.at<uchar>(row, col) > num) ? img_gray.at<uchar>(row, col) : 0.0f;
        }
    }

    return res;
}

cv::Mat CFAR::soca_vert(cv::Mat &img)
{
    cv::Mat img_gray;
    if (img.channels() == 3)
    {
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        img_gray = img;
    }

    cv::Mat integral_image;
    cv::integral(img_gray, integral_image, CV_32F);
    cv::Mat trimmed_image = integral_image(cv::Rect(1, 1, integral_image.cols - 1, integral_image.rows - 1));
    cv::Mat res = cv::Mat::zeros(img_gray.size(), CV_32F);

    int rows = trimmed_image.rows;
    int cols = trimmed_image.cols;

    for (int row = this->total_hs + 1; row < rows - this->total_hs; row++)
    {
        for (int col = this->total_hs + 1; col < cols - this->total_hs; col++)
        {
            float leading_guard = calc_rect_sum(trimmed_image, row - this->guard_hs, col - this->guard_hs, (2 * this->guard_hs + 1), this->guard_hs);
            float leading_sum = calc_rect_sum(trimmed_image, row - this->total_hs, col - this->total_hs, (2 * this->total_hs + 1), this->total_hs);
            float leading_train = (leading_sum - leading_guard) / this->total_train_cells;

            float lagging_guard = calc_rect_sum(trimmed_image, row + 1, col - this->guard_hs, (2 * this->guard_hs + 1), this->guard_hs);
            float lagging_sum = calc_rect_sum(trimmed_image, row + 1, col - this->total_hs, (2 * this->total_hs + 1), this->total_hs);
            float lagging_train = (lagging_sum - lagging_guard) / this->total_train_cells;

            float num = (this->threshold_mul * std::min(leading_train, lagging_train));
            res.at<float>(row, col) = (img_gray.at<uchar>(row, col) > num) ? img_gray.at<uchar>(row, col) : 0.0f;
            // res.at<float>(row, col) = (img_gray.at<uchar>(row, col) > num) ? 255.0f : 0.0f;
        }
    }

    return res;
}

float CFAR::calc_rect_sum(cv::Mat &img, int x, int y, int w, int h)
{
    int x1 = std::max(0, x);
    int y1 = std::max(0, y);
    int x2 = std::min(img.rows - 1, x + h - 1);
    int y2 = std::min(img.cols - 1, y + w - 1);

    float sum = img.at<float>(x2, y2) - (x1 > 0 ? img.at<float>(x1 - 1, y2) : 0) - (y1 > 0 ? img.at<float>(x2, y1 - 1) : 0) + (x1 > 0 && y1 > 0 ? img.at<float>(x1 - 1, y1 - 1) : 0);

    return sum;
}
