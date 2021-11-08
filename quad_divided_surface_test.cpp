#include "quad_divided_surface_test.h"
#include "opencv2/imgcodecs.hpp"
#include "Eigen/Core"

#include<iostream>
#include <ctime>

namespace test {

    namespace {

        void SaveMat(const cv::Mat& mat, const std::string& filename) {
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9);
            bool result = false;
            try
            {
                result = imwrite(filename + ".png", mat, compression_params);
            }
            catch (const cv::Exception& ex)
            {
                fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
            }
            if (result)
                printf("Saved %ws  \n", filename);
            else
                printf("ERROR: Can't save %ws.\n", filename);
        }

        static void DebugQuadDividedSurface(QuadDividedCubicSurface& surface, const std::string& filename, double min_v = 0., double max_v = 10.) {
            CHECK_GT(max_v, min_v);
            double ux, uy, w;
            w = surface.master_box_width();
            const int canvas_width = 512;
            cv::Mat mat(canvas_width, canvas_width, CV_8UC4);
            CV_Assert(mat.channels() == 4);

            double j_step = w / canvas_width;
            double i_step = j_step;

            for (int i = 0; i < canvas_width; ++i) {
                for (int j = 0; j < canvas_width; ++j) {
                    double x = (j + 0.5) * j_step;
                    double y = (canvas_width - i - 0.5) * i_step;

                    double v = math::Clamp((surface(x, y) - min_v) / (max_v - min_v), 0., 1.0);

                    double value = 1.0 - v;
                    cv::Vec4b& bgra = mat.at<cv::Vec4b>(i, j);
                    bgra[0] = UCHAR_MAX; // Blue
                    bgra[1] = cv::saturate_cast<uchar>(value * UCHAR_MAX); // Green
                    bgra[2] = UCHAR_MAX; // Red
                    bgra[3] = UCHAR_MAX; // Alpha

                }
            }

            SaveMat(mat, filename);
        }


        /*
        * void TestAsACubicFunction() {

            using Matrix4d = Eigen::Matrix<double, 4, 4>;

            using RowVector4d = Eigen::Matrix<double, 1, 4>;

            using ColVector4d = Eigen::Matrix<double, 4,1>;

            auto sample = [](const Matrix4d& surf, double x, double y)->double {
                double x2 = x * x;
                double x3 = x * x2;
                double y2 = y * y;
                double y3 = y * y2;
                
                RowVector4d x_polynomials;
                x_polynomials << 1, x, x2, x3;
                ColVector4d y_polynomials;
                y_polynomials << 1, y, y2, y3;

                return  x_polynomials * surf * y_polynomials;
             };

            srand(0);

            double master_width = 1.;
            int depth = 8;
            QuadDividedCubicSurface surface(depth, master_width);

            int id_ub = (1 << (depth - 1)) + 1;
            double w = master_width / (1 << (depth - 1));
            int check_division = 3;
            double w_check = w / check_division;
            double check_margin = 0.3;
            surface.reserve(id_ub * id_ub);

            for (int k = 0; k< 10; ++k) {
                LOG(ERROR) << "case " << k;
                Matrix4d surf = Matrix4d::Random();

                
                surface.reset();
                surface.reserve(20000);

                double min_v = +std::numeric_limits<double>::infinity();
                double max_v = -std::numeric_limits<double>::infinity();
                clock_t t0 = clock();
                for (int i = 0; i < id_ub; ++i)
                    for (int j = 0; j < id_ub; ++j) {
                        double sample_v = sample(surf, i * w, j * w);
                        min_v = std::min(sample_v, min_v);
                        max_v = std::max(sample_v, max_v);
                        surface.SetSample(depth-1, i, j, sample_v);
                    }
                LOG(ERROR) << "after set: " <<(clock() - t0) <<"|"<<surface.NumSamples();
                surface.Initialize();
                LOG(ERROR) << "after init: " << (clock() - t0) ;
                surface.Freeze();

                LOG(ERROR) << "after freeze: " << (clock() - t0) ;

                double max_abs_err = 0.;
                for (int i = 0; i < id_ub*check_division; ++i)
                    for (int j = 0; j < id_ub*check_division; ++j) {
                        double x = i * w_check;
                        double y = j * w_check;
                        if (x > master_width - check_margin || x < check_margin ||
                            y > master_width - check_margin || y < check_margin) {
                            continue;
                        }

                        double ground_truth = sample(surf, x, y);
                        double value = surface(x, y);

                        max_abs_err = std::max(std::abs(value - ground_truth),max_abs_err);
                    }

                LOG(ERROR) << "after operator():" << (clock() - t0) ;
                CHECK_LT(max_abs_err, (max_v - min_v) * 1e-7);
            }

            
        }
        */
        

        /*
        void TestAddMultiply() {
            using  Matrix4d = Eigen::Matrix<double, 4, 4>;
            using RowVector4d = Eigen::Matrix<double, 1, 4>;
            using ColVector4d = Eigen::Matrix<double, 4, 1>;


            auto sample = [](const Matrix4d& surf, double x, double y)->double {
                RowVector4d x_polynomials;
                x_polynomials << 1, x, x* x, x* x* x;
                ColVector4d y_polynomials;
                y_polynomials << 1, y, y* y, y* y* y;

                return  x_polynomials * surf * y_polynomials;
            };

            

            srand(0);

            double master_width = 1.;
            int depth = 8;
            QuadDividedCubicSurface surface1(depth, master_width);
            QuadDividedCubicSurface surface2(depth, master_width);

            int id_ub = (1 << (depth - 1)) + 1;
            double w = master_width / (1 << (depth - 1));
            int check_division = 3;
            double w_check = w / check_division;
            double check_margin = 0.3;
            surface1.reserve(id_ub * id_ub);
            surface2.reserve(id_ub * id_ub);

            auto make_random_surface = [&](QuadDividedCubicSurface* surface, Matrix4d* surf) {
                *surf = Matrix4d::Random();

                surface->reset();

                double min_v = +std::numeric_limits<double>::infinity();
                double max_v = -std::numeric_limits<double>::infinity();
                for (int i = 0; i < id_ub; ++i)
                    for (int j = 0; j < id_ub/2; ++j) {
                        double sample_v = sample(*surf, i * w, j * w);
                        CHECK(surface->SetSample(depth - 1, i, j, sample_v));
                        min_v = std::min(sample_v, min_v);
                        max_v = std::max(sample_v, max_v);
                    }

                for (int i = 0; i < (id_ub -1)/2+1; ++i)
                    for (int j = 0; j < (id_ub-1)/2+1; ++j) {
                        double sample_v = sample(*surf, i * w * 2, j * w * 2);
                        CHECK(surface->SetSample(depth - 2, i, j, sample_v));
                        min_v = std::min(sample_v, min_v);
                        max_v = std::max(sample_v, max_v);
                    }

                surface->Initialize();

                return std::make_pair(min_v, max_v);
            };


            for (int k = 0; k < 10; ++k) {
                LOG(ERROR) << "case " << k;
                double gain = rand()*1.0/RAND_MAX;
                Matrix4d surf1, surf2;
                double min_v1, max_v1, min_v2, max_v2;
                std::tie(min_v1,max_v1) = make_random_surface(&surface1, &surf1);
                std::tie(min_v2, max_v2)= make_random_surface(&surface2, &surf2);

                surface2.Multiply(gain);
                max_v2 *= gain;
                min_v2  *= gain;
                surface1.Add(surface2);
                surface1.Freeze();

                double max_abs_err = 0.;
                for (int i = 0; i < id_ub * check_division; ++i)
                    for (int j = 0; j < id_ub * check_division; ++j) {
                        double x = i * w_check;
                        double y = j * w_check;
                        if (x > master_width - check_margin || x < check_margin ||
                            y > master_width - check_margin || y < check_margin) {
                            continue;
                        }

                        double ground_truth = sample(surf1, x, y) + gain* sample(surf2, x, y);
                        double value = surface1(x, y);

                        max_abs_err = std::max(std::abs(value - ground_truth), max_abs_err);
                    }

                CHECK_LT(max_abs_err, (max_v1 + max_v2 - min_v1 - min_v2)* 1e-5);

               // if (k == 5) {
                //    DebugQuadDividedSurface(surface1, "Show", min_v2+min_v1, max_v2+max_v1);
               // }
            }
        }


        void DrawExample() {

            QuadDividedCubicSurface s1(5, 10.0);

            s1.SetSample(0, 0, 0, 3.0);

            s1.SetSample(0, 1, 1, -3.0);

            for (int i = 0; i < 8; ++i) {
                s1.SetSample(3, i + 1, i, -2.0);
            }

            for (int i = 0; i < 4; ++i) {
                s1.SetSample(2, i, 3 - i, 3.0);
            }

            for (int i = 8; i < 17; ++i) {
                int j = 24 - i;
                s1.SetSample(4, i, j, 1.0);
            }

            s1.SetSample(4, 3, 2, 1.0);
            s1.Initialize();
            s1.Freeze();
            DebugQuadDividedSurface(s1, "s1_image", -3.0, 3.0);
        }
*/
	} // namespace






	void TestQuadDividedSurface() {
        

        QuadDividedCubicSurface strip(8, 10.0);


        strip.CreateStrip(2.0,2.0, 4.0, 3.0);

        CHECK(strip.IsPopulated());

        DebugQuadDividedSurface(strip, "strip");
	}
} // namespace test