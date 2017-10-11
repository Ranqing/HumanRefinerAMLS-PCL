//
// Created by Ranqing on 2017/10/11.
//

#include "disp.h"


void DispProcesser::init(const string &img_jpg_name, const string &msk_jpg_name, const string &dsp_jpg_name,
                         const string &dsp_txt_name, const string &stereo_info_name) {
# if 0
    cout << "image name: " << img_jpg_name << endl;
    cout << "mask name: " << msk_jpg_name << endl;
    cout << "disp name: " << dsp_jpg_name << endl;
    cout << "disp text: " << dsp_txt_name << endl;
    cout << "stereo name: " << stereo_info_name << endl;
# endif

    m_bgr_mtx = cv::imread(img_jpg_name, 1);
    if (NULL == m_bgr_mtx.data) {
        cerr << "failed to open " << img_jpg_name << endl;
        return;
    }
    cv::Mat gray_msk = cv::imread(msk_jpg_name, 0);
    if (NULL == gray_msk.data) {
        cerr << "failed to open " << msk_jpg_name << endl;
        return;
    }
    cv::threshold(gray_msk, m_msk_mtx, 75, 255, CV_THRESH_BINARY);
    m_dsp_mtx = cv::imread(dsp_jpg_name, 0);
    if (NULL == m_dsp_mtx.data) {
        cerr << "failed to open " << dsp_jpg_name << endl;
        return;
    }
    m_w = m_dsp_mtx.size().width;
    m_h = m_dsp_mtx.size().height;
    m_stereo_mtx = cv::Mat::zeros(4, 4, CV_64FC1);

    read_in_stereo_datas(stereo_info_name, m_crop_pt, m_base_d, m_stereo_mtx.ptr<double>(0));
# if 1
    cout << "start point: " << m_crop_pt << endl;
    cout << "base d: " << m_base_d << endl;
    cout << "stereo mtx: " << m_stereo_mtx << endl;
# endif
    read_in_disp_datas(dsp_txt_name, m_h, m_w, m_dsp_data);
    cout << "disp size: " << m_dsp_data.size() << endl;

}

void
qing_dsp_to_depth_range_grid(float *ptr_dsp, uchar *ptr_msk, uchar *ptr_bgr, double *qmtx, float crop_x, float crop_y,
                             float base_d, int w, int h,
                             vector<cv::Vec3f> &points, vector<cv::Vec3b> &colors) {

    int size = w * h;
    points.resize(size);
    colors.resize(size);

    for (int y = 0, idx = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (0 == ptr_msk[idx] || 0 == ptr_dsp[idx]) {
                points[idx] = cv::Vec3f(PT_UNDEFINED, PT_UNDEFINED, PT_UNDEFINED);
            } else {
                double uvd1[4], xyzw[4];
                uvd1[0] = x + crop_x;
                uvd1[1] = y + crop_y;
                uvd1[2] = ptr_dsp[idx] + base_d;
                uvd1[3] = 1.0;

                xyzw[0] = qmtx[0 * 4 + 0] * uvd1[0] + qmtx[0 * 4 + 1] * uvd1[1] + qmtx[0 * 4 + 2] * uvd1[2] +
                          qmtx[0 * 4 + 3] * uvd1[3];
                xyzw[1] = qmtx[1 * 4 + 0] * uvd1[0] + qmtx[1 * 4 + 1] * uvd1[1] + qmtx[1 * 4 + 2] * uvd1[2] +
                          qmtx[1 * 4 + 3] * uvd1[3];
                xyzw[2] = qmtx[2 * 4 + 0] * uvd1[0] + qmtx[2 * 4 + 1] * uvd1[1] + qmtx[2 * 4 + 2] * uvd1[2] +
                          qmtx[2 * 4 + 3] * uvd1[3];
                xyzw[3] = qmtx[3 * 4 + 0] * uvd1[0] + qmtx[3 * 4 + 1] * uvd1[1] + qmtx[3 * 4 + 2] * uvd1[2] +
                          qmtx[3 * 4 + 3] * uvd1[3];

                points[idx] = cv::Vec3f(xyzw[0] / xyzw[3], xyzw[1] / xyzw[3], xyzw[2] / xyzw[3]);
            }
            colors[idx] = cv::Vec3f(ptr_bgr[3 * idx + 0], ptr_bgr[3 * idx + 1], ptr_bgr[3 * idx + 2]);
            idx++;
        }
    }

}

void qing_dsp_to_depth(float *ptr_dsp, uchar *ptr_msk, uchar *ptr_bgr, double *qmtx, float crop_x, float crop_y,
                       float base_d, int w, int h,
                       vector<cv::Vec3f> &points, vector<cv::Vec3b> &colors) {
    int size = w * h;
    points.reserve(size);
    colors.reserve(size);

    for (int y = 0, idx = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (0 == ptr_msk[idx] || 0 == ptr_dsp[idx]) {
                idx++;
                continue;
            } else {
                double uvd1[4], xyzw[4];
                uvd1[0] = x + crop_x;
                uvd1[1] = y + crop_y;
                uvd1[2] = ptr_dsp[idx] + base_d;
                uvd1[3] = 1.0;

                xyzw[0] = qmtx[0 * 4 + 0] * uvd1[0] + qmtx[0 * 4 + 1] * uvd1[1] + qmtx[0 * 4 + 2] * uvd1[2] +
                          qmtx[0 * 4 + 3] * uvd1[3];
                xyzw[1] = qmtx[1 * 4 + 0] * uvd1[0] + qmtx[1 * 4 + 1] * uvd1[1] + qmtx[1 * 4 + 2] * uvd1[2] +
                          qmtx[1 * 4 + 3] * uvd1[3];
                xyzw[2] = qmtx[2 * 4 + 0] * uvd1[0] + qmtx[2 * 4 + 1] * uvd1[1] + qmtx[2 * 4 + 2] * uvd1[2] +
                          qmtx[2 * 4 + 3] * uvd1[3];
                xyzw[3] = qmtx[3 * 4 + 0] * uvd1[0] + qmtx[3 * 4 + 1] * uvd1[1] + qmtx[3 * 4 + 2] * uvd1[2] +
                          qmtx[3 * 4 + 3] * uvd1[3];

                points.push_back(cv::Vec3f(xyzw[0] / xyzw[3], xyzw[1] / xyzw[3], xyzw[2] / xyzw[3]));
                colors.push_back(cv::Vec3f(ptr_bgr[3 * idx + 0], ptr_bgr[3 * idx + 1], ptr_bgr[3 * idx + 2]));
                idx++;
            }
        }
    }
    cout << "qing_dsp_to_depth: " << points.size() << " points.." << endl;

}

void DispProcesser::to_depth(const int &is_range_grid, const int &level, vector<cv::Vec3f> &points,
                             vector<cv::Vec3b> &colors) {
    cout << "here is to_depth()..." << endl;

    int scale = (0 == level) ? 1 : (1 << level);
    float crop_x = m_crop_pt.x / scale;
    float crop_y = m_crop_pt.y / scale;
    float base_d = m_base_d / scale;
    cv::Mat stereo_mtx = m_stereo_mtx.clone();
    stereo_mtx.at<double>(0, 3) /= scale;
    stereo_mtx.at<double>(1, 3) /= scale;
    stereo_mtx.at<double>(2, 3) /= scale;

    cout << "Level " << level << ", dsp_to_depth: " << endl;
    cout << "scale = " << scale << endl;
    cout << "crop_x = " << crop_x << endl;
    cout << "crop_y = " << crop_y << endl;
    cout << "base_d = " << base_d << endl;
    cout << "stereo_mtx = " << stereo_mtx << endl;

# if 0
    cout << "img size = " << m_bgr_mtx.size() << endl;
    cout << "msk size = " << m_msk_mtx.size() << endl;
    cout << "dsp size = " << m_dsp_mtx.size() << endl;
    cv::imshow("img", m_bgr_mtx);
    cv::imshow("msk", m_msk_mtx);
    cv::imshow("dsp", m_dsp_mtx);
    cv::waitKey(0);
    cv::destroyAllWindows();
# endif

    uchar *pmsk = m_msk_mtx.ptr<uchar>(0);
    uchar *pimg = m_bgr_mtx.ptr<uchar>(0);
    double *pstereomtx = stereo_mtx.ptr<double>(0);

    if (is_range_grid) {
        qing_dsp_to_depth_range_grid(&m_dsp_data.front(), pmsk, pimg, pstereomtx, crop_x, crop_y, base_d, m_w, m_h,
                                     points, colors);
        cout << "end of qing_dsp_to_depth_range_grid." << endl;
    } else {
        qing_dsp_to_depth(&m_dsp_data.front(), pmsk, pimg, pstereomtx, crop_x, crop_y, base_d, m_w, m_h, points,
                          colors);
        cout << "end of qing_dsp_to_depth." << endl;
    }

}