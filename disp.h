//
// Created by Ranqing on 2017/10/8.
//

#ifndef AMLS_PCL_DISP_H
#define AMLS_PCL_DISP_H

#include "reader.h"
#include "../Qing/qing_macros.h"


//usage:
//    cout << "Usage: " << argv[0] << " disp_txt_name disp_img_name" << endl;
//    string data_folder = "../test_new/";
//    string file1 = argv[1];
//    string imgname = argv[2];
//    string file2 = "rewrite_" + file1;
//    qing_rewrite_dsp(data_folder, imgname, file1, file2);
//    return 1;

void qing_rewrite_dsp(string workdir, string jpgname, string dspname, string re_dspname) {

    jpgname = workdir + jpgname;
    dspname = workdir + dspname;
    re_dspname = workdir + re_dspname;

    cv::Mat img = cv::imread(jpgname, 0);
    if (img.data == NULL) {
        cerr << "failed to open " << jpgname << endl;
        return;
    }
    int height = img.size().height;
    int width = img.size().width;
    int size = height * width;
    cout << height << '\t' << width << endl;

    float *dsp = new float[size];
    fstream fin(dspname.c_str(), ios::in);
    if (fin.is_open() == false) {
        cerr << "failed to open " << dspname << endl;
        return;
    }
    for (int i = 0; i < size; ++i) {
        fin >> dsp[i];
    }
    fin.close();

    fstream fout(re_dspname.c_str(), ios::out);
    if (fout.is_open() == false) {
        cerr << "failed to open " << re_dspname << endl;
        return;
    }
    for (int i = 0; i < size; ++i) {
        fout << dsp[i];
        if ((i + 1) % width == 0)
            fout << endl;
        else
            fout << ' ';
    }
    fout.close();
    cout << "saving " << re_dspname << endl;
}


void qing_dsp_to_depth_range_grid(float *ptr_dsp, unsigned char *ptr_msk, unsigned char *ptr_bgr, double qmtx[][4],
                                  cv::Point2f crop_point, float base_d, int w, int h,
                                  vector<cv::Vec3f> &points, vector<cv::Vec3f> &colors) {

    int size = w * h;

    points.resize(size);
    colors.resize(size);

    for (int y = 0, idx = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (0 == ptr_msk[idx] || 0 == ptr_dsp[idx]) {
                points[idx] = cv::Vec3f(PT_UNDEFINED, PT_UNDEFINED, PT_UNDEFINED);
            }
            else {
                double uvd1[4], xyzw[4];
                uvd1[0] = x + crop_point.x;
                uvd1[1] = y + crop_point.y;
                uvd1[2] = ptr_dsp[idx] + base_d;
                uvd1[3] = 1.0;

                xyzw[0] = qmtx[0][0] * uvd1[0] + qmtx[0][1] * uvd1[1] + qmtx[0][2] * uvd1[2] + qmtx[0][3] * uvd1[3];
                xyzw[1] = qmtx[1][0] * uvd1[0] + qmtx[1][1] * uvd1[1] + qmtx[1][2] * uvd1[2] + qmtx[1][3] * uvd1[3];
                xyzw[2] = qmtx[2][0] * uvd1[0] + qmtx[2][1] * uvd1[1] + qmtx[2][2] * uvd1[2] + qmtx[2][3] * uvd1[3];
                xyzw[3] = qmtx[3][0] * uvd1[0] + qmtx[3][1] * uvd1[1] + qmtx[3][2] * uvd1[2] + qmtx[3][3] * uvd1[3];

                points[idx] = cv::Vec3f(xyzw[0] / xyzw[3], xyzw[1] / xyzw[3], xyzw[2] / xyzw[3]);
            }
            colors[idx] = cv::Vec3f(ptr_bgr[3 * idx + 0], ptr_bgr[3 * idx + 1], ptr_bgr[3 * idx + 2]);
            idx++;
            continue;
        }
    }
}


void
dsp_to_depth(const string data_folder, const string dsp_txt_name, const string dsp_filename, const string img_filename,
             const string msk_filename, const string stereo_filename, const float scale,
             int &width, int &height, vector<cv::Vec3f> &points, vector<cv::Vec3f> &colors) {
    string imgname = data_folder + img_filename;
    string mskname = data_folder + msk_filename;
    string dspname = data_folder + dsp_filename;
    string dsptxt = data_folder + dsp_txt_name;
    string stereoname = data_folder + stereo_filename;

    cout << "image name: " << imgname << endl;
    cout << "mask name: " << mskname << endl;
    cout << "disp name: " << dspname << endl;
    cout << "disp text: " << dsptxt << endl;
    cout << "stereo name: " << stereoname << endl;

    cv::Mat img = cv::imread(imgname, 1);
    if (NULL == img.data) {
        cerr << "failed to open " << imgname << endl;
        return;
    }
    cv::Mat gray_msk = cv::imread(mskname, 0), msk;
    if (NULL == gray_msk.data) {
        cerr << "failed to open " << mskname << endl;
        return;
    }
    cv::threshold(gray_msk, msk, 75, 255, CV_THRESH_BINARY);
    cv::Mat dsp = cv::imread(dspname, 0);
    if (NULL == dsp.data) {
        cerr << "failed to open " << dspname << endl;
        return;
    }

    width = dsp.size().width;
    height = dsp.size().height;

    cv::Point2f pt;
    float based;
    double stereomtx[4][4];
    vector<float> dspdata(0);
    read_in_stereo_datas(stereoname, pt, based, stereomtx);
    if (1.0 != scale) {
        pt.x /= scale;
        pt.y /= scale;
        based /= scale;
        stereomtx[0][3] /= scale;
        stereomtx[1][3] /= scale;
        stereomtx[2][3] /= scale;
    }
    cout << "start point: " << pt << endl;
    cout << "base d: " << based << endl;
    cout << "stereomtx: \n"
         << stereomtx[0][0] << '\t' << stereomtx[0][1] << '\t' << stereomtx[0][2] << '\t' << stereomtx[0][3] << '\n'
         << stereomtx[1][0] << '\t' << stereomtx[1][1] << '\t' << stereomtx[1][2] << '\t' << stereomtx[1][3] << '\n'
         << stereomtx[2][0] << '\t' << stereomtx[2][1] << '\t' << stereomtx[2][2] << '\t' << stereomtx[2][3] << '\n'
         << stereomtx[3][0] << '\t' << stereomtx[3][1] << '\t' << stereomtx[3][2] << '\t' << stereomtx[3][3] << endl;
    read_in_disp_datas(dsptxt, height, width, dspdata);
    cout << "disp size: " << dspdata.size() << endl;

    uchar *pmsk = msk.ptr < uchar > (0);
    uchar *pimg = img.ptr < uchar > (0);

    qing_dsp_to_depth_range_grid(&dspdata.front(), pmsk, pimg, stereomtx, pt, based, width, height, points, colors);
    cout << "end of qing_dsp_to_depth_range_grid." << endl;

}

#endif //AMLS_PCL_DISP_H
