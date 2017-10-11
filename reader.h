//
// Created by Ranqing on 2017/10/8.
//

#ifndef AMLS_PCL_READER_H
#define AMLS_PCL_READER_H

#include "../Qing/qing_string.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

inline void qing_read_stereo_info(string infoname, int &start_x_0, int &start_y_0, int &start_x_1, int &start_y_1, double * stereomtx) {
    fstream fin(infoname.c_str(), ios::in);
    if (fin.is_open() == false) {
        cerr << "failed to open " << infoname << endl;
        return;
    }

    string temp;
    fin >> temp >> temp >> temp >> temp >> temp >> temp >> temp >> temp >> temp;

    fin >> temp;
    start_x_0 = qing_string_2_int(temp);
    fin >> temp;
    start_y_0 = qing_string_2_int(temp);
    fin >> temp;
    start_x_1 = qing_string_2_int(temp);
    fin >> temp;
    start_y_1 = qing_string_2_int(temp);

    fin >> temp >> temp;
    fin >> temp;
    fin >> temp;

    fin >> temp >> temp >> temp >> temp;
    stereomtx[0 * 4 + 0] = 1.0;
    stereomtx[0 * 4 + 1] = 0.0;
    stereomtx[0 * 4 + 2] = 0.0;
    stereomtx[0 * 4 + 3] = qing_string_2_double(temp);
    fin >> temp >> temp >> temp >> temp;
    stereomtx[1 * 4 + 0] = 0.0;
    stereomtx[1 * 4 + 1] = 1.0;
    stereomtx[1 * 4 + 2] = 0.0;
    stereomtx[1 * 4 + 3] = qing_string_2_double(temp);
    fin >> temp >> temp >> temp >> temp;
    stereomtx[2 * 4 + 0] = 0.0;
    stereomtx[2 * 4 + 1] = 0.0;
    stereomtx[2 * 4 + 2] = 0.0;
    stereomtx[2 * 4 + 3] = qing_string_2_double(temp);
    fin >> temp >> temp >> temp;
    stereomtx[3 * 4 + 0] = 0.0;
    stereomtx[3 * 4 + 1] = 0.0;
    stereomtx[3 * 4 + 2] = qing_string_2_double(temp);
    stereomtx[3 * 4 + 3] = 0.0;
    fin.close();
    return;
}

inline void read_in_stereo_datas(const string stereoname, cv::Point2f &pt, int &base_d, double * stereo_mtx) {
    int st_x_0, st_y_0, st_x_1, st_y_1;
    qing_read_stereo_info(stereoname, st_x_0, st_y_0, st_x_1, st_y_1, stereo_mtx);
    pt.x = st_x_0;
    pt.y = st_y_0;
    base_d = st_x_0 - st_x_1;
}

inline void read_in_disp_datas(const string dspname, const int height, const int width, vector<float> &dspdata) {
    int size = width * height;
    dspdata.resize(size, 0.f);
    fstream fin(dspname.c_str(), ios::in);
    if (fin.is_open() == false) {
        cerr << "failed to open " << dspname << endl;
        return;
    }
    for (int i = 0; i < size; ++i) {
        fin >> dspdata[i];
    }
    fin.close();
}

inline void qing_rewrite_dsp(string workdir, string jpgname, string dspname, string re_dspname) {

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


inline void qing_write_point_color_ply(const string &plyname, const vector<cv::Vec3f> &points, const vector<cv::Vec3f> &colors) {
    fstream fout(plyname.c_str(), ios::out);
    if (fout.is_open() == false) {
        cerr << "failed to open " << plyname << endl;
        return;
    }
    fout << "ply" << endl;
    fout << "format ascii 1.0" << endl;
    fout << "element vertex " << points.size() << endl;
    fout << "property float x" << endl;
    fout << "property float y" << endl;
    fout << "property float z" << endl;
    fout << "property uchar red" << endl;
    fout << "property uchar green" << endl;
    fout << "property uchar blue" << endl;
    fout << "end_header" << endl;

    for (int i = 0; i < points.size(); ++i) {
        cv::Vec3f p = points[i];
        cv::Vec3f c = colors[i];
        fout << p.val[0] << ' ' << p.val[1] << ' ' << p.val[2] << ' '
             << c.val[0] << ' ' << c.val[1] << ' ' << c.val[2] << endl;

    }
    fout.close();
}


#endif //AMLS_PCL_READER_H
