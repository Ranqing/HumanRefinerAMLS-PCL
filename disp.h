//
// Created by Ranqing on 2017/10/11.
//

#ifndef AMLS_PCL_DISP_H
#define AMLS_PCL_DISP_H

#include "reader.h"
#include "../Qing/qing_macros.h"

class DispProcesser {
public:
    DispProcesser()  {}
    ~DispProcesser() {}

    void init(const string& img_jpg_name, const string& msk_jpg_name, const string& dsp_jpg_name, const string& dsp_txt_name, const string& stereo_info_name);
    void to_depth(const int& is_range_grid, const int& level, vector<cv::Vec3f>& points, vector<cv::Vec3b>& colors);

    vector<float>& get_dsp_data() { return m_dsp_data; }
    cv::Point2f get_crop_pt()  { return m_crop_pt; }
    cv::Mat get_bgr_mtx()  { return m_bgr_mtx; }
    cv::Mat get_msk_mtx()  { return m_msk_mtx; }

    int get_w() { return m_w; }
    int get_h() { return m_h; }
    int get_base_d() { return m_base_d; }
    int get_level()  { return m_level; }

private:
    cv::Mat m_dsp_mtx, m_bgr_mtx, m_msk_mtx;
    cv::Mat m_stereo_mtx;
    cv::Point2f m_crop_pt;
    int m_w, m_h, m_base_d, m_level;
    vector<float> m_dsp_data;
};


#endif //AMLS_PCL_DISP_H
