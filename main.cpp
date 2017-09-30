#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <fstream>

#include "processer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

inline void qing_cwd()
{
    char cwd[1024];
    getcwd(cwd, 1024);
    printf("\t%s\n", cwd);
}

int qing_string_2_int(const string& str) {
    stringstream ss(str);
    int num;
    ss >> num;
    return num;
}

string qing_int_2_string(const int& num) {
    stringstream ss;
    ss << num;
    return ss.str();
}

double qing_string_2_double(const string& str) {
    stringstream ss(str);
    double num;
    ss >> num;
    return num;
}

string qing_double_2_string(const double& num) {
    stringstream ss;
    ss << num;
    return ss.str();
}

void convert(const string src_file_name, const string dst_file_name) {

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr src_pointcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::PLYReader reader;
    reader.read(src_file_name, *src_pointcloud);
    int size = src_pointcloud->points.size();
    if(0==size) {
        cerr << "failed to load " << src_file_name << endl;
        return ;
    }

    dst_pointcloud->points.reserve(2*size);
    for(int i = 0; i < size; ++i) {
        pcl::PointXYZRGBNormal src_pt = (*src_pointcloud)[i];
        pcl::PointXYZRGB dst_pt ;
        dst_pt.x = src_pt.x;
        dst_pt.y = src_pt.y;
        dst_pt.z = src_pt.z;
        dst_pt.rgb = src_pt.rgb;
        dst_pointcloud->points.push_back(dst_pt);
    }
    cout << dst_pointcloud->points.size() << " points.." << endl;

    pcl::PLYWriter writer;
    writer.write(dst_file_name, *dst_pointcloud);
    cout << "saving " << dst_file_name << endl;

}

void qing_read_stereo_info(string infoname, int& start_x_0, int& start_y_0, int& start_x_1, int& start_y_1, double stereomtx[][4]) {
    fstream fin(infoname.c_str(), ios::in);
    if(fin.is_open() == false) {
        cerr << "failed to open " << infoname << endl;
        return;
    }

    string temp;
    fin >> temp >> temp >> temp >> temp >> temp >> temp >> temp >> temp >> temp;

    fin >> temp;  start_x_0 = qing_string_2_int(temp);
    fin >> temp;  start_y_0 = qing_string_2_int(temp);
    fin >> temp;  start_x_1 = qing_string_2_int(temp);
    fin >> temp;  start_y_1 = qing_string_2_int(temp);

    fin >> temp >> temp;
    fin >> temp;
    fin >> temp;

    fin >> temp >> temp >> temp >> temp;
    stereomtx[0][0] = 1.0;
    stereomtx[0][1] = 0.0;
    stereomtx[0][2] = 0.0;
    stereomtx[0][3] = qing_string_2_double(temp);
    fin >> temp >> temp >> temp >> temp;
    stereomtx[1][0] = 0.0;
    stereomtx[1][1] = 1.0;
    stereomtx[1][2] = 0.0;
    stereomtx[1][3] = qing_string_2_double(temp);
    fin >> temp >> temp >> temp >> temp;
    stereomtx[2][0] = 0.0;
    stereomtx[2][1] = 0.0;
    stereomtx[2][2] = 0.0;
    stereomtx[2][3] = qing_string_2_double(temp);
    fin >> temp >> temp >> temp ;
    stereomtx[3][0] = 0.0;
    stereomtx[3][1] = 0.0;
    stereomtx[3][2] = qing_string_2_double(temp);
    stereomtx[3][3] = 0.0;
    fin.close();
    return ;
}


void qing_rewrite_dsp(string workdir, string jpgname, string dspname, string re_dspname){
    jpgname = workdir + jpgname;
    dspname = workdir + dspname;
    re_dspname = workdir + re_dspname;

    cv::Mat img = cv::imread(jpgname, 0);
    if(img.data == NULL) {
        cerr << "failed to open " << jpgname << endl;
        return ;
    }
    int hei = img.size().height;
    int wid = img.size().width;
    int size = hei * wid;
    cout << hei << '\t' << wid << endl;

    float * dsp = new float[size];
    fstream fin(dspname.c_str(), ios::in);
    if (fin.is_open() == false) {
        cerr << "failed to open " << dspname << endl;
        return ;
    }
    for(int i = 0; i < size; ++i) {
        fin >> dsp[i];
    }
    fin.close();

    fstream fout(re_dspname.c_str(), ios::out);
    if(fout.is_open() == false){
        cerr << "failed to open " << re_dspname << endl;
        return ;
    }
    for(int i = 0; i < size; ++i) {
        fout << dsp[i] ;
        if((i+1)%wid==0)
            fout << endl;
        else
            fout << ' ';
    }
    fout.close();
    cout << "saving " << re_dspname << endl;
}

void read_in_datas(string jpgname, string dspname, string stereoname, cv::Mat& img, cv::Point2f& pt, float& base_d, double stereo_mtx[][4], vector<float>& dsp_data){
    int st_x_0, st_y_0, st_x_1, st_y_1;

    qing_read_stereo_info(stereoname, st_x_0, st_y_0, st_x_1, st_y_1, stereo_mtx);
    pt.x = st_x_0;
    pt.y = st_y_0;
    base_d = st_x_0 - st_x_1;

//    cout << pt << endl;

    img = cv::imread(jpgname, 0);
    int wid = img.size().width;
    int hei = img.size().height;
    int size = wid * hei;
    dsp_data.resize(wid * hei, 0.f);
    fstream fin(dspname.c_str(), ios::in);
    if (fin.is_open() == false) {
        cerr << "failed to open " << dspname << endl;
        return ;
    }
    for(int i = 0; i < size; ++i) {
        fin >> dsp_data[i];
    }
    fin.close();
}

// qing_dsp_2_depth(&dsp_data.front(), msk_mtx.ptr<uchar>(0), img_mtx.ptr<uchar>(0), stereo_mtx, start_pt_0, base_d, w, h, pointcnt, points, colors);

void qing_dsp_2_depth(float * ptr_dsp, unsigned char * ptr_msk, unsigned char * ptr_bgr, double qmtx[][4], cv::Point2f crop_point, int base_d, int w, int h,
                      int& pointcnt, vector<cv::Vec3f>& points, vector<cv::Vec3f>& colors) {

    for(int y = 0, idx = 0; y < h; ++y) {
        for(int x = 0; x < w; ++x) {
            if(0==ptr_msk[idx] || 0 == ptr_dsp[idx]) {idx++;continue;}
            else {
                double uvd1[4], xyzw[4] ;
                uvd1[0] = x + crop_point.x;
                uvd1[1] = y + crop_point.y;
                uvd1[2] = ptr_dsp[idx] + base_d;
                uvd1[3] = 1.0;

                xyzw[0] = qmtx[0][0] * uvd1[0] + qmtx[0][1] * uvd1[1] + qmtx[0][2] * uvd1[2] + qmtx[0][3] * uvd1[3];
                xyzw[1] = qmtx[1][0] * uvd1[0] + qmtx[1][1] * uvd1[1] + qmtx[1][2] * uvd1[2] + qmtx[1][3] * uvd1[3];
                xyzw[2] = qmtx[2][0] * uvd1[0] + qmtx[2][1] * uvd1[1] + qmtx[2][2] * uvd1[2] + qmtx[2][3] * uvd1[3];
                xyzw[3] = qmtx[3][0] * uvd1[0] + qmtx[3][1] * uvd1[1] + qmtx[3][2] * uvd1[2] + qmtx[3][3] * uvd1[3];

                points.push_back(cv::Vec3f(xyzw[0]/xyzw[3], xyzw[1]/xyzw[3], xyzw[2]/xyzw[3]));
                colors.push_back(cv::Vec3f(ptr_bgr[3*idx + 2], ptr_bgr[3*idx + 1], ptr_bgr[3*idx + 0]));
                idx++;
            }
        }
    }
    pointcnt = points.size();
    //   cout << points.size() << " points are generated... " << endl;

}

void qing_write_point_color_ply(const string& plyname, const vector<cv::Vec3f>& points, const vector<cv::Vec3f>& colors)
{
    fstream fout (plyname.c_str(), ios::out);
    if(fout.is_open() == false) {
        cerr << "failed to open " << plyname << endl;
        return ;
    }
    fout << "ply" << endl;
    fout << "format ascii 1.0"     << endl;
    fout << "element vertex "      << points.size() << endl;
    fout << "property float x"     << endl;
    fout << "property float y"     << endl;
    fout << "property float z"     << endl;
    fout << "property uchar red"   << endl;
    fout << "property uchar green" << endl;
    fout << "property uchar blue"  << endl;
    fout << "end_header"           << endl;

    for(int i = 0; i < points.size(); ++i)
    {
        cv::Vec3f p = points[i];
        cv::Vec3f c = colors[i];


        fout << p.val[0] << ' ' << p.val[1] << ' ' << p.val[2] << ' '
             << c.val[0] << ' ' << c.val[1] << ' ' << c.val[2] << endl;

    }
    fout.close();
}

int main(int argc, char * argv[]) {
    qing_cwd();
    string data_folder = "/Users/Qing/Projects/HumanRefinerAMLS/test_new/";
//    string file1 = argv[1];
//    string imgname = argv[2];
//    string file2 = "rewrite_" + file1;
//    qing_rewrite_dsp(data_folder, imgname, file1, file2);
//    return 1;


    string dsp_filename = "rewrite_erode_final_disp_l_0.txt";
    string dsp_jpgname = "final_disp_l_0.jpg";
    string img_filename = "crop_imgL.jpg";
    string msk_filename = "crop_mskL.jpg";
    string stereo_filename = "stereo_A07A08.info";

    cv::Mat dsp_img;
    cv::Point2f start_pt_0;
    vector<float> dsp_data(0);
    float base_d;
    double stereo_mtx[4][4];

    read_in_datas(data_folder+dsp_jpgname, data_folder+dsp_filename, data_folder+stereo_filename, dsp_img, start_pt_0, base_d, stereo_mtx, dsp_data);
    cout << "after reading in datas:" << endl;
    cout << dsp_img.size() << endl;
    cout << start_pt_0 << endl;
    cout << base_d << endl;
    cout << stereo_mtx[0][0] << '\t' << stereo_mtx[0][1] << '\t' << stereo_mtx[0][2] << '\t' << stereo_mtx[0][3] << endl;
    cout << stereo_mtx[3][0] << '\t' << stereo_mtx[3][1] << '\t' << stereo_mtx[3][2] << '\t' << stereo_mtx[3][3] << endl;
    cout << dsp_data.size() << endl;

    vector<cv::Vec3f> points;
    vector<cv::Vec3f> colors;
    cv::Mat img_mtx = cv::imread(data_folder + img_filename, 1);
    cv::Mat msk_mtx = cv::imread(data_folder + msk_filename, 0), binary_msk_mtx;
    cv::threshold(msk_mtx, binary_msk_mtx, 125, 255, cv::THRESH_BINARY);
    int w = img_mtx.size().width;
    int h = img_mtx.size().height;
    int pointcnt;
    qing_dsp_2_depth(&dsp_data.front(), msk_mtx.ptr<uchar>(0), img_mtx.ptr<uchar>(0), stereo_mtx, start_pt_0, base_d, w, h, pointcnt, points, colors);

    string out_folder  = "../output/";
    string ply_name = "init_A07A08.ply";
    qing_write_point_color_ply(out_folder + ply_name, points, colors);
    cout << "saving " << out_folder + ply_name << endl;


    string src_file_name = out_folder + ply_name;
    string dst_file_name = out_folder + "rewrite_" + ply_name;
    cout << src_file_name << endl;
    cout << dst_file_name << endl;
    convert(src_file_name, dst_file_name);

    PointcloudProcesser * processer = new PointcloudProcesser();
    if(NULL == processer) {
        cerr << "failed to start a point process pipeline..." << endl;
        return -1;
    }
    processer->load_ply(dst_file_name);
//    vector<qing_vec3f> qing_points(0), qing_colors(0);
//    processer->to_points_and_colors(qing_points, qing_colors);
//    return 1;

    string ply_file_name, out_file_name;
    ply_file_name = dst_file_name;
    processer->load_ply(ply_file_name);
    if(true == processer->get_is_loaded()) {
        processer->outliers_removal(1);
        out_file_name = out_folder + "clean_" + ply_name;
        processer->save_ply(out_file_name);

        processer->down_sampling(10);

        float radius = 15;
        out_file_name = out_folder + "mls_" + qing_int_2_string((int)(radius)) + "_" + ply_name;
        processer->mls_resampling(radius);
        processer->correct_normal(0.f, 0.f, 0.f);
        processer->save_ply(out_file_name);
    }

    return 1;
}


