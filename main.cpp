#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include "processer.h"
#include "disp.h"

using namespace cv;

inline void qing_cwd() {
    char cwd[1024];
    getcwd(cwd, 1024);
    printf("\t%s\n", cwd);
}


int main(int argc, char *argv[]) {
    qing_cwd();

    int level = 2;
    float scale = 1.f * pow(2, level);
    string test_level = qing_int_2_string(level);
    string data_folder = "../test_new/";
    string out_folder = "../output/";
    string ply_name = "FRM_0116_pointcloud_A07A08.ply";    //range_grid_data
    string dsp_name = "erode_final_disp_l_" + test_level;
    string dsp_jpg_name = dsp_name + ".jpg";
    string dsp_txt_name = "rewrite_" + dsp_name + ".txt";
    string img_filename = "crop_imgL_" + test_level + ".jpg";
    string msk_filename = "crop_mskL_" + test_level + ".jpg";
    string stereo_filename = "stereo_A07A08.info";

    int width, height;
    vector<Vec3f> points(0);
    vector<Vec3f> colors(0);
    dsp_to_depth(data_folder, dsp_txt_name, dsp_jpg_name, img_filename, msk_filename, stereo_filename, scale, width, height,
                 points, colors);

    string src_file_name = data_folder + ply_name;
    string ply_file_name = "", out_file_name;

    PointcloudProcesser *processer = new PointcloudProcesser();
    if (NULL == processer) {
        cerr << "failed to start a point process pipeline..." << endl;
        return -1;
    }

    if ("" == ply_file_name)
        processer->load_range_grid_datas(points, colors, width, height);
    else
        processer->load_ply(ply_file_name);

    if (true == processer->get_is_loaded()) {
//        out_file_name = out_folder + "test_load_range_grid.ply";
//        processer->save_ply(out_file_name);
//        processer->show_pointcloud();

        processer->outliers_removal(1);
        out_file_name = out_folder + "clean_" + ply_name;
        processer->save_ply(out_file_name);
//
//        processer->down_sampling(10);
//
//        float radius = 15;
//        out_file_name = out_folder + "mls_" + qing_int_2_string((int)(radius)) + "_" + ply_name;
//        processer->mls_resampling(radius);
//        processer->correct_normal(0.f, 0.f, 0.f);
//        processer->save_ply(out_file_name);
    }

    return 1;
}


