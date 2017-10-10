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

    int is_range_grid = false;
    int test_level = 2;
    string test_lvl_str = qing_int_2_string(test_level);
    string data_folder = "../test_new/";
    string out_folder = "../output/";
    string dsp_name = "erode_final_disp_l_" + test_lvl_str;
    string dsp_jpg_name = dsp_name + ".jpg";
    string dsp_txt_name = "rewrite_" + dsp_name + ".txt";                     //must be re-written in correct size(width, height)
    string img_jpg_name = "crop_imgL_" + test_lvl_str + ".jpg";
    string msk_jpg_name = "crop_mskL_" + test_lvl_str + ".jpg";
    string stereo_info = "stereo_A07A08.info";

    vector<Vec3f> points(0);
    vector<Vec3b> colors(0);
    int width, height;
    dsp_to_depth(is_range_grid, data_folder, dsp_txt_name, dsp_jpg_name, img_jpg_name, msk_jpg_name, stereo_info, test_level, width, height, points, colors);


    string out_file_name;

    PointcloudProcesser *processer = new PointcloudProcesser();
    if (NULL == processer) {
        cerr << "failed to start a point process pipeline..." << endl;
        return -1;
    }

    if(is_range_grid)
        processer->load_range_grid_datas(points, colors, width, height);
    else
        processer->load_datas(points, colors);                                  //processer->load_ply(ply_file_name);

    string ply_name = "FRM_0116_pointcloud_A07A08.ply";
    if (true == processer->get_is_loaded()) {
//        out_file_name = out_folder + "test_load_range_grid.ply";
//        processer->save_ply(out_file_name);
//        processer->show_pointcloud();

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


