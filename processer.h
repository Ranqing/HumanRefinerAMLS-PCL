#ifndef PROCESSER_H
#define PROCESSER_H

#include <string>
using namespace std;

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>

typedef struct _vec3f{
    float val[3];
}qing_vec3f;


//only process with point with xyz,rgb
typedef pcl::PointXYZRGBNormal QingPoint;
typedef pcl::PointCloud<QingPoint> QingPointcloud;
typedef pcl::PointCloud<QingPoint>::Ptr QingPointcloudPtr;

class PointcloudProcesser {
public:
    PointcloudProcesser();
    ~PointcloudProcesser();

    void load_ply(const string infile);
    void save_ply(const string outfile);

    void outliers_removal(int times = 1);
    void down_sampling(int scale = 4);
    void mls_resampling(float radius = 10);
    void correct_normal(float vx = 0.f, float vy = 0.f, float vz = 0.f);


    void to_points_and_colors(vector<qing_vec3f>& points, vector<qing_vec3f>& colors);

    bool get_is_loaded() { return m_is_loaded; }

private:
    bool m_is_loaded;
    QingPointcloudPtr m_cloud;
    QingPointcloudPtr m_cloud_filtered;
};



#endif