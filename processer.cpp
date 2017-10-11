#include "processer.h"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../Qing/qing_macros.h"

PointcloudProcesser::PointcloudProcesser() : m_is_loaded(false), m_cloud(new QingPointcloud()),
                                             m_cloud_filtered(new QingPointcloud()) {
}

PointcloudProcesser::~PointcloudProcesser() {

}

void PointcloudProcesser::load_ply(const string &filename) {
    m_cloud->clear();
    m_cloud_filtered->clear();

    cout << "loading " << filename << endl;
    pcl::PLYReader reader;
    reader.read(filename, *m_cloud);

    int size = m_cloud->width * m_cloud->height;
    if (0 == size) { m_is_loaded = false; }
    else { m_is_loaded = true; }

    cout << "Pointcloud size: " << m_cloud->width << " * " << m_cloud->height
         << " data points (" << pcl::getFieldsList(*m_cloud) << "). " << endl;
}

void PointcloudProcesser::save_ply(const string &outfile) {
    m_cloud_filtered = m_cloud->makeShared();
    pcl::PLYWriter writer;
    writer.write(outfile, *m_cloud_filtered);
    cout << "saving " << outfile << endl;
}

void PointcloudProcesser::load_range_grid_datas(const vector<cv::Vec3f> &points, const vector<cv::Vec3b> &colors,
                                                const int &height, const int &width) {
    m_cloud->clear();
    m_cloud_filtered->clear();

    int size = height * width;

    m_cloud->resize(size);
    m_cloud->height = height;
    m_cloud->width = width;
    m_cloud->is_dense = false;

    for (int i = 0; i < size; ++i) {
        if (PT_UNDEFINED != points[i].val[0]) {
            m_cloud->points[i].x = points[i].val[0] / 1000;
            m_cloud->points[i].y = points[i].val[1] / 1000;
            m_cloud->points[i].z = points[i].val[2] / 1000;  //mm -> m
            m_cloud->points[i].b = colors[i].val[0];
            m_cloud->points[i].g = colors[i].val[1];
            m_cloud->points[i].r = colors[i].val[2];
        } else {
            m_cloud->points[i].x = NAN; //std::numeric_limits<float>::quiet_NaN();  //or all zeros
            m_cloud->points[i].y = NAN; //std::numeric_limits<float>::quiet_NaN();
            m_cloud->points[i].z = NAN; //std::numeric_limits<float>::quiet_NaN();
        }
    }

    size = m_cloud->width * m_cloud->height;
    if (0 == size) {
        m_is_loaded = false;
    } else {
        m_is_loaded = true;
    }
    cout << "end of loading range grid datas..." << endl;

//    string out_filename = "../output/test_load_range_grid_data.ply";
//    save_ply(out_filename);
//    exit(1);
}

void PointcloudProcesser::load_datas(const vector<cv::Vec3f> &points, const vector<cv::Vec3b> &colors) {
    m_cloud->clear();
    m_cloud_filtered->clear();

    int size = colors.size();
    m_cloud->width = size;
    m_cloud->height = 1;
    for (int i = 0; i < size; ++i) {
        QingPoint pt;
        pt.x = points[i].val[0];
        pt.y = points[i].val[1];
        pt.z = points[i].val[2];
        pt.b = colors[i].val[0];
        pt.g = colors[i].val[1];
        pt.r = colors[i].val[2];

        m_cloud->points.push_back(pt);
    }
    size = m_cloud->width * m_cloud->height;
    if (0 == size) { m_is_loaded = false; }
    else { m_is_loaded = true; }
    cout << "Pointcloud size: " << m_cloud->width << " * " << m_cloud->height
         << " data points (" << pcl::getFieldsList(*m_cloud) << "). " << endl;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> qing_rgb_vis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {

    cout << "qing_rgb_vis." << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return viewer;
}

void PointcloudProcesser::show_pointcloud() {
    // pcl::visualization::CloudViewer viewer("Viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    to_point_xyzrgb(show_cloud_ptr);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = qing_rgb_vis(show_cloud_ptr);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    cout << "end of show pointcloud" << endl;
}


void PointcloudProcesser::outliers_removal(int times/* = 1*/) {
    int numk = 60;
    double stddev = 1.0;

    m_cloud_filtered = m_cloud->makeShared();
    while (times--) {
        pcl::StatisticalOutlierRemoval<QingPoint> sor;
        cout << "before outliers removal: " << (m_cloud_filtered->points.size()) << " points." << std::endl;
        sor.setInputCloud(m_cloud);
        sor.setMeanK(numk);
        sor.setStddevMulThresh(stddev);
        sor.filter(*m_cloud_filtered);

        cout << "after: " << (m_cloud_filtered->points.size()) << " points." << std::endl;
        m_cloud = m_cloud_filtered->makeShared();
        numk += 20;
    }
}

void PointcloudProcesser::down_sampling(int scale /*=4*/) {
    m_cloud_filtered = m_cloud->makeShared();
    int src_size = m_cloud->points.size();
    int dst_size = m_cloud_filtered->points.size();

    float leafsize = 0.5f;
    float stepsize = 0.05f;

    pcl::VoxelGrid<QingPoint> sor;
    sor.setInputCloud(m_cloud);
    while ((dst_size * scale) > src_size) {
        sor.setLeafSize(leafsize, leafsize, leafsize);
        sor.filter(*m_cloud_filtered);
        dst_size = m_cloud_filtered->points.size();
        cout << "after down-sampling, pointcloud size: " << dst_size << " <-- src_size = "
             << src_size << "\tleafsize = " << leafsize << endl;
        leafsize += stepsize;
    }

    m_cloud = m_cloud_filtered->makeShared();
}

void PointcloudProcesser::mls_resampling(float radius) {
    cout << "mls-resampling radius = " << radius << endl;
    pcl::search::KdTree<QingPoint>::Ptr tree(new pcl::search::KdTree<QingPoint>());
    pcl::MovingLeastSquares<QingPoint, QingPoint> mls;

    mls.setComputeNormals(true);
    mls.setInputCloud(m_cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.process(*m_cloud_filtered);

    cout << "msl_resample: " << m_cloud_filtered->points.size() << " <-- " << m_cloud->points.size() << endl;
    m_cloud = m_cloud_filtered->makeShared();

}

void PointcloudProcesser::correct_normal(float vx, float vy, float vz) {
    m_cloud_filtered = m_cloud->makeShared();

    int size = m_cloud_filtered->points.size();
    float v_px, v_py, v_pz, dot_val;
    float nx, ny, nz;

    for (int i = 0; i < size; ++i) {
        QingPoint pt = m_cloud_filtered->points[i];

        v_px = vx - pt.x;
        v_py = vy - pt.y;
        v_pz = vz - pt.z;
        nx = pt.normal[0];
        ny = pt.normal[1];
        nz = pt.normal[2];

        dot_val = v_px * nx + v_py * ny + v_pz * nz;
        if (dot_val < 0) {
            m_cloud_filtered->points[i].normal[0] = -nx;
            m_cloud_filtered->points[i].normal[1] = -ny;
            m_cloud_filtered->points[i].normal[2] = -nz;
        }
    }
    m_cloud = m_cloud_filtered->makeShared();
}

void PointcloudProcesser::to_points_and_colors(vector<qing_vec3f> &points, vector<qing_vec3f> &colors) {
    int size = m_cloud->points.size();
    points.reserve(size);
    colors.reserve(size);
    for (int i = 0; i < size; ++i) {
        QingPoint pt = m_cloud->points[i];
        qing_vec3f xyz, rgb;
        xyz.val[0] = pt.x;
        xyz.val[1] = pt.y;
        xyz.val[2] = pt.z;
        rgb.val[0] = pt.r;
        rgb.val[1] = pt.g;
        rgb.val[2] = pt.b;
        points.push_back(xyz);
        colors.push_back(rgb);
    }
    cout << "pointcloud to points and colors: " << points.size() << '\t' << colors.size() << endl;

}

void PointcloudProcesser::to_point_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst_pointcloud_ptr) {
    int size = m_cloud->points.size();
    if (0 == size) {
        cerr << "there is no points." << endl;
        return;
    }

    cout << size << " points in m_cloud" << endl;
    dst_pointcloud_ptr->points.reserve(2 * size);
    for (int i = 0; i < size; ++i) {
        QingPoint src_pt = (*m_cloud)[i];
        pcl::PointXYZRGB dst_pt;
        dst_pt.x = src_pt.x;
        dst_pt.y = src_pt.y;
        dst_pt.z = src_pt.z;
        dst_pt.rgb = src_pt.rgb;

        dst_pointcloud_ptr->points.push_back(dst_pt);
    }
    cout << "end of to_point_xyzrgb: " << dst_pointcloud_ptr->points.size() << " points.." << endl;

//check
//    pcl::PLYWriter writer;
//    string dst_file_name = "../output/check_to_point_xyzrgb.ply";
//    writer.write(dst_file_name, *dst_pointcloud_ptr);
//    cout << "saving " << dst_file_name << endl;
}
