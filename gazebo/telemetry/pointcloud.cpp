#include "pointcloud.h"
#include <QHBoxLayout>

PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QWidget(parent), viewer(new pcl::visualization::PCLVisualizer ("viewer", false))
{    

    qvtkWidget = new QVTKWidget(this);
    qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());

    qvtkWidget->update ();
    viewer->addPointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(), "cloud");
    viewer->resetCamera ();
    qvtkWidget->update ();

    QHBoxLayout *layout = new QHBoxLayout;
    layout->addWidget(qvtkWidget);

    setLayout(layout);
    show();
}




void GrowingRegionPointCloudWidget::ReceivePointCloud(Robot::PointCloud::Ptr &cloud)
{
    if(segmenter.Update(cloud))
    {
        std::cout << "Updating the cloud view ("<< segmenter.lastProccessed->points.size() <<")!\n";

        viewer->updatePointCloud (segmenter.lastProccessed, "cloud");
        viewer->resetCamera ();
        qvtkWidget->update ();
    }
}



void PlaneSegmentCloudWidget::ReceivePointCloud(Robot::PointCloud::Ptr &cloud)
{
    if(segmenter.Update(cloud))
    {
        auto& regions = segmenter.lastProccessed;

        unsigned char red [6] = {255,   0,   0, 255, 255,   0};
        unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
        unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
        auto contour = boost::make_shared<pcl::PointCloud<PointT>>();
        auto approx_contour = boost::make_shared<pcl::PointCloud<PointT>>();
        char name[1024];

        viewer->removeAllPointClouds (0);
        viewer->removeAllShapes (0);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (cloud, 0, 255, 0);
        viewer->addPointCloud<PointT> (cloud, single_color, "cloud");

        pcl::PlanarPolygon<PointT> approx_polygon;
        //Draw Visualization
        for (size_t i = 0; i < regions.size (); i++)
        {
            Eigen::Vector3f centroid = regions[i].getCentroid ();
            Eigen::Vector4f model = regions[i].getCoefficients ();
            pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
            pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                             centroid[1] + (0.5f * model[1]),
                                             centroid[2] + (0.5f * model[2]));
            sprintf (name, "normal_%d", unsigned (i));
            viewer->addArrow (pt2, pt1, 1.0, 0, 0, std::string (name));

            contour->points = regions[i].getContour ();        
            sprintf (name, "plane_%02d", int (i));
            pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, red[i], grn[i], blu[i]);
            viewer->addPointCloud (contour, color, name);

            pcl::approximatePolygon (regions[i], approx_polygon, segmenter.threshold_, false);
            approx_contour->points = approx_polygon.getContour ();
            //std::cout << "polygon: " << contour->size () << " -> " << approx_contour->size () << std::endl;
            pcl::PointCloud<PointT>::ConstPtr approx_contour_const = approx_contour;

            //        sprintf (name, "approx_plane_%02d", int (i));
            //        viewer.addPolygon<PointT> (approx_contour_const, 0.5 * red[i], 0.5 * grn[i], 0.5 * blu[i], name);
            for (unsigned idx = 0; idx < approx_contour->points.size (); ++idx)
            {
                sprintf (name, "approx_plane_%02d_%03d", int (i), int(idx));
                viewer->addLine (approx_contour->points [idx], approx_contour->points [(idx+1)%approx_contour->points.size ()], 0.5 * red[i], 0.5 * grn[i], 0.5 * blu[i], name);
            }
        }

        //viewer->updatePointCloud (segmentedCloud, "cloud");
        //viewer->resetCamera ();
        qvtkWidget->update ();

    }
}