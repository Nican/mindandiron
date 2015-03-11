#include "pointcloud.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QDialog>

#include <QLineEdit>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QCheckBox>

PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QWidget(parent), viewer(new pcl::visualization::PCLVisualizer ("viewer", false))
{    

    //
    //  CRreate PCL related widgets
    //
    qvtkWidget = new QVTKWidget(this);
    qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());

    qvtkWidget->update ();
    viewer->addPointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(), "cloud");
    viewer->resetCamera ();
    qvtkWidget->update ();

    //
    //  Settings button
    //
    auto button = new QPushButton("Settings", this);

    // 
    //  Popup dialog for settings
    //
    mDialog = new QDialog(this);
    mDialogLayout = new QFormLayout;
    mDialog->setLayout(mDialogLayout);


    auto *layout = new QVBoxLayout;
    layout->addWidget(qvtkWidget);
    layout->addWidget(button);

    setLayout(layout);
    show();

    connect(button, SIGNAL(released()), this, SLOT(OpenSettings()));
}



void PointCloudWidget::OpenSettings()
{
    mDialog->show();
    mDialog->raise();
    mDialog->activateWindow();
}

GrowingRegionPointCloudWidget::GrowingRegionPointCloudWidget(QWidget *parent) 
: PointCloudWidget(parent)
{
    numberOfNeighbours = new QSpinBox(this);
    numberOfNeighbours->setValue(segmenter.numberOfNeighbours);
    mDialogLayout->addRow(tr("&Number Of Neighbours:"), numberOfNeighbours);
    connect(numberOfNeighbours, SIGNAL(valueChanged(int)), this, SLOT(UpdateSettings()));

    smoothnessThreshold = new QDoubleSpinBox(this);
    smoothnessThreshold->setValue(segmenter.smoothnessThreshold);
    mDialogLayout->addRow(tr("&Smoothness Threshold:"), smoothnessThreshold);
    connect(smoothnessThreshold, SIGNAL(valueChanged(double)), this, SLOT(UpdateSettings()));

    curvatureThreshold = new QDoubleSpinBox(this);
    curvatureThreshold->setValue(segmenter.curvatureThreshold);
    mDialogLayout->addRow(tr("&Curvature Threshold:"), curvatureThreshold);
    connect(curvatureThreshold, SIGNAL(valueChanged(double)), this, SLOT(UpdateSettings()));
}

void GrowingRegionPointCloudWidget::ReceivePointCloud(Robot::PointCloud::Ptr &cloud)
{
    if(segmenter.Update(cloud))
    {
        if(!segmenter.lastProccessed)
        {
            std::cout << "Update failed????\n";
            return;
        }

        //std::cout << "Updating the cloud view ("<< segmenter.lastProccessed->points.size() <<")!\n";

        viewer->updatePointCloud (segmenter.lastProccessed, "cloud");
        viewer->resetCamera ();
        qvtkWidget->update ();
    }
}

void GrowingRegionPointCloudWidget::UpdateSettings()
{
    segmenter.numberOfNeighbours = numberOfNeighbours->value();
    segmenter.smoothnessThreshold = smoothnessThreshold->value();
    segmenter.curvatureThreshold = curvatureThreshold->value();
}


PlaneSegmentCloudWidget::PlaneSegmentCloudWidget(QWidget *parent)
: PointCloudWidget(parent)
{
    threshold = new QDoubleSpinBox(this);
    threshold->setValue(segmenter.threshold);
    mDialogLayout->addRow(tr("&Theshold:"), threshold);
    connect(threshold, SIGNAL(valueChanged(double)), this, SLOT(UpdateSettings()));

    maxDepthChangeFactor = new QDoubleSpinBox(this);
    maxDepthChangeFactor->setValue(segmenter.maxDepthChangeFactor);
    mDialogLayout->addRow(tr("&Max Deapth Change:"), maxDepthChangeFactor);
    connect(maxDepthChangeFactor, SIGNAL(valueChanged(double)), this, SLOT(UpdateSettings()));

    normalSmoothingSize = new QDoubleSpinBox(this);
    normalSmoothingSize->setValue(segmenter.normalSmoothingSize);
    mDialogLayout->addRow(tr("&Normal Smoothing Size:"), normalSmoothingSize);
    connect(normalSmoothingSize, SIGNAL(valueChanged(double)), this, SLOT(UpdateSettings()));

    minInliers = new QSpinBox(this);
    minInliers->setValue(segmenter.minInliers);
    mDialogLayout->addRow(tr("&Min inliners:"), minInliers);
    connect(minInliers, SIGNAL(valueChanged(int)), this, SLOT(UpdateSettings()));

    angularThreshold = new QDoubleSpinBox(this);
    angularThreshold->setValue(segmenter.angularThreshold);
    mDialogLayout->addRow(tr("&Angular Threshold:"), angularThreshold);
    connect(angularThreshold, SIGNAL(valueChanged(double)), this, SLOT(UpdateSettings()));

    distanceThreshold = new QDoubleSpinBox(this);
    distanceThreshold->setValue(segmenter.distanceThreshold);
    mDialogLayout->addRow(tr("&Distance thershold:"), distanceThreshold);
    connect(distanceThreshold, SIGNAL(valueChanged(double)), this, SLOT(UpdateSettings()));

    refine = new QCheckBox(this); 
    refine->setCheckState(segmenter.refine ? Qt::Checked : Qt::Unchecked );
    mDialogLayout->addRow(tr("&Refine:"), refine);
    connect(refine, SIGNAL(stateChanged(int)), this, SLOT(UpdateSettings()));

}

void PlaneSegmentCloudWidget::UpdateSettings()
{
    segmenter.threshold = threshold->value();
    segmenter.maxDepthChangeFactor = maxDepthChangeFactor->value();
    segmenter.normalSmoothingSize = normalSmoothingSize->value();

    segmenter.minInliers = minInliers->value();
    segmenter.angularThreshold = angularThreshold->value();
    segmenter.distanceThreshold = distanceThreshold->value();

    segmenter.refine = refine->checkState() == Qt::Checked;
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

            pcl::approximatePolygon (regions[i], approx_polygon, segmenter.threshold, false);
            approx_contour->points = approx_polygon.getContour ();
            //std::cout << "polygon: " << contour->size () << " -> " << approx_contour->size () << std::endl;
            pcl::PointCloud<PointT>::ConstPtr approx_contour_const = approx_contour;

            //        sprintf (name, "approx_plane_%02d", int (i));
            //        viewer.addPolygon<PointT> (approx_contour_const, 0.5 * red[i], 0.5 * grn[i], 0.5 * blu[i], name);
            for (unsigned idx = 0; idx < approx_contour->points.size (); ++idx)
            {
                //sprintf (name, "approx_plane_%02d_%03d", int (i), int(idx));
                //viewer->addLine (approx_contour->points [idx], approx_contour->points [(idx+1)%approx_contour->points.size ()], 0.5 * red[i], 0.5 * grn[i], 0.5 * blu[i], name);
            }
        }

        //viewer->updatePointCloud (segmentedCloud, "cloud");
        //viewer->resetCamera ();
        qvtkWidget->update ();

    }
}