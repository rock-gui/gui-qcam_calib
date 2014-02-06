#include "ImageView.hpp"

#include<QGraphicsPixmapItem>

using namespace qcam_calib;

ImageView::ImageView(QWidget *parent):
    QGraphicsView(parent),
    welcome(NULL)
{
    QGraphicsScene *scene = new QGraphicsScene(this);
    pixmap_item = new QGraphicsPixmapItem();
    scene->addItem(pixmap_item);
    scene->setBackgroundBrush(QColor(120,120,120));
    setScene(scene);

    welcome = new QGraphicsTextItem("Camera calibration:\n\n1.) load images\n2.) find cessboard corners\n3.) calibrate camera\n4.) save parameter");
    scene->addItem(welcome);
}

ImageView::~ImageView()
{
}

void ImageView::displayImage(const QImage &image)
{
    if(welcome)
    {
        scene()->removeItem(welcome);
        welcome = NULL; //deleted by scene
    }
    QPixmap pixmap = QPixmap::fromImage(image);
    pixmap_item->setPixmap(pixmap);
    fitImage();
}

void ImageView::fitImage()
{
    fitInView(scene()->sceneRect(),Qt::KeepAspectRatio);
}

void ImageView::resizeEvent (QResizeEvent * event)
{
    fitImage();
}
