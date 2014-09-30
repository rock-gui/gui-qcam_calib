#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <iostream>
#include <stdexcept>
#include <QMutex>
#include <cv_detectors/chessboard.hpp>
#include <opensfm/structured_light/line_scanner.hpp>
#include <opensfm/opensfm.hpp>

#include <limits>
#include <assert.h>

#include "Items.hpp"

using namespace qcam_calib;

QVector<QPointF> convertToQt(const std::vector<cv::Point2f>&points1)
{
    QVector<QPointF> points2;
    std::vector<cv::Point2f>::const_iterator iter = points1.begin();
    for(;iter != points1.end();++iter)
        points2.push_back(QPointF(iter->x,iter->y));
    return points2;
}

std::vector<cv::Point2f> convertFromQt(const QVector<QPointF>&points1)
{
    std::vector<cv::Point2f> points2;
    QVector<QPointF>::const_iterator iter = points1.begin();
    for(;iter != points1.end();++iter)
        points2.push_back(cv::Point2f(iter->x(),iter->y()));
    return points2;
}

CameraParameterItem::CameraParameterItem(const QString &string):
    QCamCalibItem(string)
{
    setEditable(false);
    setColumnCount(2);

    setParameter("fx",0);
    setParameter("fy",0);
    setParameter("cx",0);
    setParameter("cy",0);
    setParameter("k1",0);
    setParameter("k2",0);
    setParameter("p1",0);
    setParameter("p2",0);
    setParameter("projection error",0);
    setParameter("pixel error",0);
};

CameraParameterItem::~CameraParameterItem()
{}

void CameraParameterItem::setParameter(const QString &name,double val)
{
    QModelIndexList list;
    if(model() && index().isValid())
        list = model()->match(index().child(0,0),Qt::DisplayRole,QVariant(name),1,Qt::MatchExactly);
    if(list.empty())
    {
        QList<QStandardItem*> items;
        items.append(new QCamCalibItem(name));
        items.back()->setEditable(false);
        items.append(new QCamCalibItem(val));
        items.back()->setEditable(false);
        appendRow(items);
    }
    else
    {
        QStandardItem *item = child(list.front().row(),1);
        if(item)
            item->setText(QString::number(val));
    }
}

double CameraParameterItem::getParameter(const QString &name)const
{
    QModelIndexList list;
    if(model() && index().isValid())
        list = model()->match(index().child(0,0),Qt::DisplayRole,QVariant(name),1,Qt::MatchExactly);
    if(!list.empty())
    {
        QStandardItem *item = child(list.front().row(),1);
        if(item)
            return item->data(Qt::EditRole).toDouble();
    }
    else
        throw std::runtime_error("cannot find parameter");
    return 0;
}

void CameraParameterItem::save(const QString &path)const
{
    cv::Mat k = cv::Mat::zeros(3,3,CV_64FC1);
    cv::Mat dist(4,1,CV_64FC1);
    k.at<double>(0,0) = getParameter("fx");
    k.at<double>(1,1) = getParameter("fy");
    k.at<double>(0,2) = getParameter("cx");
    k.at<double>(1,2) = getParameter("cy");
    k.at<double>(2,2) = 1.0;
    dist.at<double>(0) = getParameter("k1");
    dist.at<double>(1) = getParameter("k2");
    dist.at<double>(2) = getParameter("p1");
    dist.at<double>(3) = getParameter("p2");
    
    cv::FileStorage fs(path.toStdString(), cv::FileStorage::WRITE);
    time_t rawtime; time(&rawtime);
    fs << "calibrationDate" << asctime(localtime(&rawtime));
    fs << "cameraMatrix" << k << "distCoeffs" << dist;
    fs.release();
}

StructuredLightParameterItem::StructuredLightParameterItem(const QString &string):
    CameraParameterItem(string)
{
    setEditable(false);
    setColumnCount(2);
    removeRows(0,rowCount());
    setParameter("cam_fx",0);
    setParameter("cam_fy",0);
    setParameter("cam_cx",0);
    setParameter("cam_cy",0);
    setParameter("cam_k1",0);
    setParameter("cam_k2",0);
    setParameter("cam_p1",0);
    setParameter("cam_p2",0);
    setParameter("cam_projection error",0);
    setParameter("cam_pixel error",0);

    setParameter("proj_r1",0);
    setParameter("proj_r2",0);
    setParameter("proj_r3",0);
    setParameter("proj_t1",0);
    setParameter("proj_t2",0);
    setParameter("proj_t3",0);
    setParameter("proj_dist",0);

    setParameter("proj_error",0);
};

StructuredLightParameterItem::~StructuredLightParameterItem()
{}

void StructuredLightParameterItem::save(const QString &path)const
{
    cv::Mat k = cv::Mat::zeros(3,3,CV_64FC1);
    cv::Mat dist(4,1,CV_64FC1);
    k.at<double>(0,0) = getParameter("cam_fx");
    k.at<double>(1,1) = getParameter("cam_fy");
    k.at<double>(0,2) = getParameter("cam_cx");
    k.at<double>(1,2) = getParameter("cam_cy");
    k.at<double>(2,2) = 1.0;
    dist.at<double>(0) = getParameter("cam_k1");
    dist.at<double>(1) = getParameter("cam_k2");
    dist.at<double>(2) = getParameter("cam_p1");
    dist.at<double>(3) = getParameter("cam_p2");

    cv::Mat rvec(3,1,CV_64FC1);
    cv::Mat tvec(3,1,CV_64FC1);
    cv::Mat dist2(1,1,CV_64FC1);
    rvec.at<double>(0) = getParameter("proj_r1");
    rvec.at<double>(1) = getParameter("proj_r2");
    rvec.at<double>(2) = getParameter("proj_r3");
    tvec.at<double>(0) = getParameter("proj_t1");
    tvec.at<double>(1) = getParameter("proj_t2");
    tvec.at<double>(2) = getParameter("proj_t3");
    dist2.at<double>(0) = getParameter("proj_dist");

    cv::FileStorage fs(path.toStdString(), cv::FileStorage::WRITE);
    time_t rawtime; time(&rawtime);
    fs << "calibrationDate" << asctime(localtime(&rawtime));
    fs << "cameraMatrix" << k << "distCoeffs" << dist << "projT" << tvec << "projR" << rvec;
    fs << "projDistCoeffs" << dist2;
    fs.release();
}

CalibrationObj::CalibrationObj(int id, const QString &string):
    QCamCalibItem(string),
    id(id)
{
    setEditable(false);
    images = new QStandardItem("images");
    images->setEditable(false);
    appendRow(images);
};

int CalibrationObj::getId()
{
    return id;
}

ImageItem* CalibrationObj::getImageItem(int id)
{
    ImageItem *item = dynamic_cast<ImageItem*>(images->child(id,0));
    if(!item)
        throw std::runtime_error("cannot find image item");
    return item;
}

int CalibrationObj::countImages()const
{
    return images->rowCount();
}

QList<QImage> CalibrationObj::getRawImages()
{
    QList<QImage> imgs;
    for(int row=0;row < images->rowCount(); ++row)
    {
        ImageItem *item = dynamic_cast<ImageItem*>(images->child(row,0));
        imgs.push_back(item->getRawImage());
    }
    return imgs;
}

ImageItem* CalibrationObj::getImageItem(const QString &name)
{
    QModelIndexList list = model()->match(index(),Qt::DisplayRole,QVariant(name),1,Qt::MatchExactly);
    ImageItem *item = NULL;
    if(!list.empty())
        item = dynamic_cast<ImageItem*>(model()->itemFromIndex(list.front()));
    if(!item)
        throw std::runtime_error("Cannot find image item");
    return item;
}

ImageItem *CalibrationObj::addImage(const QString &name,const QImage &image)
{
    ImageItem *item = new ImageItem(name,image);
    QList<QStandardItem*> items;
    items.append(item);
    items.back()->setEditable(false);
    items.append(new QCamCalibItem("no chessboard"));
    items.back()->setEditable(false);
    images->appendRow(items);
    return item;
}

CameraItem::CameraItem(int id, const QString &string):
    CalibrationObj(id,string),
    parameter(NULL)
{
    setEditable(false);
    parameter = new CameraParameterItem("Parameter");
    appendRow(parameter);
};

bool CameraItem::isCalibrated()const
{
    return (parameter->getParameter("fx") > 0);
}

void CameraItem::saveParameter(const QString &path)const
{
    parameter->save(path);
}

int CameraItem::countChessboards()const
{
    int count = 0;
    for(int row=0;row < images->rowCount(); ++row)
    {
        ImageItem *item = dynamic_cast<ImageItem*>(images->child(row,0));
        if(item && !item->getChessboardCorners().empty())
            ++count;
    }
    return count;
}

void CameraItem::calibrate(int cols,int rows,float dx,float dy)
{
    std::vector<std::vector<cv::Point3f> > object_points;
    std::vector<std::vector<cv::Point2f> > image_points;
    cv::Size image_size;

    //generate chessboard points
    std::vector<cv::Point3f> points3f;
    for(int row=0;row < rows; ++row)
    {
        for(int col=0;col < cols; ++col)
            points3f.push_back(cv::Point3f(dx*col,dy*row,0));
    }

    //collect image points
    for(int row=0;row < images->rowCount(); ++row)
    {
        ImageItem *item = dynamic_cast<ImageItem*>(images->child(row,0));
        if(item && item->getChessboardCorners().size() ==(int) points3f.size())
        {
            image_size = cv::Size(item->getImage().width(),item->getImage().height());
            image_points.push_back(convertFromQt(item->getChessboardCorners()));
            object_points.push_back(points3f);
        }
    }
    if(object_points.size() < 1)
        throw std::runtime_error("not enough detected chessboards");

    cv::Mat k(3,3,CV_64FC1);
    cv::Mat dist(4,1,CV_64FC1);
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    double error = cv::calibrateCamera(object_points,image_points,
                                       image_size,k,dist,rvecs,tvecs,0,
                                       cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 50, DBL_EPSILON));

    //store parameters
    parameter->setParameter("fx",k.at<double>(0,0));
    parameter->setParameter("fy",k.at<double>(1,1));
    parameter->setParameter("cx",k.at<double>(0,2));
    parameter->setParameter("cy",k.at<double>(1,2));
    parameter->setParameter("k1",dist.at<double>(0));
    parameter->setParameter("k2",dist.at<double>(1));
    parameter->setParameter("p1",dist.at<double>(2));
    parameter->setParameter("p2",dist.at<double>(3));
    parameter->setParameter("projection error",error);
    parameter->setParameter("pixel error",sqrt(error/points3f.size()));

    //store positions of the chessboards

}

ImageItem::ImageItem(const QString &name, const QImage &image):
    QCamCalibItem(name),
    raw_image(image.copy()),
    image(raw_image),
    chessboard_rows(0),
    chessboard_cols(0)
{
}

ImageItem::~ImageItem()
{
}

const QVector<QPointF> &ImageItem::getChessboardCorners()const
{
    return chessboard;
}

bool ImageItem::hasChessboard()const
{
    return !chessboard.empty();
}

QVector<QPointF> ImageItem::findChessboard(const QImage &image,int cols ,int rows)
{
    std::vector<cv::Point2f> points;
    QImage img = image.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(img.height(), img.width(), CV_8UC3, img.bits(), img.bytesPerLine());
    cv::Mat gray;

    cv::cvtColor(mat,gray,cv::COLOR_RGB2GRAY);
    cv::findChessboardCorners(gray,cv::Size(cols,rows),points,cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
    return convertToQt(points);
}

QVector<QPointF> ImageItem::findChessboardFastX(const QImage &image,int cols ,int rows)
{
    std::vector<cv::Point2f> points;
    QImage img = image.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(img.height(), img.width(), CV_8UC3, img.bits(), img.bytesPerLine());
    cv::Mat gray;
    cv::cvtColor(mat,gray,cv::COLOR_RGB2GRAY);

    std::vector<cv::KeyPoint> key_points;
    cv_detectors::Chessboard::Parameters para;
    para.chessboard_size = cv::Size(cols,rows);
    para.strength = 5;
    para.resolution = M_PI*0.25;
    para.scale = int(log(std::min(gray.rows,gray.cols)/10-2)/log(2));
    for(;key_points.empty() && para.scale > 1;--para.scale)
    {
        cv_detectors::Chessboard detector(para);
        detector.detect(gray,key_points);
    }
    cv::KeyPoint::convert(key_points,points);
    return convertToQt(points);
}

bool ImageItem::findChessboard(int cols ,int rows)
{
    setChessboard(ImageItem::findChessboard(raw_image,cols,rows),cols,rows);
    if(chessboard.empty())
        return false;
    return true;
}

bool ImageItem::findChessboardFastX(int cols ,int rows)
{
    setChessboard(ImageItem::findChessboardFastX(raw_image,cols,rows),cols,rows);
    if(chessboard.empty())
        return false;
    return true;
}

void ImageItem::setChessboard(const QVector<QPointF> &chessboard,int cols,int rows)
{
    this->chessboard = chessboard;
    chessboard_rows = rows;
    chessboard_cols = cols;
    if(chessboard.empty())
        image = raw_image;
    else
    {
        QImage img = raw_image.convertToFormat(QImage::Format_RGB888);
        cv::Mat mat(img.height(), img.width(), CV_8UC3, img.bits(), img.bytesPerLine());
        std::vector<cv::Point2f> points = convertFromQt(chessboard);
        cv::drawChessboardCorners(mat,cv::Size(cols,rows),points,true);
        image = QImage(mat.data, mat.cols, mat.rows,mat.step,QImage::Format_RGB888).copy();
    }
    if(parent())
    {
        QStandardItem *item = parent()->child(row(),1);
        if(item)
        {
            if(!chessboard.empty())
                item->setText("ok");
            else
                item->setText("no chessboard");
        }
    }
}

QImage &ImageItem::getImage()
{
    return image;
}

QImage &ImageItem::getRawImage()
{
    return raw_image;
}

void ImageItem::setChessboardPlane(const QVector3D &plane, const QVector3D &normal)
{
    chessboard_plane = plane;
    chessboard_normal = normal;
}

QVector<QPointF> StructuredLightImageItem::findLaserLine(const Data &data)
{
    QImage img = data.image.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(img.height(), img.width(), CV_8UC3, img.bits(), img.bytesPerLine());
    cv::Mat gray;
    cv::cvtColor(mat,gray,cv::COLOR_RGB2GRAY);

    std::vector<cv::Point2f> points;
    std::vector<cv::KeyPoint> key_points;

    // generate mask
    // only points on the chessboard are from interest
    cv::Mat mask = cv::Mat::zeros(gray.rows,gray.cols,CV_8UC1);
    cv::Point pts[4];
    for(int i=0;i<4;++i)
        pts[i] = cv::Point(data.rect[i].x(),data.rect[i].y());
    cv::fillConvexPoly(mask,(cv::Point*)&pts,4,cv::Scalar(255,255,255,255));

    opensfm::structured_light::LaserLineDetector::Parameters para;
    para.laser_width_top = 3;
    para.laser_width_bottom = 4;
    para.min_segment_length = 5;
    para.max_gap_length = 5;
    para.min_seperation = 100;
    para.min_snr = 10;
    opensfm::structured_light::LaserLineDetector detector(para);
    detector.config(gray.cols,gray.rows);
    detector.detect(gray,key_points,mask);
    cv::KeyPoint::convert(key_points,points);

    //filter out points which are to far away from a polynomial
    //this is possible because the pattern lies on a flat surface
    cv::Mat dat,x,y;
    cv::Mat coeff(3,1,CV_64FC1);
    for(int i=0;i<4 && points.size() >= 30 ;++i)
    {
        dat = cv::Mat(points).reshape(1,points.size());
        // we have to clone otherwise checkVector(1) will return -1
        x = dat.col(0).clone();
        y = dat.col(1).clone();
        cv::polyfit(x,y,coeff,2);

        std::vector<cv::Point2f> points2;
        std::vector<cv::Point2f>::iterator iter = points.begin();
        for(;iter != points.end();++iter)
            if(std::abs(coeff.at<float>(2)*iter->x*iter->x+coeff.at<float>(1)*iter->x+coeff.at<float>(0)-iter->y) < 4.0-0.95*i)
                points2.push_back(*iter);
        points.swap(points2);
    }

    if(points.size() < 30)
        points.clear();
    return convertToQt(points);
}

QVector<QVector3D> StructuredLightImageItem::get3DPoints(const QVector<QPointF> &laser_line,const QVector3D &chessboard_plane,const QVector3D &chessboard_normal)
{
    QVector<QVector3D> points3d;

    //crossing

    return points3d;
}

StructuredLightImageItem::StructuredLightImageItem(const QString &name, const QImage &image):
    ImageItem(name,image)
{

}

void StructuredLightImageItem::setLaserLine(const QVector<QPointF> &laser_line)
{
    this->laser_line = laser_line;
    if(laser_line.empty())
        setChessboard(chessboard,chessboard_cols,chessboard_rows);
    else
    {
        QImage img = raw_image.convertToFormat(QImage::Format_RGB888);
        cv::Mat mat(img.height(), img.width(), CV_8UC3, img.bits(), img.bytesPerLine());
        std::vector<cv::Point2f> points = convertFromQt(laser_line);
        std::vector<cv::KeyPoint> key_points;
        cv::KeyPoint::convert(points,key_points);
        cv::drawKeypoints(mat,key_points,mat);
        std::vector<cv::Point2f> chess_points = convertFromQt(chessboard);
        cv::drawChessboardCorners(mat,cv::Size(chessboard_cols,chessboard_rows),chess_points,true);
        image = QImage(mat.data, mat.cols, mat.rows,mat.step,QImage::Format_RGB888).copy();
    }

    if(parent())
    {
        QStandardItem *item = parent()->child(row(),1);
        if(item)
        {
            if(!laser_line.empty())
                item->setText("ok");
            else if(chessboard.empty())
                item->setText("no chessboard");
            else
                item->setText("no laser points");
        }
    }
}


const QVector<QPointF> &StructuredLightImageItem::getLaserLine()const
{
    return laser_line;
}

StructuredLightItem::StructuredLightItem(int id, const QString &string):
    CameraItem(id,string)
{
    parameter = new StructuredLightParameterItem("Parameter");
    removeRow(1);
    appendRow(parameter);
}

StructuredLightImageItem *StructuredLightItem::addImage(const QString &name,const QImage &image)
{
    StructuredLightImageItem *item = new StructuredLightImageItem(name,image);
    QList<QStandardItem*> items;
    items.append(item);
    items.back()->setEditable(false);
    items.append(new QCamCalibItem("no chessboard"));
    items.back()->setEditable(false);
    images->appendRow(items);
    return item;
}

bool StructuredLightItem::isCalibrated()const
{
    return (parameter->getParameter("proj_r1") != 0 && parameter->getParameter("proj_r1") != 0);
}

int StructuredLightItem::countLaserLines()const
{
    int count = 0;
    for(int row=0;row < images->rowCount(); ++row)
    {
        StructuredLightImageItem *item = dynamic_cast<StructuredLightImageItem*>(images->child(row,0));
        if(item && !item->getLaserLine().empty())
            ++count;
    }
    return count;
}

StructuredLightImageItem* StructuredLightItem::getImageItem(int id)
{
    StructuredLightImageItem *item = dynamic_cast<StructuredLightImageItem*>(images->child(id,0));
    if(!item)
        throw std::runtime_error("cannot find image item");
    return item;
}

StructuredLightImageItem* StructuredLightItem::getImageItem(const QString &name)
{
    QModelIndexList list = model()->match(index(),Qt::DisplayRole,QVariant(name),1,Qt::MatchExactly);
    StructuredLightImageItem *item = NULL;
    if(!list.empty())
        item = dynamic_cast<StructuredLightImageItem*>(model()->itemFromIndex(list.front()));
    if(!item)
        throw std::runtime_error("Cannot find image item");
    return item;
}

void StructuredLightItem::calibrate(int cols,int rows,float dx,float dy)
{
    std::vector<std::vector<cv::Point3f> > object_points;
    std::vector<std::vector<cv::Point2f> > image_points;
    std::vector<StructuredLightImageItem*> items;
    cv::Size image_size;

    //generate chessboard points
    std::vector<cv::Point3f> points3f;
    for(int row=0;row < rows; ++row)
    {
        for(int col=0;col < cols; ++col)
            points3f.push_back(cv::Point3f(dx*col,dy*row,0));
    }

    //collect image points
    for(int row=0;row < images->rowCount(); ++row)
    {
        StructuredLightImageItem *item = dynamic_cast<StructuredLightImageItem*>(images->child(row,0));
        if(item && item->getChessboardCorners().size() ==(int) points3f.size())
        {
            image_size = cv::Size(item->getImage().width(),item->getImage().height());
            image_points.push_back(convertFromQt(item->getChessboardCorners()));
            object_points.push_back(points3f);
            items.push_back(item);
        }
    }
    if(object_points.size() < 1)
        throw std::runtime_error("not enough detected chessboards");

    cv::Mat k(3,3,CV_64FC1);
    cv::Mat dist(4,1,CV_64FC1);
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    double error = cv::calibrateCamera(object_points,image_points,
                                       image_size,k,dist,rvecs,tvecs,0,
                                       cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 50, DBL_EPSILON));
    parameter->setParameter("cam_fx",k.at<double>(0,0));
    parameter->setParameter("cam_fy",k.at<double>(1,1));
    parameter->setParameter("cam_cx",k.at<double>(0,2));
    parameter->setParameter("cam_cy",k.at<double>(1,2));
    parameter->setParameter("cam_k1",dist.at<double>(0));
    parameter->setParameter("cam_k2",dist.at<double>(1));
    parameter->setParameter("cam_p1",dist.at<double>(2));
    parameter->setParameter("cam_p2",dist.at<double>(3));
    parameter->setParameter("cam_projection error",error);
    parameter->setParameter("cam_pixel error",sqrt(error/points3f.size()));

    //collect image points and calc their 3d coordinate
    std::vector<cv::Point3f> laser_points;
    for(int row=0;row < (int)items.size(); ++row)
    {
        StructuredLightImageItem *item = items[row];
        cv::Mat r = rvecs[row];
        cv::Mat t = tvecs[row]*0.001;  //convert into meter
        cv::Rodrigues(r,r);
        assert(r.type() == CV_64FC1);
        assert(t.type() == CV_64FC1);
        QVector3D plane(t.at<double>(0),t.at<double>(1),t.at<double>(2));        //offset
        QVector3D normal(r.at<double>(0,2),r.at<double>(1,2),r.at<double>(2,2)); //normal R*[0;0;1]
        item->setChessboardPlane(plane,normal);

        QVector3D v1(r.at<double>(0,0),r.at<double>(1,0),r.at<double>(2,0)); // plane vector R*[1;0;0]
        QVector3D v2(r.at<double>(0,1),r.at<double>(1,1),r.at<double>(2,1)); // plane vector R*[0;1;0]

        cv::Mat x(3,3,CV_64FC1);
        cv::Mat y(3,1,CV_64FC1);
        cv::Mat dst;
        x.at<double>(0,0) = v1.x();
        x.at<double>(1,0) = v1.y();
        x.at<double>(2,0) = v1.z();
        x.at<double>(0,1) = v2.x();
        x.at<double>(1,1) = v2.y();
        x.at<double>(2,1) = v2.z();

        y.at<double>(0) = -plane.x();
        y.at<double>(1) = -plane.y();
        y.at<double>(2) = -plane.z();

        cv::Mat v3(3,1,CV_64FC1);
        const QVector<QPointF> laser_line = item->getLaserLine();
        QVector<QPointF>::const_iterator iter = laser_line.begin();

        std::vector<cv::Point3f> points3d;
        cv::Mat pts(1,2,CV_64FC1);
        cv::Mat out;
        for(;iter != laser_line.end() ;++iter)
        {
            // TODO use matrices
            // undistort laser points
            pts.at<double>(0) = iter->x();
            pts.at<double>(1) = iter->y();
            cv::undistortPoints(pts.reshape(2,1),out,k,dist);
            out = out.reshape(1,1);

            // calc intersection between chessboard plane and camera ray
            v3.at<double>(0) = out.at<double>(0);
            v3.at<double>(1) = out.at<double>(1);
            v3.at<double>(2) = 1.0;  // focal length is one for unified camera
            v3 = v3/cv::norm(v3);

            x.at<double>(0,2) = -v3.at<double>(0);
            x.at<double>(1,2) = -v3.at<double>(1);
            x.at<double>(2,2) = -v3.at<double>(2);

            cv::solve(x,y,dst);
            QVector3D result = v1*dst.at<double>(0)+v2*dst.at<double>(1)+plane;
            laser_points.push_back(cv::Point3f(result.x(),result.y(),result.z()));
        }
    }

    cv::Mat coeffs,rvec,tvec;

    opensfm::RANSAC::Param param(1e-3,1e-2,200,1000);
    error = opensfm::structured_light::LineScanner::calibrateLaserLine(laser_points,coeffs,rvec,tvec,param);
    parameter->setParameter("proj_t1",tvec.at<double>(0));
    parameter->setParameter("proj_t2",tvec.at<double>(1));
    parameter->setParameter("proj_t3",tvec.at<double>(2));
    parameter->setParameter("proj_r1",rvec.at<double>(0));
    parameter->setParameter("proj_r2",rvec.at<double>(1));
    parameter->setParameter("proj_r3",rvec.at<double>(2));
    parameter->setParameter("proj_dist",coeffs.at<double>(0));
    parameter->setParameter("proj_error",error);
}
