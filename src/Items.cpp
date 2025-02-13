#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

#include "Items.hpp"
#include <QMutex>
#include <stdexcept>

using namespace qcam_calib;

QVector<QPointF> convertToQt(const std::vector<cv::Point2f>& points1)
{
    QVector<QPointF> points2;
    std::vector<cv::Point2f>::const_iterator iter = points1.begin();
    for (; iter != points1.end(); ++iter)
        points2.push_back(QPointF(iter->x, iter->y));
    return points2;
}

std::vector<cv::Point2f> convertFromQt(const QVector<QPointF>& points1)
{
    std::vector<cv::Point2f> points2;
    QVector<QPointF>::const_iterator iter = points1.begin();
    for (; iter != points1.end(); ++iter)
        points2.push_back(cv::Point2f(iter->x(), iter->y()));
    return points2;
}

CameraParameterItem::CameraParameterItem(const QString& string)
    : QCamCalibItem(string)
{
    setEditable(false);
    setColumnCount(2);

    setParameter("fx", 0);
    setParameter("fy", 0);
    setParameter("cx", 0);
    setParameter("cy", 0);
    setParameter("k1", 0);
    setParameter("k2", 0);
    setParameter("p1", 0);
    setParameter("p2", 0);
    setParameter("projection error", 0);
    setParameter("pixel error", 0);
    // By using the ValidROI from the image with blackbars, one can calculate the
    // percentage of the image that is cropped to maintain the aspect ratio after
    // the undistort.
    setParameter("Horizontal FOV percentage after undistort", 0);
};

void CameraParameterItem::setParameter(const QString& name, double val)
{
    QModelIndexList list;
    if (model() && index().isValid())
        list = model()->match(index().child(0, 0),
            Qt::DisplayRole,
            QVariant(name),
            1,
            Qt::MatchExactly);
    if (list.empty()) {
        QList<QStandardItem*> items;
        items.append(new QCamCalibItem(name));
        items.back()->setEditable(false);
        items.append(new QCamCalibItem(val));
        items.back()->setEditable(false);
        appendRow(items);
    }
    else {
        QStandardItem* item = child(list.front().row(), 1);
        if (item)
            item->setText(QString::number(val));
    }
}

double CameraParameterItem::getParameter(const QString& name) const
{
    QModelIndexList list;
    if (model() && index().isValid())
        list = model()->match(index().child(0, 0),
            Qt::DisplayRole,
            QVariant(name),
            1,
            Qt::MatchExactly);
    if (!list.empty()) {
        QStandardItem* item = child(list.front().row(), 1);
        if (item)
            return item->data(Qt::EditRole).toDouble();
    }
    else
        throw std::runtime_error("cannot find parameter");
    return 0;
}

void CameraParameterItem::save(const QString& path) const
{
    cv::Mat k = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Mat dist(4, 1, CV_64FC1);
    k.at<double>(0, 0) = getParameter("fx");
    k.at<double>(1, 1) = getParameter("fy");
    k.at<double>(0, 2) = getParameter("cx");
    k.at<double>(1, 2) = getParameter("cy");
    k.at<double>(2, 2) = 1.0;
    dist.at<double>(0) = getParameter("k1");
    dist.at<double>(1) = getParameter("k2");
    dist.at<double>(2) = getParameter("p1");
    dist.at<double>(3) = getParameter("p2");

    cv::FileStorage fs(path.toStdString(), cv::FileStorage::WRITE);
    time_t rawtime;
    time(&rawtime);
    fs << "calibrationDate" << asctime(localtime(&rawtime));
    fs << "cameraMatrix" << k << "distCoeffs" << dist;
    fs.release();
}

CameraItem::CameraItem(int id, const QString& string)
    : QCamCalibItem(string)
    , camera_id(id)
    , camera_parameter(NULL)
    , images(NULL)
{
    setEditable(false);
    camera_parameter = new CameraParameterItem("Parameter");
    appendRow(camera_parameter);

    images = new QStandardItem("images");
    images->setEditable(false);
    appendRow(images);
    images->setColumnCount(2);
};

int CameraItem::getId()
{
    return camera_id;
}

bool CameraItem::isCalibrated()
{
    return (camera_parameter->getParameter("fx") > 0);
}

void CameraItem::saveParameter(const QString& path) const
{
    camera_parameter->save(path);
}

cv::Mat CameraItem::getCameraMatrix()
{
    return m_k;
}

cv::Mat CameraItem::getDistCoeffs()
{
    return m_dist;
}

cv::Mat CameraItem::getFullCameraMatrix()
{
    return m_full_camera_matrix;
}

cv::Rect CameraItem::getValidROI()
{
    return m_valid_ROI;
}

cv::Rect CameraItem::getPreservedROI()
{
    return m_preserved_ROI;
}

std::vector<cv::Mat> CameraItem::getRotationVector()
{
    return m_rvecs;
}

std::vector<cv::Mat> CameraItem::getTranslationVector()
{
    return m_tvecs;
}

int CameraItem::countChessboards()
{
    int count = 0;
    for (int row = 0; row < images->rowCount(); ++row) {
        ImageItem* item = dynamic_cast<ImageItem*>(images->child(row, 0));
        if (item && !item->getChessboardCorners().empty())
            ++count;
    }
    return count;
}

void CameraItem::calibrate(int cols, int rows, float dx, float dy, int iterations)
{

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size image_size;

    // generate chessboard points
    std::vector<cv::Point3f> points3f;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col)
            points3f.push_back(cv::Point3f(dx * col, dy * row, 0));
    }

    // collect image points
    for (int row = 0; row < images->rowCount(); ++row) {
        ImageItem* item = dynamic_cast<ImageItem*>(images->child(row, 0));
        if (item && item->getChessboardCorners().size() == (int)points3f.size()) {
            image_size = cv::Size(item->getImage().width(), item->getImage().height());
            image_points.push_back(convertFromQt(item->getChessboardCorners()));
            object_points.push_back(points3f);
        }
    }
    if (object_points.size() < 1)
        throw std::runtime_error("not enough detected chessboards");

    cv::Mat k(3, 3, CV_64FC1);
    cv::Mat dist(4, 1, CV_64FC1);
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::Mat stdDeviationsIntrisics;
    cv::Mat stdDeviationsExtrisics;
    std::vector<double> perViewErrors;

    double error = cv::calibrateCamera(object_points,
        image_points,
        image_size,
        k,
        dist,
        rvecs,
        tvecs,
        stdDeviationsIntrisics,
        stdDeviationsExtrisics,
        perViewErrors,
        0,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
            iterations,
            DBL_EPSILON));

    // store parameters
    camera_parameter->setParameter("fx", k.at<double>(0, 0));
    camera_parameter->setParameter("fy", k.at<double>(1, 1));
    camera_parameter->setParameter("cx", k.at<double>(0, 2));
    camera_parameter->setParameter("cy", k.at<double>(1, 2));
    camera_parameter->setParameter("k1", dist.at<double>(0));
    camera_parameter->setParameter("k2", dist.at<double>(1));
    camera_parameter->setParameter("p1", dist.at<double>(2));
    camera_parameter->setParameter("p2", dist.at<double>(3));
    camera_parameter->setParameter("projection error", error);
    camera_parameter->setParameter("pixel error", sqrt(error / points3f.size()));

    size_t errorSize = perViewErrors.size();
    size_t errorCounter = 0;

    for (int row = 0; row < images->rowCount(); ++row) {
        ImageItem* item = dynamic_cast<ImageItem*>(images->child(row, 0));
        auto chessboard = item->getChessboardCorners();
        if (item->parent()) {
            QStandardItem* item_cell = images->child(row, 1);
            if (item_cell) {
                if (!chessboard.empty() && errorCounter <= errorSize) {
                    item_cell->setText(
                        QString::number(perViewErrors[errorCounter], 'f', 2));
                    errorCounter++;
                }
                else
                    item_cell->setText("unavailable");
            }
        }
    }

    cv::Rect valid_ROI;
    cv::Mat full_camera_matrix =
        getOptimalNewCameraMatrix(k, dist, image_size, 1, image_size, &valid_ROI);
    float desired_aspect_ratio =
        static_cast<float>(image_size.width) / static_cast<float>(image_size.height);
    cv::Rect preserved_aspect_ratio_ROI =
        adjustToDesiredAspectRatio(valid_ROI, desired_aspect_ratio);
    float percentage_fov = static_cast<float>(preserved_aspect_ratio_ROI.width) /
                           static_cast<float>(valid_ROI.width);
    camera_parameter->setParameter("Horizontal FOV percentage after undistort",
        percentage_fov);

    m_k = k.clone();
    m_dist = dist.clone();
    m_full_camera_matrix = full_camera_matrix.clone();
    m_valid_ROI = valid_ROI;
    m_preserved_ROI = preserved_aspect_ratio_ROI;
}

cv::Rect CameraItem::adjustToDesiredAspectRatio(const cv::Rect& original_rect,
    const float target_aspect_ratio)
{
    float current_aspect_ratio =
        static_cast<float>(original_rect.width) / original_rect.height;

    cv::Rect adjusted_rect;

    if (current_aspect_ratio > target_aspect_ratio) {
        // Original is wider than target aspect ratio → adjust width to match height
        adjusted_rect.width =
            static_cast<int>(std::round(original_rect.height * target_aspect_ratio));
        adjusted_rect.height = original_rect.height;
        adjusted_rect.x =
            original_rect.x + (original_rect.width - adjusted_rect.width) / 2;
        adjusted_rect.y = original_rect.y;
    }
    else {
        // Original is taller than target aspect ratio → adjust height to match width
        adjusted_rect.height =
            static_cast<int>(std::round(original_rect.width / target_aspect_ratio));
        adjusted_rect.width = original_rect.width;
        adjusted_rect.x = original_rect.x;
        adjusted_rect.y =
            original_rect.y + (original_rect.height - adjusted_rect.height) / 2;
    }

    return adjusted_rect;
}

ImageItem* CameraItem::getImageItem(const QString& name)
{
    QModelIndexList list =
        model()->match(index(), Qt::DisplayRole, QVariant(name), 1, Qt::MatchExactly);
    ImageItem* item = NULL;
    if (!list.empty())
        item = dynamic_cast<ImageItem*>(model()->itemFromIndex(list.front()));
    if (!item)
        throw std::runtime_error("Cannot find image item");
    return item;
}

ImageItem* CameraItem::addImage(const QString& name, const QImage& image)
{
    ImageItem* item = new ImageItem(name, image);
    QList<QStandardItem*> items;
    items.append(item);
    items.back()->setEditable(false);
    items.append(new QCamCalibItem("no chessboard"));
    items.back()->setEditable(false);
    images->appendRow(items);
    return item;
}

ImageItem::ImageItem(const QString& name, const QImage& image)
    : QCamCalibItem(name)
    , raw_image(image.copy())
    , image(raw_image)
{
}
ImageItem::~ImageItem()
{
}

const QVector<QPointF>& ImageItem::getChessboardCorners() const
{
    return chessboard;
}

QVector<QPointF> ImageItem::findChessboard(const QImage& image, int cols, int rows)
{
    std::vector<cv::Point2f> points;
    QImage img = image.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(img.height(), img.width(), CV_8UC3, img.bits(), img.bytesPerLine());
    cv::Mat gray;

    // opencv is not thread save here
    //   static QMutex mutex;
    //   mutex.lock();
    cv::cvtColor(mat, gray, cv::COLOR_RGB2GRAY);
    cv::findChessboardCorners(gray,
        cv::Size(cols, rows),
        points,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
            cv::CALIB_CB_FAST_CHECK);
    //   mutex.unlock();

    return convertToQt(points);
}

bool ImageItem::findChessboard(int cols, int rows)
{
    setChessboard(ImageItem::findChessboard(raw_image, cols, rows), cols, rows);
    if (chessboard.empty())
        return false;
    return true;
}

void ImageItem::setChessboard(const QVector<QPointF>& chessboard, int cols, int rows)
{
    this->chessboard = chessboard;
    if (chessboard.empty())
        image = raw_image;
    else {
        QImage img = raw_image.convertToFormat(QImage::Format_RGB888);
        cv::Mat mat(img.height(), img.width(), CV_8UC3, img.bits(), img.bytesPerLine());
        std::vector<cv::Point2f> points = convertFromQt(chessboard);
        cv::drawChessboardCorners(mat, cv::Size(cols, rows), points, true);
        image =
            QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888).copy();
    }
    if (parent()) {
        QStandardItem* item = parent()->child(row(), 1);
        if (item) {
            if (!chessboard.empty())
                item->setText("ok");
            else
                item->setText("no chessboard");
        }
    }
}

QImage& ImageItem::getImage()
{
    return image;
}

QImage& ImageItem::getRawImage()
{
    return raw_image;
}

QImage& ImageItem::getUndistortedImage(cv::Mat k, cv::Mat dist)
{
    QImage img = raw_image.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(img.height(), img.width(), CV_8UC3, img.bits(), img.bytesPerLine());
    cv::Mat out;
    cv::undistort(mat, out, k, dist);
    undistorted_image =
        QImage(out.data, out.cols, out.rows, out.step, QImage::Format_RGB888).copy();
    return undistorted_image;
}

QImage& ImageItem::getUndistortedImageWithBlackBars(cv::Mat k,
    cv::Mat dist,
    cv::Mat full_camera_matrix,
    cv::Rect valid_ROI,
    cv::Rect preserved_aspect_ratio_ROI)
{
    QImage img = raw_image.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(img.height(), img.width(), CV_8UC3, img.bits(), img.bytesPerLine());
    cv::Mat out;
    cv::undistort(mat, out, k, dist, full_camera_matrix);
    cv::Scalar green(0, 255, 0);
    cv::rectangle(out, cv::Point(valid_ROI.x, valid_ROI.y), valid_ROI.br(), green, 2);
    cv::Scalar blue(0, 0, 255);
    cv::rectangle(out,
        cv::Point(preserved_aspect_ratio_ROI.x, preserved_aspect_ratio_ROI.y),
        preserved_aspect_ratio_ROI.br(),
        blue,
        2);
    undistorted_image_with_black_bars =
        QImage(out.data, out.cols, out.rows, out.step, QImage::Format_RGB888).copy();
    return undistorted_image_with_black_bars;
}

QImage& ImageItem::getReprojectedPointsImage(cv::Mat k,
    cv::Mat dist,
    cv::Size pattern_size)
{
    QImage img = raw_image.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(img.height(), img.width(), CV_8UC3, img.bits(), img.bytesPerLine());
    cv::Mat gray;
    cv::cvtColor(mat, gray, cv::COLOR_RGB2GRAY);
    std::vector<cv::Point2f> objectPoints;
    bool found = cv::findChessboardCorners(gray,
        pattern_size,
        objectPoints,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
            cv::CALIB_CB_FAST_CHECK);

    if (!found) {
        return raw_image;
    }

    std::vector<cv::Point2f> undistortedPoints;
    cv::undistortPoints(objectPoints, undistortedPoints, k, dist, cv::noArray(), k);
    cv::Mat out(mat.clone());
    cv::drawChessboardCorners(out, pattern_size, undistortedPoints, true);
    reprojected_image =
        QImage(out.data, out.cols, out.rows, out.step, QImage::Format_RGB888).copy();
    return reprojected_image;
}