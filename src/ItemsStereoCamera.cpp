/*
 * ItensStereoCamera.cpp
 *
 *  Created on: Jan 4, 2016
 *      Author: tiagotrocoli
 */

#include "ItemsStereoCamera.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdexcept>
#include <iostream>
#include <QFileInfo>

using namespace qcam_calib;

StereoCameraParameterItem::StereoCameraParameterItem(const QString& string, QList<QString> parameters) :
        QCamCalibItem(string) {

    setEditable(false);
    setColumnCount(2);

    for (int i = 0; i < parameters.size(); ++i) {
        setParameter(parameters[i], 0);
    }
}

void StereoCameraParameterItem::setParameter(const QString& name, double val) {
    QModelIndexList list;
    if (model() && index().isValid())
        list = model()->match(index().child(0, 0), Qt::DisplayRole, QVariant(name), 1, Qt::MatchExactly);
    if (list.empty()) {
        QList<QStandardItem*> items;
        items.append(new QCamCalibItem(name));
        items.back()->setEditable(false);
        items.append(new QCamCalibItem(val));
        items.back()->setEditable(false);
        appendRow(items);
    } else {
        QStandardItem *item = child(list.front().row(), 1);
        if (item)
            item->setText(QString::number(val));
    }
}

double StereoCameraParameterItem::getParameter(const QString& name) const {
    QModelIndexList list;
    if (model() && index().isValid())
        list = model()->match(index().child(0, 0), Qt::DisplayRole, QVariant(name), 1, Qt::MatchExactly);
    if (!list.empty()) {
        QStandardItem *item = child(list.front().row(), 1);
        if (item)
            return item->data(Qt::EditRole).toDouble();
    } else
        throw std::runtime_error("cannot find parameter");
    return 0;
}

void StereoCameraParameterItem::save(const QString& path) const {

}

//StereoImageItem
StereoImageItem::StereoImageItem(const QString& name, const QString& path) :
        chessboard(NULL), QCamCalibItem(name), image_path(path) {
}

StereoImageItem::~StereoImageItem() {
}

bool StereoImageItem::isChessboardFound() {
    return &this->chessboard != 0 && this->chessboard.size() > 0;
}

const QString& StereoImageItem::getImagePath() const {
    return this->image_path;
}

void StereoImageItem::setChessboardCorners(QVector<QPointF> points) {
    this->chessboard = points;
}

const QVector<QPointF>& StereoImageItem::getChessboardCorners() const {
    return this->chessboard;
}

QImage qcam_calib::StereoImageItem::getImageWithChessboard(int cols, int rows) {

    QImage qtImage(this->image_path);
    qtImage = qtImage.convertToFormat(QImage::Format_RGB888);
    cv::Mat cvImage(qtImage.height(), qtImage.width(), CV_8UC3, qtImage.bits(), qtImage.bytesPerLine());

    std::vector<cv::Point2f> cvPoints;
    if (this->isChessboardFound()) {
        cvPoints = StereoTools::convertQVectorQPointFToVectorPoints2f(this->chessboard);
        cv::drawChessboardCorners(cvImage, cv::Size(cols, rows), cvPoints, true);
    }

    return QImage(cvImage.data, cvImage.cols, cvImage.rows, cvImage.step, QImage::Format_RGB888).copy();
}

//StereoCameraItem
StereoCameraItem::StereoCameraItem(int id, const QString& string) :
        QCamCalibItem(string), camera_id(id) {

    setEditable(false);
    this->parameter = new StereoCameraParameterItem("Parameters", INTRINSIC_PARAMETERS_LIST);
    appendRow(this->parameter);

    images = new QStandardItem("Images");
    images->setEditable(false);
    appendRow(images);
}

int StereoCameraItem::getId() {
    return this->camera_id;
}

StereoImageItem* StereoCameraItem::addImages(const QList<QStandardItem*> &stereoImageitems) {
    images->appendRow(stereoImageitems);
}

StereoImageItem* StereoCameraItem::getImageItem(const QString& name) {
}

QStandardItem* StereoCameraItem::getImagesItems() const {
    return this->images;
}

//StereoItem
StereoItem::StereoItem(int id, const QString& string) :
        QCamCalibItem(string), stereo_id(id) {

    setEditable(false);

    this->fundamental_matrix = new StereoCameraParameterItem("Fundamental Matrix", FUNDAMENTAL_MATRIX_PARAMETERS_LIST);
    appendRow(this->fundamental_matrix);

    this->left_camera = new StereoCameraItem(0, "Left Camera");
    appendRow(this->left_camera);

    this->right_camera = new StereoCameraItem(1, "Right Camera");
    appendRow(this->right_camera);
}

int StereoItem::getId() {
    return this->stereo_id;
}

char* StereoItem::getBaseName() {
    return "Stereo Camera ";
}

void StereoItem::calibrate(int cols, int rows, float dx, float dy) {

    std::cout << "StereoItem::calibrate" << std::endl;
    QStandardItem* leftImages = this->left_camera->getImagesItems();
    QStandardItem* rightImages = this->right_camera->getImagesItems();

    int itemRowCount = leftImages->rowCount();
    if (leftImages->rowCount() > rightImages->rowCount())
        itemRowCount = rightImages->rowCount();

    std::vector<std::vector<cv::Point3f> > objectPoints;
    std::vector<std::vector<cv::Point2f> > leftPoints, rightPoints;

    std::vector<cv::Point3f> chessboard3DPonts;
    for (int row = 0; row < rows; ++row)
        for (int col = 0; col < cols; ++col)
            chessboard3DPonts.push_back(cv::Point3f(dx * col, dy * row, 0));

    int imageSample = -1;
    for (int i = 0; i < itemRowCount; ++i) {
        StereoImageItem* leftStereoImage = dynamic_cast<StereoImageItem*>(leftImages->child(i, 0));
        StereoImageItem* rightStereoImage = dynamic_cast<StereoImageItem*>(rightImages->child(i, 0));

        bool bothChessboardDetect = leftStereoImage->isChessboardFound() && rightStereoImage->isChessboardFound();
        if (!bothChessboardDetect)
            continue;

        leftPoints.push_back(StereoTools::convertQVectorQPointFToVectorPoints2f(leftStereoImage->getChessboardCorners()));
        rightPoints.push_back(StereoTools::convertQVectorQPointFToVectorPoints2f(rightStereoImage->getChessboardCorners()));
        objectPoints.push_back(chessboard3DPonts);
        imageSample = i;
    }

    if (objectPoints.size() < 5)
        throw std::runtime_error("Not enough detected chessboards!");

    StereoImageItem* stereoImage = dynamic_cast<StereoImageItem*>(leftImages->child(imageSample, 0));
    QImage qtImage(stereoImage->getImagePath());
    qtImage = qtImage.convertToFormat(QImage::Format_RGB888);
    cv::Size imageSize(qtImage.height(), qtImage.width());

    StereoTools::stereoCalibrate(leftPoints, rightPoints, objectPoints, imageSize);

}

void StereoItem::saveParameter(const QString& path) const {
}

bool StereoItem::isCalibrated() {

}

QList<QStandardItem*> StereoTools::loadStereoImageAndFindChessboardItem(const QString& path, int cols, int rows) {

    QList<QStandardItem*> items = loadStereoImageItem(path);

    StereoImageItem *item = dynamic_cast<StereoImageItem*>(items.at(0));
    item->setChessboardCorners(findChessboard(path, cols, rows));

    if (item->isChessboardFound())
        items.at(1)->setText("OK, Chessboard Found");

    return items;

}

QList<QStandardItem*> StereoTools::loadStereoImageItem(const QString &path) {

    QFileInfo info(path);

    QList<QStandardItem*> items;
    StereoImageItem *item = new StereoImageItem(info.fileName(), info.absoluteFilePath());

    item->setEditable(false);
    items.append(item);

    items.append(new QCamCalibItem("No Chessboard"));
    items.back()->setEditable(false);

    return items;
}

QVector<QPointF> StereoTools::findChessboard(const QString &path, int cols, int rows) {

    QImage qtImage(path);
    qtImage = qtImage.convertToFormat(QImage::Format_RGB888);
    cv::Mat cvImage(qtImage.height(), qtImage.width(), CV_8UC3, qtImage.bits(), qtImage.bytesPerLine());
    cv::cvtColor(cvImage, cvImage, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> cvPoints;
    cv::findChessboardCorners(cvImage, cv::Size(cols, rows), cvPoints);
//
    return convertVectorPoints2fToQVectorQPointF(cvPoints);
}

void qcam_calib::StereoTools::stereoCalibrate(std::vector<std::vector<cv::Point2f> > leftPoints, std::vector<std::vector<cv::Point2f> > rightPoints,
        std::vector<std::vector<cv::Point3f> > objectPoints, cv::Size imageSize) {

    std::cout << "SIZE LEFT" << leftPoints.size() << std::endl;
    std::cout << "SIZE RIGHT" << rightPoints.size() << std::endl;
    std::cout << "SIZE CHESSBOARD" << objectPoints.size() << std::endl;
    std::cout << "IMAGE SIZE" << imageSize << std::endl;

}

QVector<QPointF> StereoTools::convertVectorPoints2fToQVectorQPointF(const std::vector<cv::Point2f>&points1) {
    QVector<QPointF> points2;
    std::vector<cv::Point2f>::const_iterator iter = points1.begin();
    for (; iter != points1.end(); ++iter)
        points2.push_back(QPointF(iter->x, iter->y));
    return points2;
}

std::vector<cv::Point2f> StereoTools::convertQVectorQPointFToVectorPoints2f(const QVector<QPointF>&points1) {
    std::vector<cv::Point2f> points2;
    QVector<QPointF>::const_iterator iter = points1.begin();
    for (; iter != points1.end(); ++iter)
        points2.push_back(cv::Point2f(iter->x(), iter->y()));
    return points2;
}

