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

void fillParametersValuesFromIntrinsecMatrix(StereoCameraParameterItem* items, cv::Mat intrinsic, cv::Mat distMatrix, QList<QString> parameters) {

    items->setParameter(parameters[0], intrinsic.at<double>(0, 0)); // fx
    items->setParameter(parameters[1], intrinsic.at<double>(1, 1)); // fy
    items->setParameter(parameters[2], intrinsic.at<double>(0, 2)); // cx
    items->setParameter(parameters[3], intrinsic.at<double>(1, 2)); // cx
    items->setParameter(parameters[4], distMatrix.at<double>(0)); // k1
    items->setParameter(parameters[5], distMatrix.at<double>(1)); // k2
    items->setParameter(parameters[6], distMatrix.at<double>(2)); // p1
    items->setParameter(parameters[7], distMatrix.at<double>(3)); // p2
    items->setParameter(parameters[8], distMatrix.at<double>(4)); // k3
    items->setParameter(parameters[9], distMatrix.at<double>(5)); // k4
    items->setParameter(parameters[10], distMatrix.at<double>(6)); // k5
    items->setParameter(parameters[11], distMatrix.at<double>(7)); // k6
}

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
        throw std::runtime_error("Cannot find parameter!");
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

StereoCameraParameterItem* StereoCameraItem::getParameter() const {
    return this->parameter;
}

//StereoItem
StereoItem::StereoItem(int id, const QString& string) :
        QCamCalibItem(string), stereo_id(id) {

    setEditable(false);

    this->parameters = new QStandardItem("Parameters");
    this->parameters->setEditable(false);
    appendRow(this->parameters);

    this->parameters->appendRow(new StereoCameraParameterItem("Fundamental Matrix", FUNDAMENTAL_MATRIX_PARAMETERS_LIST));
    this->parameters->appendRow(new StereoCameraParameterItem("Essential Matrix", FUNDAMENTAL_MATRIX_PARAMETERS_LIST));
    this->parameters->appendRow(new StereoCameraParameterItem("Rotation Matrix", ROTATION_MATRIX_PARAMETERS_LIST));
    this->parameters->appendRow(new StereoCameraParameterItem("Translation Matrix", TRANSLATION_VECTOR_PARAMETERS_LIST));

    this->error_values = new StereoCameraParameterItem("Error Values", ERROR_VALUES_PARAMETERS_LIST);
    appendRow(this->error_values);

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

    std::vector<cv::Mat> vecMatrix = StereoTools::stereoCalibrate(leftPoints, rightPoints, objectPoints, imageSize);

    fillParametersValuesFromIntrinsecMatrix(this->left_camera->getParameter(), vecMatrix[0], vecMatrix[1], INTRINSIC_PARAMETERS_LIST);
    fillParametersValuesFromIntrinsecMatrix(this->right_camera->getParameter(), vecMatrix[2], vecMatrix[3], INTRINSIC_PARAMETERS_LIST);

    this->error_values->setParameter(ERROR_VALUES_PARAMETERS_LIST[0], vecMatrix[4].at<float>(0,0));
    this->error_values->setParameter(ERROR_VALUES_PARAMETERS_LIST[1], vecMatrix[4].at<float>(0,1));

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

std::vector<cv::Mat> StereoTools::stereoCalibrate(std::vector<std::vector<cv::Point2f> > leftPoints, std::vector<std::vector<cv::Point2f> > rightPoints,
        std::vector<std::vector<cv::Point3f> > objectPoints, cv::Size imageSize) {

    cv::Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat rotation, translation, essential, fundamental;

    double rms = cv::stereoCalibrate(objectPoints, leftPoints, rightPoints, cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], imageSize, rotation, translation, essential, fundamental,
            cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 1000, 1e-8), cv::CALIB_RATIONAL_MODEL);
    std::cout << " RMS ERROR " << rms << std::endl;

    double err = 0;
    int npoints = 0;
    std::vector<cv::Vec3f> lines[2];
    for (int i = 0; i < objectPoints.size(); i++) {
        int npt = (int) leftPoints[i].size();
        cv::Mat imgpt[2];
        for (int k = 0; k < 2; k++) {
            std::vector<cv::Point2f> tempPoints;
            k == 0 ? tempPoints = leftPoints[i] : tempPoints = rightPoints[i];
            imgpt[k] = cv::Mat(tempPoints);
            cv::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
            cv::computeCorrespondEpilines(imgpt[k], k + 1, fundamental, lines[k]);
        }
        for (int j = 0; j < npt; j++) {
            double errij = fabs(leftPoints[i][j].x * lines[1][j][0] + leftPoints[i][j].y * lines[1][j][1] + lines[1][j][2])
                    + fabs(rightPoints[i][j].x * lines[0][j][0] + rightPoints[i][j].y * lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }

    std::cout << " AVAREGE ERROR " << err / npoints << std::endl;

    cv::Mat errorMat = (cv::Mat_<float>(1, 2) << rms, err/npoints);

    std::vector<cv::Mat> mats;
    mats.push_back(cameraMatrix[0]);
    mats.push_back(distCoeffs[0]);
    mats.push_back(cameraMatrix[1]);
    mats.push_back(distCoeffs[1]);

    mats.push_back(errorMat);

    mats.push_back(rotation);
    mats.push_back(translation);
    mats.push_back(fundamental);
    mats.push_back(essential);

//    std::cout << "CAMERA INTRIN LEFt " << cameraMatrix[0] << std::endl;
//    std::cout << "DIST COEFF LEFT " << distCoeffs[0] << std::endl;
//
//    std::cout << "CAMERA INTRIN RIGHT" << cameraMatrix[1] << std::endl;
//    std::cout << "DIST COEFF RIGHT" << distCoeffs[1] << std::endl;
//    std::cout << "error  " << errorMat << std::endl;

    return mats;
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
