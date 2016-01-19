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

cv::Mat fillMatrixParametersbyParametersValues(StereoCameraParameterItem* items, cv::Size sizeMat, QList<QString> parameters) {

    cv::Mat mat = cv::Mat(sizeMat, CV_64FC1);
    cv::MatIterator_<double> it = mat.begin<double>();
    int i = 0;
    while (it != mat.end<double>()) {
        (*it) = items->getParameter(parameters[i]);
        ++it;
        ++i;
    }

    return mat;
}

void fillIntrinsecMatrixFromParametersValues(StereoCameraParameterItem* items, cv::Mat &intrinsic, cv::Mat &distCoeff, QList<QString> parameters) {

    cv::Mat k = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Mat dist = cv::Mat::zeros(8, 1, CV_64FC1);

    k.at<double>(0, 0) = items->getParameter(parameters[0]);
    k.at<double>(1, 1) = items->getParameter(parameters[1]);
    k.at<double>(0, 2) = items->getParameter(parameters[2]);
    k.at<double>(1, 2) = items->getParameter(parameters[3]);

    dist.at<double>(0) = items->getParameter(parameters[3]);
    dist.at<double>(1) = items->getParameter(parameters[4]);
    dist.at<double>(2) = items->getParameter(parameters[5]);
    dist.at<double>(3) = items->getParameter(parameters[6]);
    dist.at<double>(4) = items->getParameter(parameters[7]);
    dist.at<double>(5) = items->getParameter(parameters[8]);
    dist.at<double>(6) = items->getParameter(parameters[9]);
    dist.at<double>(7) = items->getParameter(parameters[10]);
    dist.at<double>(8) = items->getParameter(parameters[11]);

    k.copyTo(intrinsic);
    dist.copyTo(distCoeff);
}

void fillParametersValuesbyListParameters(StereoCameraParameterItem* items, cv::Mat mat, QList<QString> parameters) {

    cv::MatIterator_<double> it = mat.begin<double>();
    int i = 0;
    while (it != mat.end<double>()) {
        items->setParameter(parameters[i], (*it));
        ++it;
        ++i;
    }
}

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

    QImage qt_image(this->image_path);
    qt_image = qt_image.convertToFormat(QImage::Format_RGB888);
    cv::Mat cv_image(qt_image.height(), qt_image.width(), CV_8UC3, qt_image.bits(), qt_image.bytesPerLine());

    std::vector<cv::Point2f> cv_points;
    if (this->isChessboardFound()) {
        cv_points = StereoTools::convertQVectorQPointFToVectorPoints2f(this->chessboard);
        cv::drawChessboardCorners(cv_image, cv::Size(cols, rows), cv_points, true);
    }

    return QImage(cv_image.data, cv_image.cols, cv_image.rows, cv_image.step, QImage::Format_RGB888).copy();
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

    QStandardItem* left_images = this->left_camera->getImagesItems();
    QStandardItem* right_images = this->right_camera->getImagesItems();

    int item_row_count = left_images->rowCount();
    if (left_images->rowCount() > right_images->rowCount())
        item_row_count = right_images->rowCount();

    std::vector<std::vector<cv::Point3f> > object_points;
    std::vector<std::vector<cv::Point2f> > left_points, right_points;

    std::vector<cv::Point3f> chessboard_3Dpoints;
    for (int row = 0; row < rows; ++row)
        for (int col = 0; col < cols; ++col)
            chessboard_3Dpoints.push_back(cv::Point3f(dx * col, dy * row, 0));

    int image_sample = -1;
    for (int i = 0; i < item_row_count; ++i) {
        StereoImageItem* left_stereo_image = dynamic_cast<StereoImageItem*>(left_images->child(i, 0));
        StereoImageItem* right_stereo_image = dynamic_cast<StereoImageItem*>(right_images->child(i, 0));

        bool both_chessboard_detect = left_stereo_image->isChessboardFound() && right_stereo_image->isChessboardFound();
        if (!both_chessboard_detect)
            continue;

        left_points.push_back(StereoTools::convertQVectorQPointFToVectorPoints2f(left_stereo_image->getChessboardCorners()));
        right_points.push_back(StereoTools::convertQVectorQPointFToVectorPoints2f(right_stereo_image->getChessboardCorners()));
        object_points.push_back(chessboard_3Dpoints);
        image_sample = i;
    }

    if (object_points.size() < 5)
        throw std::runtime_error("Not enough detected chessboards!");

    StereoImageItem* stereo_image = dynamic_cast<StereoImageItem*>(left_images->child(image_sample, 0));
    QImage qt_image(stereo_image->getImagePath());
    qt_image = qt_image.convertToFormat(QImage::Format_RGB888);
    cv::Size image_size(qt_image.height(), qt_image.width());

    std::vector<cv::Mat> vec_matrix = StereoTools::stereoCalibrate(left_points, right_points, object_points, image_size);

    fillParametersValuesFromIntrinsecMatrix(this->left_camera->getParameter(), vec_matrix[0], vec_matrix[1], INTRINSIC_PARAMETERS_LIST);
    fillParametersValuesFromIntrinsecMatrix(this->right_camera->getParameter(), vec_matrix[2], vec_matrix[3], INTRINSIC_PARAMETERS_LIST);

    this->error_values->setParameter(ERROR_VALUES_PARAMETERS_LIST[0], vec_matrix[4].at<float>(0, 0));
    this->error_values->setParameter(ERROR_VALUES_PARAMETERS_LIST[1], vec_matrix[4].at<float>(0, 1));

    fillParametersValuesbyListParameters(dynamic_cast<StereoCameraParameterItem*>(this->parameters->child(0, 0)), vec_matrix[5], FUNDAMENTAL_MATRIX_PARAMETERS_LIST);
    fillParametersValuesbyListParameters(dynamic_cast<StereoCameraParameterItem*>(this->parameters->child(1, 0)), vec_matrix[6], FUNDAMENTAL_MATRIX_PARAMETERS_LIST);
    fillParametersValuesbyListParameters(dynamic_cast<StereoCameraParameterItem*>(this->parameters->child(2, 0)), vec_matrix[7], ROTATION_MATRIX_PARAMETERS_LIST);
    fillParametersValuesbyListParameters(dynamic_cast<StereoCameraParameterItem*>(this->parameters->child(3, 0)), vec_matrix[8], TRANSLATION_VECTOR_PARAMETERS_LIST);

}

void StereoItem::saveParameter(const QString& path) const {

    std::vector<cv::Mat> intrisc_mat(2), dist_coeff(2);
    fillIntrinsecMatrixFromParametersValues(this->left_camera->getParameter(), intrisc_mat[0], dist_coeff[0], INTRINSIC_PARAMETERS_LIST);
    fillIntrinsecMatrixFromParametersValues(this->right_camera->getParameter(), intrisc_mat[1], dist_coeff[1], INTRINSIC_PARAMETERS_LIST);

    cv::Mat fundamental = fillMatrixParametersbyParametersValues(dynamic_cast<StereoCameraParameterItem*>(parameters->child(0, 0)), cv::Size(3, 3), FUNDAMENTAL_MATRIX_PARAMETERS_LIST);
    cv::Mat essential = fillMatrixParametersbyParametersValues(dynamic_cast<StereoCameraParameterItem*>(parameters->child(1, 0)), cv::Size(3, 3), FUNDAMENTAL_MATRIX_PARAMETERS_LIST);
    cv::Mat rotation = fillMatrixParametersbyParametersValues(dynamic_cast<StereoCameraParameterItem*>(parameters->child(2, 0)), cv::Size(3, 3), ROTATION_MATRIX_PARAMETERS_LIST);
    cv::Mat translation = fillMatrixParametersbyParametersValues(dynamic_cast<StereoCameraParameterItem*>(parameters->child(3, 0)), cv::Size(3, 1), TRANSLATION_VECTOR_PARAMETERS_LIST);

    cv::FileStorage fs(path.toStdString(), cv::FileStorage::WRITE);
    time_t rawtime;
    time(&rawtime);
    fs << "calibrationDate" << asctime(localtime(&rawtime));
    fs << "leftCameraMatrix" << intrisc_mat[0] << "leftDistCoeffs" << dist_coeff[0];
    fs << "rightCameraMatrix" << intrisc_mat[1] << "rightDistCoeffs" << dist_coeff[1];
    fs << "fundamentalMatrix" << fundamental << "essentialMatrix" << essential;
    fs << "rotationMatrix" << rotation << "translationMatrix" << translation;
    fs.release();
}

bool StereoItem::isCalibrated() {
    return error_values->getParameter(ERROR_VALUES_PARAMETERS_LIST[0]) > 0;
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

    QImage qt_image(path);
    qt_image = qt_image.convertToFormat(QImage::Format_RGB888);
    cv::Mat cv_image(qt_image.height(), qt_image.width(), CV_8UC3, qt_image.bits(), qt_image.bytesPerLine());
    cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> cv_points;
    cv::findChessboardCorners(cv_image, cv::Size(cols, rows), cv_points);

    return convertVectorPoints2fToQVectorQPointF(cv_points);
}

std::vector<cv::Mat> StereoTools::stereoCalibrate(std::vector<std::vector<cv::Point2f> > left_points, std::vector<std::vector<cv::Point2f> > right_points,
        std::vector<std::vector<cv::Point3f> > object_points, cv::Size image_size) {

    cv::Mat camera_matrix[2], dist_coeffs[2];
    camera_matrix[0] = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix[1] = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat rotation, translation, essential, fundamental;

    double rms = cv::stereoCalibrate(object_points, left_points, right_points, camera_matrix[0], dist_coeffs[0], camera_matrix[1], dist_coeffs[1], image_size, rotation, translation, essential,
            fundamental, cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 1000, 1e-8), cv::CALIB_RATIONAL_MODEL);

    double err = 0;
    int npoints = 0;
    std::vector<cv::Vec3f> lines[2];
    for (int i = 0; i < object_points.size(); i++) {
        int npt = (int) left_points[i].size();
        cv::Mat imgpt[2];
        for (int k = 0; k < 2; k++) {
            std::vector<cv::Point2f> tempPoints;
            k == 0 ? tempPoints = left_points[i] : tempPoints = right_points[i];
            imgpt[k] = cv::Mat(tempPoints);
            cv::undistortPoints(imgpt[k], imgpt[k], camera_matrix[k], dist_coeffs[k], cv::Mat(), camera_matrix[k]);
            cv::computeCorrespondEpilines(imgpt[k], k + 1, fundamental, lines[k]);
        }
        for (int j = 0; j < npt; j++) {
            double errij = fabs(left_points[i][j].x * lines[1][j][0] + left_points[i][j].y * lines[1][j][1] + lines[1][j][2])
                    + fabs(right_points[i][j].x * lines[0][j][0] + right_points[i][j].y * lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }

    cv::Mat error_mat = (cv::Mat_<float>(1, 2) << rms, err / npoints);

    std::vector<cv::Mat> mats;
    mats.push_back(camera_matrix[0]);
    mats.push_back(dist_coeffs[0]);
    mats.push_back(camera_matrix[1]);
    mats.push_back(dist_coeffs[1]);

    mats.push_back(error_mat);
    mats.push_back(fundamental);
    mats.push_back(essential);
    mats.push_back(rotation);
    mats.push_back(translation);

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
