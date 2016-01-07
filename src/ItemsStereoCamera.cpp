/*
 * ItensStereoCamera.cpp
 *
 *  Created on: Jan 4, 2016
 *      Author: tiagotrocoli
 */

#include "ItemsStereoCamera.hpp"

#include <stdexcept>
#include <iostream>
#include <QFileInfo>

using namespace qcam_calib;

//StereoCameraParameterItem
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
QVector<QPointF> StereoImageItem::findChessboard(const QImage& image, int cols, int rows) {
}

StereoImageItem::StereoImageItem(const QString& name, const QString& path) :
        chessboard(NULL), QCamCalibItem(name), image_path(path) {
}

StereoImageItem::~StereoImageItem() {
}

const QVector<QPointF>& StereoImageItem::getChessboardCorners() const {
}

QImage StereoImageItem::loadRawImage() {
}

QImage StereoImageItem::drawImageWithChessboard() {
}

bool StereoImageItem::findChessboard() {
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
}

StereoImageItem* StereoCameraItem::addImages(const QList<QStandardItem*> &stereoImageitems) {
    images->appendRow(stereoImageitems);
}

StereoImageItem* StereoCameraItem::getImageItem(const QString& name) {
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
}

void StereoItem::saveParameter(const QString& path) const {
}

bool StereoItem::isCalibrated() {
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

QList<QStandardItem*> StereoTools::findChessboard(const QList<QStandardItem*> &items) {

    std::cout<<"findChessboard"<<std::endl;
    std::cout<<"ITEM NAME "<<items.at(0)->text().toStdString()<<std::endl;

    return items;
}
