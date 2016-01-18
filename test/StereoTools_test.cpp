/*
 * StereoTools_test.cpp
 *
 *  Created on: Jan 18, 2016
 *      Author: tiagotrocoli
 */

#define BOOST_TEST_MODULE "StereoTools_test"
#include <boost/test/unit_test.hpp>

#include <vector>
#include <string>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <qcam_calib/ItemsStereoCamera.hpp>

#define IMAGE_STEREO_CHESSBOARD "../../test/resource/stereo_dataset"
#define NO_CHESSBOARD "No Chessboard"
#define OK_CHESSBOARD "OK, Chessboard Found"

using namespace qcam_calib;

int precision = 1000;
int chessboard_cols = 9;
int chessboard_rows = 6;

int ground_truth_points[][54 * 2] = { { 244, 94, 274, 92, 305, 90, 338, 88, 371, 87, 406, 86, 441, 86, 477, 86, 514, 86, 244, 126, 275, 125, 306, 124, 338, 123, 372, 122, 406, 122, 442, 122, 478, 122,
        513, 123, 245, 158, 275, 158, 306, 157, 338, 157, 372, 157, 406, 157, 441, 157, 477, 158, 513, 158, 246, 190, 275, 190, 307, 190, 339, 191, 372, 191, 406, 192, 441, 193, 477, 194, 513, 195,
        247, 222, 276, 223, 307, 224, 339, 225, 372, 226, 406, 227, 440, 228, 476, 229, 511, 231, 248, 253, 277, 255, 308, 256, 340, 258, 372, 259, 406, 261, 440, 262, 475, 264, 510, 266 },

{ 256, 357, 255, 334, 254, 308, 253, 280, 252, 248, 252, 213, 251, 172, 251, 128, 251, 77, 291, 366, 292, 343, 293, 318, 294, 290, 296, 257, 298, 221, 301, 182, 304, 135, 306, 86, 327, 374, 331, 352,
        334, 327, 337, 299, 342, 267, 347, 233, 352, 192, 357, 145, 365, 96, 367, 382, 369, 360, 375, 335, 381, 308, 388, 277, 396, 241, 404, 202, 413, 158, 423, 106, 401, 389, 407, 368, 416, 344,
        425, 317, 433, 287, 445, 252, 456, 213, 469, 169, 482, 120, 437, 396, 446, 376, 456, 352, 467, 326, 480, 296, 492, 262, 508, 224, 523, 181, 539, 131 },

{ 277, 72, 313, 81, 352, 91, 394, 101, 434, 113, 476, 126, 519, 140, 561, 154, 604, 169, 259, 106, 297, 114, 336, 126, 378, 137, 421, 150, 465, 165, 508, 178, 552, 193, 594, 208, 242, 141, 279, 152,
        320, 164, 364, 177, 406, 190, 450, 205, 495, 219, 540, 235, 585, 250, 224, 177, 262, 191, 303, 204, 345, 218, 389, 233, 435, 248, 481, 264, 528, 280, 573, 295, 205, 217, 244, 230, 284, 245,
        328, 261, 372, 276, 419, 293, 465, 309, 513, 326, 560, 342, 187, 257, 225, 272, 267, 289, 309, 305, 354, 323, 402, 340, 449, 358, 496, 373, 545, 390 } };

std::string intToStr(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

BOOST_AUTO_TEST_CASE(convertOpenCVpointstoQt_testcase) {

    std::vector<cv::Point2f> pointsCV;
    pointsCV.push_back(cv::Point2f(0, 0));
    pointsCV.push_back(cv::Point2f(-0.1235, 0.1235));
    pointsCV.push_back(cv::Point2f(1.2345, 0.1235));
    pointsCV.push_back(cv::Point2f(0.2345, -1.1235));

    QVector<QPointF> qtPoints = StereoTools::convertVectorPoints2fToQVectorQPointF(pointsCV);

    QVector<QPointF>::const_iterator iterQt = qtPoints.begin();
    std::vector<cv::Point2f>::const_iterator iterCV = pointsCV.begin();
    for (; iterCV != pointsCV.end(); ++iterCV, ++iterQt) {
        BOOST_CHECK_EQUAL((int )(iterQt->x() * precision), (int ) (iterCV->x * precision));
        BOOST_CHECK_EQUAL((int )(iterQt->y() * precision), (int )(iterCV->y * precision));
    }
}

BOOST_AUTO_TEST_CASE(convertQtpointstoOpenCV_testcase) {

    QVector<QPointF> qt_points;
    qt_points.push_back(QPointF(0, 0));
    qt_points.push_back(QPointF(-0.123, 0.123));
    qt_points.push_back(QPointF(1.234, 0.123));
    qt_points.push_back(QPointF(0.234, -1.123));

    std::vector<cv::Point2f> cv_points = StereoTools::convertQVectorQPointFToVectorPoints2f(qt_points);

    QVector<QPointF>::const_iterator iterQt = qt_points.begin();
    std::vector<cv::Point2f>::const_iterator iterCV = cv_points.begin();
    for (; iterQt != qt_points.end(); ++iterCV, ++iterQt) {
        BOOST_CHECK_EQUAL((int )(iterQt->x() * precision), (int ) (iterCV->x * precision));
        BOOST_CHECK_EQUAL((int )(iterQt->y() * precision), (int )(iterCV->y * precision));
    }
}

BOOST_AUTO_TEST_CASE(testSearchChessboard_testcase) {

    std::string root_path = std::string(IMAGE_STEREO_CHESSBOARD) + "/left_images/left";
    for (int i = 1; i <= 3; ++i) {

        std::string temp_path = root_path + "0" + intToStr(i) + ".jpg";
        QVector<QPointF> qt_points = StereoTools::findChessboard(QString::fromUtf8(temp_path.c_str()), chessboard_cols, chessboard_rows);

        int count_points_gt = 0;
        QVector<QPointF>::const_iterator iterQt = qt_points.begin();
        for (; iterQt != qt_points.end(); ++iterQt) {
            BOOST_CHECK_EQUAL((int )iterQt->x(), ground_truth_points[i - 1][count_points_gt++]);
            BOOST_CHECK_EQUAL((int )iterQt->y(), ground_truth_points[i - 1][count_points_gt++]);
        }
    }
}

BOOST_AUTO_TEST_CASE(loadStereoImageAndFindChessboardItem_test) {

    std::string root_path = std::string(IMAGE_STEREO_CHESSBOARD) + "/left_images/left";
    for (int i = 1; i <= 3; ++i) {

        std::string temp_path = root_path + "0" + intToStr(i) + ".jpg";
        QList<QStandardItem*> items = StereoTools::loadStereoImageAndFindChessboardItem(QString::fromUtf8(temp_path.c_str()), chessboard_cols, chessboard_rows);
        StereoImageItem *item = dynamic_cast<StereoImageItem*>(items.at(0));

        BOOST_CHECK_EQUAL(items.at(1)->text().toStdString(), std::string(OK_CHESSBOARD));

        int count_points_gt = 0;
        QVector<QPointF>::const_iterator iterQt = item->getChessboardCorners().begin();
        for (; iterQt != item->getChessboardCorners().end(); ++iterQt) {
            BOOST_CHECK_EQUAL((int )iterQt->x(), ground_truth_points[i - 1][count_points_gt++]);
            BOOST_CHECK_EQUAL((int )iterQt->y(), ground_truth_points[i - 1][count_points_gt++]);
        }

        std::cout << std::endl;
    }
}

