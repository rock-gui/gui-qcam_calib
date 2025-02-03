#ifndef QCAMCALIB_ITEM_MODELS_HPP
#define QCAMCALIB_ITEM_MODELS_HPP

#include <QStandardItem>
#include <QMenu>
#include <QVector>

#include <opencv2/imgproc/imgproc.hpp>

namespace qcam_calib
{
    class QCamCalibItem: public QStandardItem
    {
        public:
            QCamCalibItem():QStandardItem(){};
            QCamCalibItem(const QString &string):QStandardItem(string){};
            QCamCalibItem(float val):QStandardItem(val){};
    };

    class CameraParameterItem: public QCamCalibItem
    {
        public:
            CameraParameterItem(const QString &string);
            void setParameter(const QString &name,double val=0);
            void save(const QString &path)const;
            double getParameter(const QString &name)const;
    };

    // class ImageParameterItem: public QCamCalibItem
    // {
    //     public:
    //     ImageParameterItem(const Qstring &string);
    //     void setParameter(const QString &name, double val=0);
    //     double getParameter(const QString &name)const;
    // }

    class ImageItem : public QCamCalibItem
    {
        public:
            static QVector<QPointF> findChessboard(const QImage &image,int cols ,int rows);

            ImageItem(const QString &name, const QImage &image);
            virtual ~ImageItem();
            QImage &getImage();
            QImage &getRawImage();
            QImage &getUndistortedImage(cv::Mat k, cv::Mat dist);
            QImage &getReprojectedPointsImage(cv::Mat k, cv::Mat dist, cv::Size pattern_size);
            const QVector<QPointF> &getChessboardCorners()const;

            bool findChessboard(int cols ,int rows);
            void setChessboard(const QVector<QPointF> &chessboard,int cols,int rows);

        private:
            QImage raw_image;
            QImage image;     // image with chessboard overlay
            QImage undistorted_image;   // undistorted image
            QImage reprojected_image;   // reprojected image
            QVector<QPointF> chessboard;
    };

    class CameraItem: public QCamCalibItem
    {
        public:
            CameraItem(int id, const QString &string);
            int getId();
            cv::Mat getCameraMatrix();
            cv::Mat getDistCoeffs();
            std::vector<cv::Mat> getRotationVector();
            std::vector<cv::Mat> getTranslationVector();
            ImageItem* addImage(const QString &name,const QImage &image);
            ImageItem* getImageItem(const QString &name);
            void calibrate(int cols,int rows,float dx,float dy);
            void saveParameter(const QString &path)const;
            bool isCalibrated();
            int countChessboards();

        private:
            int camera_id;
            CameraParameterItem* camera_parameter;
            QStandardItem *images;
            cv::Mat m_dist;
            cv::Mat m_k;
            std::vector<cv::Mat> m_rvecs;
            std::vector<cv::Mat> m_tvecs;
    };

}
#endif
