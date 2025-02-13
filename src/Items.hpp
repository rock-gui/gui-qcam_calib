#ifndef QCAMCALIB_ITEM_MODELS_HPP
#define QCAMCALIB_ITEM_MODELS_HPP

#include <QMenu>
#include <QStandardItem>
#include <QVector>

#include <opencv2/imgproc/imgproc.hpp>

namespace qcam_calib {
    class QCamCalibItem : public QStandardItem {
    public:
        QCamCalibItem()
            : QStandardItem() {};
        QCamCalibItem(const QString& string)
            : QStandardItem(string) {};
        QCamCalibItem(float val)
            : QStandardItem(val) {};
    };

    class CameraParameterItem : public QCamCalibItem {
    public:
        CameraParameterItem(const QString& string);
        void setParameter(const QString& name, double val = 0);
        void save(const QString& path) const;
        double getParameter(const QString& name) const;
    };

    class ImageItem : public QCamCalibItem {
    public:
        static QVector<QPointF> findChessboard(const QImage& image, int cols, int rows);

        ImageItem(const QString& name, const QImage& image);
        virtual ~ImageItem();
        QImage& getImage();
        QImage& getRawImage();
        QImage& getUndistortedImage(cv::Mat k, cv::Mat dist);
        QImage& getUndistortedImageWithBlackBars(cv::Mat k,
            cv::Mat dist,
            cv::Mat full_camera_matrix,
            cv::Rect valid_ROI,
            cv::Rect preserved_aspect_ratio_ROI);
        QImage& getReprojectedPointsImage(cv::Mat k, cv::Mat dist, cv::Size pattern_size);

        const QVector<QPointF>& getChessboardCorners() const;
        bool findChessboard(int cols, int rows);
        void setChessboard(const QVector<QPointF>& chessboard, int cols, int rows);

    private:
        QImage raw_image;
        QImage image;                             // image with chessboard overlay
        QImage undistorted_image;                 // undistorted image
        QImage undistorted_image_with_black_bars; // undistorted image with black bars
        QImage reprojected_image;                 // reprojected image
        QVector<QPointF> chessboard;
    };

    class CameraItem : public QCamCalibItem {
    public:
        CameraItem(int id, const QString& string);
        int getId();
        cv::Mat getCameraMatrix();
        cv::Mat getDistCoeffs();
        cv::Mat getFullCameraMatrix();
        cv::Rect getValidROI();
        cv::Rect getPreservedROI();
        cv::Rect adjustToDesiredAspectRatio(const cv::Rect& original_rect,
            const float target_aspect_ratio);
        std::vector<cv::Mat> getRotationVector();
        std::vector<cv::Mat> getTranslationVector();
        ImageItem* addImage(const QString& name, const QImage& image);
        ImageItem* getImageItem(const QString& name);
        void calibrate(int cols, int rows, float dx, float dy, int iterations);
        void saveParameter(const QString& path) const;
        bool isCalibrated();
        int countChessboards();

    private:
        int camera_id;
        CameraParameterItem* camera_parameter;
        QStandardItem* images;
        cv::Mat m_dist;
        cv::Mat m_k;
        cv::Mat m_full_camera_matrix;
        cv::Rect m_valid_ROI;
        cv::Rect m_preserved_ROI;
        std::vector<cv::Mat> m_rvecs;
        std::vector<cv::Mat> m_tvecs;
    };

}
#endif
