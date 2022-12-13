#ifndef CAMERA_H
#define CAMERA_H

#include <QWidget>

class QCamera;
class QCameraViewfinder;
class QCameraImageCapture;
class QVBoxLayout;

namespace Ui {
class Camera;
}

class Camera : public QWidget
{
    Q_OBJECT

public:
    explicit Camera(QWidget *parent = nullptr);
    ~Camera();

private slots:
    void on_Back_Button_clicked();

    void on_Cam_Button_clicked();

private:
    Ui::Camera *ui;
    QCamera *mCamera;
    QCameraViewfinder *mCameraViewfinder;
    QCameraImageCapture *mCameraImageCapture;
    QVBoxLayout *mLayout;
};

#endif // CAMERA_H
