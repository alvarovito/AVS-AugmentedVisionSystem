#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pthread.h>
#include "augmentedVision/twoImages.h"
#include "augmentedVision/rectanglesVector.h"
#include "augmentedVision/rectangle.h"
#include <iostream>

using namespace cv;
using namespace std;

Mat leftFrame,rightFrame;

bool cerrar=false;

vector<Rect> faces;
vector <int> numdetections;
bool holdImage = true;

//CascadeClassifier face_cascade;
//Mat leftFrame, rightFrame;
Mat workingImage;

CascadeClassifier face_cascade;
CascadeClassifier eyeDetector;

float EYE_SX = 0.12f;
float EYE_SY = 0.17f;
float EYE_SW = 0.37f;
float EYE_SH = 0.36f;

double DESIRED_LEFT_EYE_Y = 0.14;
double DESIRED_LEFT_EYE_X = 0.19;

int FaceWidth = 100;
int FaceHeight = 100;


void updateImages(const augmentedVision::twoImages::ConstPtr& msg)
{
  try
  {
    cv_bridge::toCvCopy( msg->leftimage, "bgr8")->image.copyTo(leftFrame);
    cv_bridge::toCvCopy( msg->rightimage, "bgr8")->image.copyTo(rightFrame);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Fallo al cargar frames");
  }
}

void detectFaces(Mat& imgInput)
{
    cvtColor(imgInput, workingImage, COLOR_BGR2GRAY);
    equalizeHist(workingImage, workingImage);

    face_cascade.detectMultiScale(workingImage, faces, 1.1, 8, CV_HAAR_SCALE_IMAGE | CV_HAAR_DO_CANNY_PRUNING, Size(50,50), Size(300,300));
}

bool EyeDetection(Mat& image, Rect& face_detected, Rect& lEye, Rect& rEye)
{
    Mat face = image(face_detected);

    int leftX = cvRound(face.cols * EYE_SX);
    int topY = cvRound(face.rows * EYE_SY);
    int widthX = cvRound(face.cols * EYE_SW);
    int heightY = cvRound(face.rows * EYE_SH);
    int rightX = cvRound(face.cols * (1.0 - EYE_SX - EYE_SW));

    Mat topLeftOfFace = face(Rect(leftX, topY, widthX, heightY));
    Mat topRightOfFace = face(Rect(rightX, topY, widthX, heightY));

    vector<Rect> lEyeR, rEyeR;

    eyeDetector.detectMultiScale(topLeftOfFace, lEyeR, 1.1, 3, CASCADE_DO_ROUGH_SEARCH);
    eyeDetector.detectMultiScale(topRightOfFace, rEyeR, 1.1, 3, CASCADE_DO_ROUGH_SEARCH);

    if (lEyeR.size() == 1 && rEyeR.size() == 1)
    {
        lEye = lEyeR[0];
        rEye = rEyeR[0];

        lEye.x += leftX;
        lEye.y += topY;

        rEye.x += rightX;
        rEye.y += topY;

        return true;
    }

    return false;
}

void CropFace(Mat face, Mat warped, Rect leftEye, Rect rightEye)
{
    Point left = Point(leftEye.x + leftEye.width / 2, leftEye.y + leftEye.height / 2);
    Point right = Point(rightEye.x + rightEye.width / 2, rightEye.y + rightEye.height / 2);
    Point2f eyesCenter = Point2f((left.x + right.x) * 0.5f, (left.y + right.y) * 0.5f);

    // Get the angle between the 2 eyes.
    double dy = (right.y - left.y);
    double dx = (right.x - left.x);
    double len = sqrt(dx*dx + dy*dy);
    double angle = atan2(dy, dx) * 180.0 / CV_PI;

    // Hand measurements shown that the left eye center should ideally be at roughly (0.19,0.14) of a scaled face image.
    const double DESIRED_RIGHT_EYE_X = (1.0f - DESIRED_LEFT_EYE_X);

    // Get the amount we need to scale the image to be the desired fixed size we want.
    double desiredLen = (DESIRED_RIGHT_EYE_X - DESIRED_LEFT_EYE_X) * FaceWidth;
    double scale = desiredLen / len;

    // Get the transformation matrix for rotating and scaling the face to the desired angle & size.
    Mat rot_mat = getRotationMatrix2D(eyesCenter, angle, scale);

    // Shift the center of the eyes to be the desired center between the eyes.
    rot_mat.at<double>(0, 2) += FaceWidth * 0.5f - eyesCenter.x;
    rot_mat.at<double>(1, 2) += FaceHeight * DESIRED_LEFT_EYE_Y - eyesCenter.y;

    warped = Mat(FaceHeight, FaceWidth, CV_8U, Scalar(128));

    warpAffine(face, warped, rot_mat, warped.size());
}

void DrawMarker(Mat& dst, Rect rect, string msg, int LINE_WIDTH)
{
    Rect r = rect;
    Scalar DETECT_COLOR = CV_RGB(0, 255, 0);

    line(dst, Point(r.x, r.y), Point(r.x, r.y + LINE_WIDTH), DETECT_COLOR, 3);
    line(dst, Point(r.x, r.y), Point(r.x + LINE_WIDTH, r.y), DETECT_COLOR, 3);

    line(dst, Point(r.x + r.width, r.y), Point(r.x + r.width, r.y + LINE_WIDTH), DETECT_COLOR, 3);
    line(dst, Point(r.x + r.width, r.y), Point(r.x + r.width - LINE_WIDTH, r.y), DETECT_COLOR, 3);

    line(dst, Point(r.x, r.y + r.height), Point(r.x, r.y + r.height - LINE_WIDTH), DETECT_COLOR, 3);
    line(dst, Point(r.x, r.y + r.height), Point(r.x + LINE_WIDTH, r.y + r.height), DETECT_COLOR, 3);

    line(dst, Point(r.x + r.width, r.y + r.height), Point(r.x + r.width, r.y + r.height - LINE_WIDTH), DETECT_COLOR, 3);
    line(dst, Point(r.x + r.width, r.y + r.height), Point(r.x + r.width - LINE_WIDTH, r.y + r.height), DETECT_COLOR, 3);

    int font = FONT_HERSHEY_DUPLEX;
    Size s = getTextSize(msg, font, 1, 1, 0);

    int x = (dst.cols - s.width) / 2;
    int y = rect.y + rect.height + s.height + 5;

    putText(dst, msg, Point(x, y), font, 1, CV_RGB(255, 0, 0), 1, CV_AA);
}

bool Init()
{

    if (!face_cascade.load("/home/alvarovito/catkin_ws/src/augmentedVision/src/data/haarcascade_frontalface_alt.xml"))
    {
        cout << "No se encuentra el archivo haarcascade_frontalface_alt_tree.xml" << endl;
        return false;
    }

    if (!eyeDetector.load("/home/alvarovito/catkin_ws/src/augmentedVision/src/data/haarcascade_eye_tree_eyeglasses.xml"))
    {
        cout << "No se encuentra el archivo haarcascade_eye_tree_eyeglasses.xml" << endl;
        return false;
    }

    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "faceDetector_nodo");
    ros::NodeHandle nodo;
    ROS_INFO("Nodo faceDetector creado y registrado");

    /*if (!face_cascade.load("/home/alvarovito/catkin_ws/src/augmentedVision/src/data/haarcascade_frontalface_alt.xml")) {
        cout << "Cannot load face xml!" << endl;
        return -1;
    }*/

    Ptr<FaceRecognizer> model = createLBPHFaceRecognizer();
    vector<Mat> rostros;
    vector<int> ids;
    map<int, string> names;

    bool entrenamiento = false;
    bool agregarRostro = false;
    bool entrenar = false;
    int identificador = 0, capCount = 0;

    string msg1 = "Reconocimiento Facial \n\n\t[E] Iniciar Entrenamiento \n\t[ESC] Salir\n";
    string msg2 = "Reconocimiento Facial \n\n\t[A] Capturar Rostro \n\t[T] Finalizar Entrenamiento \n\t[ESC] Salir\n";
    cout << msg1;

    if (!Init()){
        cout << "error al cargas los archivos .xml" << endl;
        return 1;
    }


    Rect lEye, rEye;
    Mat nface;

    ros::Subscriber subscriptor = nodo.subscribe("imagenesCapturadas", 0, updateImages);

    ros::Publisher publicadorMensajes = nodo.advertise<augmentedVision::rectanglesVector>("faces", 0);
    augmentedVision::rectanglesVector msjEnviar;

    namedWindow("Face Detection", WINDOW_AUTOSIZE);

    ros::Duration rate(0.05);

    while(ros::ok() && !cerrar)
    {
        if(!leftFrame.empty())
        {
            msjEnviar.rectanglesLeft.clear();

            detectFaces(leftFrame);
            int n = faces.size();

            if (entrenamiento) 
            {
                if (n == 1 && EyeDetection(workingImage, faces[0], lEye, rEye)) 
                {
                    CropFace(workingImage(faces[0]), nface, lEye, rEye);

                    if (agregarRostro)
                    {
                        Mat fface;

                        flip(nface, fface, 1);
                        rostros.push_back(fface);
                        ids.push_back(identificador);

                        rostros.push_back(nface);
                        ids.push_back(identificador);
                        agregarRostro = false;

                        capCount += 1;
                        cout << "Se han capturado " << capCount << " Rostros" << endl;
                    }

                    if (entrenar && rostros.size() >= 1)
                    {
                        model->update(rostros, ids);

                        cout << "\nNombre de la persona: ";
                        cin >> names[identificador];
                        system("cls");

                        entrenar = agregarRostro = entrenamiento = false;
                        rostros.clear();
                        ids.clear();
                        identificador += 1;
                        capCount = 0;

                        cout << msg1;
                    }
                }
            }

            //Mat Rois

            for (size_t i = 0;i < faces.size(); i++) 
            {
                augmentedVision::rectangle rec;
                rec.x=faces[i].x;
                rec.y=faces[i].y;
                rec.width=faces[i].x + faces[i].width;
                rec.height=faces[i].y + faces[i].height;
                msjEnviar.rectanglesLeft.push_back(rec);
            }
            //Add rois to image

            
            imshow("Face Detection", leftFrame);
        }
        if(!rightFrame.empty())
        {
            msjEnviar.rectanglesRight.clear();

            detectFaces(rightFrame);
            int n = faces.size();

            for (size_t i = 0;i < faces.size(); i++) 
            {
                augmentedVision::rectangle rec;
                rec.x=faces[i].x;
                rec.y=faces[i].y;
                rec.width=faces[i].x + faces[i].width;
                rec.height=faces[i].y + faces[i].height;
                msjEnviar.rectanglesRight.push_back(rec);
            }
        }
        publicadorMensajes.publish(msjEnviar);

        switch (waitKey(30))
        {
        case 'T':
        case 't':
            entrenar = true;
            break;
        case 'A':
        case 'a':
            agregarRostro = entrenamiento;
            break;
        case 'E':
        case 'e':
            entrenamiento = true;
            system("cls");
            cout << msg2 << endl;
            break;
        
        case 32:

            if (holdImage)
                holdImage = false;
            else
                holdImage = true;
            break;

        default:
            break;
        
        case 27:
            return 0;
        }
        

        ros::spinOnce();
        rate.sleep();
    }
    
    leftFrame.release();
    rightFrame.release();
    workingImage.release();

    destroyAllWindows();

    return 0;
}
