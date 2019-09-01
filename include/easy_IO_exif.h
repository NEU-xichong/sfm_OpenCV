//
// Created by xc on 19-5-28.
//

#ifndef EXIFEXTRACT_EASY_IO_EXIF_H
#define EXIFEXTRACT_EASY_IO_EXIF_H

#include <memory>
#include <string>
#include <vector>
#include"exif.h"

namespace exifExtract {

class easy_IO_exif {
public:

    struct sensor_Info
    {

        sensor_Info()= default;

        sensor_Info(std::string model, double sensor):model_Name(model),sensor_(sensor){}

        /**
         *
         * @param sensorFile sensor的路径
         * @param res sensor路径下的文件加载到res中
         */
        void loadSensor(std::string  sensorFile,std::vector<sensor_Info>& res) const;


        /**
         *
         * @brief 运算符重载
         * @return
         */
        bool operator==(const sensor_Info& rhs)const;


        std::string model_Name;
        double sensor_;
    };


    /**
     *@brief 默认构造函数
        */
    easy_IO_exif() : isHaveExifInfo(false) {}




    /**
     * @param imageFile 图像的路径
     * @return 判断图片是否解析出exif信息
     */
    bool paseFile(std::string &imageFile) ;


    //void loadSensor(std::string sensorFile,std::vector<sensor_Info>& res) ;
    /**
     * @brief 获取图像的宽度
     * @return 相机的宽度
     */
    size_t imgWidth() const;

    /**
     * @brief 获取图像的高度
     * @return 图像的高度
     */
    size_t imgHeight() const;

    /**
     * @brief 获取焦距 （单位：mm）
     * @return 相机的焦距
     */
    float imgFocal() const;

    /**
     * @brief 获取相机的模型
     * @return 相机的模型名字
     */
    std::string cameraModels() const;

    /**
     * @brief 获取相机的牌子
     * @return 相机的牌子名字
     */
    std::string cameraBrand() const;


    /**
     *
     * @param latitude GPS的纬度值
     * @return 返回是否从EXIF中获取纬度值
     */
    bool GPSlatitude(double  & latitude) const;

    /**
     *
     * @param longitude GPS的经度值
     * @return 返回是否从EXIF中获取纬度值
     */
    bool GPSlongitude(double & longitude) const;

    /**
     *
     * @param altitude GPS的海拔高度
     * @return 返回是否从EXIF中获取海拔高度值
     */
    bool GPSaltitude(double & altitude) const;


private:
    easyexif::EXIFInfo exif_info;
    bool isHaveExifInfo;

};


}

#endif //EXIFEXTRACT_EASY_IO_EXIF_H
