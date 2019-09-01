//
// Created by xc on 19-5-28.
//
#include "easy_IO_exif.h"
#include "exif.h"

#include <fstream>
#include <limits>
#include <sstream>
#include <iostream>
#include <algorithm>

namespace exifExtract{

    bool easy_IO_exif::paseFile(std::string &imageFile)  {

        FILE *fp=fopen(imageFile.c_str(),"rb");
        if(!fp)
        {
            return false;
        }

        fseek(fp,0,SEEK_END);

        unsigned long fileSize=ftell(fp);

        rewind(fp);

        std::vector<unsigned char>buf(fileSize);

        if ( fread( &buf[0], 1, fileSize, fp ) != fileSize )
        {
            fclose( fp );
            return false;
        }
        fclose( fp );

        isHaveExifInfo = (exif_info.parseFrom( &buf[0], fileSize ) == PARSE_EXIF_SUCCESS );

        return isHaveExifInfo;
    }

    size_t easy_IO_exif::imgWidth() const {

        return exif_info.ImageWidth;
    }
    size_t easy_IO_exif::imgHeight() const {

        return exif_info.ImageHeight;
    }
    std::string easy_IO_exif::cameraBrand() const {

        return exif_info.Make;
    }
    std::string easy_IO_exif::cameraModels() const {

        return exif_info.Model;
    }

    float easy_IO_exif::imgFocal() const {

        return static_cast<float>(exif_info.FocalLength);
    }

    bool easy_IO_exif::GPSlatitude(double  & latitude) const {

        if(exif_info.GeoLocation.Latitude!=std::numeric_limits<double>::infinity())
        {
           latitude=exif_info.GeoLocation.Latitude;
           return true;
        }

        return false;
    }

    bool easy_IO_exif::GPSaltitude(double &altitude) const {

        if(exif_info.GeoLocation.Altitude!=std::numeric_limits<double>::infinity())
        {
            altitude=exif_info.GeoLocation.Altitude;
            return true;
        }

        return false;
    }

    bool easy_IO_exif::GPSlongitude(double &longitude) const {

        if(exif_info.GeoLocation.Longitude!=std::numeric_limits<double>::infinity())
        {
            longitude=exif_info.GeoLocation.Longitude;
            return true;
        }

        return false;
    }
    void easy_IO_exif::sensor_Info::loadSensor(std::string sensorFile,
                                               std::vector<exifExtract::easy_IO_exif::sensor_Info> &res) const {
        std::ifstream infile;

        infile.open(sensorFile.c_str());

        if(!infile.is_open())
        {
            std::cerr<<" sensorFile 读取失败！" <<std::endl;
        }

        std::string str;

        while(getline(infile,str))
        {
            res.emplace_back(str.substr(0,str.find(';')),std::atof(str.substr(str.find(';')+1).c_str()));
        }
    }


    bool easy_IO_exif::sensor_Info::operator==(const exifExtract::easy_IO_exif::sensor_Info &rhs) const {

    }


}