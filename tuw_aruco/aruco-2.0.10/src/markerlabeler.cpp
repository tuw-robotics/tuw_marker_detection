#include "markerlabeler.h"
#include "markerlabelers/svmmarkers.h"
#include "markerlabelers/dictionary_based.h"
namespace aruco{
cv::Ptr<MarkerLabeler> MarkerLabeler::create(Dictionary::DICT_TYPES dict_type,float error_correction_rate)throw (cv::Exception)
{
    Dictionary dict=Dictionary::loadPredefined(dict_type);
    DictionaryBased *db=new DictionaryBased();
    db->setParams(dict,error_correction_rate);
    return db;

}


cv::Ptr<MarkerLabeler> MarkerLabeler::create(std::string detector,std::string params)throw (cv::Exception){


    if (detector=="SVM"){
        SVMMarkers *svm=new SVMMarkers;
        if (!svm->load( params)) throw cv::Exception( -1,"Could not open svm file :"+params,"Detector::create"," ",-1 );
        //*SVMmodel,dictsize, -1, 1, true);
        return svm;
    }
    else{
        Dictionary dict=Dictionary::loadPredefined(detector);
        DictionaryBased *db=new DictionaryBased();
        db->setParams(dict,0);
        return db;
    }

    throw cv::Exception( -1,"No valid labeler indicated:"+detector,"Detector::create"," ",-1 );

}


}
