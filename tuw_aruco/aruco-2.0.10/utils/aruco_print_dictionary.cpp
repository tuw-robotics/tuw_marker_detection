//saves all the images of the dictionary indicated to a dicrectory

#include "dictionary.h"
#include <opencv2/highgui/highgui.hpp>
#include <string>
using namespace std;

int main(int argc,char **argv){

    if(argc<=3){
        cerr << "Usage:  dictionary  outdir [bit_image_size: 75 default]  " << endl;
        auto dict_names=aruco::Dictionary::getDicTypes();
        cerr<<"\t\tDictionaries: ";
        for(auto dict:dict_names)    cerr<<dict<<" ";cerr<<endl;
        return -1;
    }
    aruco::Dictionary dict=aruco::Dictionary::loadPredefined( argv[1]);
    int pixSize =75;
    if (argc>=4) pixSize = std::stoi(  argv[3]);

    string dict_name=aruco::Dictionary::getTypeString(dict.getType());
    std::transform(dict_name.begin(), dict_name.end(), dict_name.begin(), ::tolower);
    //
    for(auto m:dict.getMapCode())   {
        string number=std::to_string(m.second);
        while(number.size()!=5) number="0"+number;
        stringstream name;
        name<<argv[2]<<"/"+dict_name+"_"<<number<<".png";
        cout<<name.str()<<endl;
        cv::imwrite(name.str(), dict.getMarkerImage_id(m.second, pixSize));

    }
}
