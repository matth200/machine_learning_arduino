#include <iostream>
#include <filesystem>
#include <string>
#include <regex>
#include <fstream>
#include "src/m_learning.h"

#define TRAINMODEL_FOLDER "../resources/trained_model/"
#define SAVE_FILE "../resources/extracted/data.txt"

namespace fs = std::filesystem;
using namespace std;



std::ifstream::pos_type filesize(const char* filename)
{
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    return in.tellg(); 
}


int main(int argc, char **argv){
    string filename = "";
    if(argc==2){
        cout << "Extraction du réseau de neurones pour l'envoyer dans l'arduino" << endl;
        filename = argv[1];
        cout << "fichier:" << filename << endl;
    }else{
        cout << "Argument error" << endl;
        cout << argv[0] << " path_to_neuralnetwork.ml" << endl;
        return 1;
    }
    cout << "Chargement du réseau de neurones:"<< filename << endl;
    MachineLearning machine;

    if(!machine.loadStructure(filename.c_str())){
        cout << "Erreur lors de la récupération de la structure" << endl;
    }
    machine.backupTraining(filename.c_str());

    filename = "../resources/tmp_model_arduino/light.ml";
    machine.saveTrainingArduino(filename.c_str());

    cout << "Ouverture du fichier..." << endl; 
    ifstream file(filename, ios::binary);
    if(file.is_open()){
        string data = "";
        cout << "Code arduino:" <<endl;
        int size = filesize(filename.c_str());
        data="const PROGMEM char data["+to_string(size)+"]={";
        char buffer = 0;
        for(int i=0;i<size;i++){
            file.read(&buffer,1);
            data+=to_string(int(buffer));
            if(i!=size-1){
                data+=",";
            }
        }
        data+="};\n";
        cout << data;
        
        cout << "Enregistrement dans le fichier " << SAVE_FILE << endl;
        ofstream save_file(SAVE_FILE);
        if(!save_file.is_open()){
            cout << "Erreur lors de l'ouverture du fichier" << endl;
            return 1;
        }
        save_file << data;
        cout << "Enregistrement effectué avec succés" << endl;
    }else{
        cout << "Fichier inexistant" << endl;
        return 1;
    }

    return 0;
}