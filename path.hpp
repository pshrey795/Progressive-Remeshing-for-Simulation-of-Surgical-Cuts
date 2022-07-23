#ifndef PATH_HPP
#define PATH_HPP

#define TIME_TO_PARAM_FACTOR 15.0

#include "curve.hpp"
using namespace std;

class Path {
    public:
        int size;
        vector<Curve*> curves; 
        int currentCurve = 0;
        float currParamVal = 0;
        bool isOver = false;
        vec3 lastPoint;
        vec3 lastTangent;
        Path(){
            this->size = 0; 
            this->currentCurve = 0;
            this->currParamVal = 0.0f; 
        }        
        void addCurve(vector<vec3> inputPts){
            this->curves.push_back(new Curve(inputPts,0));
            if(size == 0){
                this->lastPoint = this->curves[0]->getPoint(0);
                this->lastTangent = this->curves[0]->getTangent(0);
            }   
            this->size++;
        }
        void updatePath(){
            if(currentCurve < size){
                this->currParamVal += 0.125;
                if(double_eq(this->currParamVal,1.0f)){
                    this->lastPoint = this->curves[currentCurve]->getPoint(1.0f);
                    this->lastTangent = this->curves[currentCurve]->getTangent(1.0f);
                    this->currParamVal = 0.0f;
                    this->currentCurve++;
                    if(currentCurve == size){
                        this->isOver = true;
                    }
                }else{
                    this->lastPoint = this->curves[currentCurve]->getPoint(this->currParamVal);
                    this->lastTangent = this->curves[currentCurve]->getTangent(this->currParamVal);
                }
            }
        }    

};

#endif