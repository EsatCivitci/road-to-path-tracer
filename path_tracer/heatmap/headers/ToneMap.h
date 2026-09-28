#include "Real.h"

#include <iostream>
#include <string>

struct TM {
    std::string TMO = "";
    real key = 0.18;
    real burn_out = 1;
    real saturation = 1;
    real gamma = 2.2;
    std::string extension = "";
};

class ToneMap {
public:
    ToneMap() = default;
    ~ToneMap() = default;

    void applyToneMapping(const std::vector<real>& data, std::string name, int nx, int ny);

    void addTM(TM tm) { tm_list.push_back(tm); }

    bool isActive() { return tm_list.size() > 0; }

private:    
    real mapACES(real L);
    real mapFilmic(real L);
    Vec3r operateColor(real L_d, real L_w, real saturation, real gamma, real R, real G, real B);
    void computeL(real key, real L_w_prime,const std::vector<real>& L_w, std::vector<real>& L);
    void computeL_w(const std::vector<real> data, std::vector<real>& L_w);
    real computeL_white(const std::vector<real>& L, real burn_out);
    real computeL_w_prime(const std::vector<real>& L_w);

    std::vector<TM> tm_list;

    real A = 2.51;
    real B = 0.03;
    real C = 2.43;
    real D = 0.59;
    real E = 0.14;

    real a = 0.22;
    real b = 0.3;
    real c = 0.1;
    real d = 0.2;
    real e = 0.01;
    real f = 0.3;
};
