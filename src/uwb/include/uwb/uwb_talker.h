#ifndef UWB_TALKER_H
#define UWB_TALKER_H



class kalf
{
  public:
    kalf()
    {
      Q = 0.018;//0.25*(a^2)*(t^4)
      R = 0.542;
    }
    float x_last[4];
    float p_last[4];
    float Q ;
    float R ;
    float kg[4];
    float x_mid[4];
    float x_now[4];
    float p_mid[4];
    float p_now[4];
};


#endif
