#include <QWidget>

#ifdef QCOM2
  const int vwp_w = 2160, vwp_h = 1080;
#else
  // const int vwp_w = 1920, vwp_h = 1080;
  const int vwp_w = 1164, vwp_h = 874;
#endif

void setMainWindow(QWidget *w);
