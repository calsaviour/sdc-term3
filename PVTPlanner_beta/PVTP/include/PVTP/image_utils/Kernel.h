#ifndef __KERNEL_H__
#define __KERNEL_H__

#include "DTwoDimArray.h"
#include <string.h>

// A very simple kernel class; basically just a copy of SImage.h
//
class Kernel : public _DTwoDimArray<double>
{
 public:
  Kernel() { }
  Kernel(int _rows, int _cols)  : _DTwoDimArray<double>(_rows, _cols)
    {
      // be nice and initialize plane to all 0's
      memset(data_ptr(), 0, sizeof(double) * rows() * cols());
    }


};

#endif
