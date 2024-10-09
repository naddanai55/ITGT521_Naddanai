/** This example shows how to use GMTL for calculations using vectors, points, and matrices */

#include <iostream>

// Includes for GMTL: Method 1 - include only what are used; or
//#include <gmtl/Point.h>
//#include <gmtl/Vec.h>
//#include <gmtl/Matrix.h>
//#include <gmtl/Generate.h> // for makeRot(), makeTrans(), etc.
//#include <gmtl/Output.h>   // for << operator

// Method 2 - use the main header that includes everything for you
#include <gmtl/gmtl.h>

int main(int argc,char **argv)
{
  gmtl::Point3f point(5, 0, 0);    // intializing a point
  gmtl::Vec3f   vector(1, 0, 0);   // intializing a vector


  // *** Set matrices using utility functions *** //


  // create a rotation matrix from 3 euler angles that rotate 90 degrees about Y-axis
  gmtl::Matrix44f rotY   = gmtl::makeRot<gmtl::Matrix44f>( gmtl::EulerAngleXYZf(0.f, gmtl::Math::deg2Rad(90.f), 0.f) );
  // create a translation matrix, translating 1 unit along +X, from a vector 
  gmtl::Matrix44f transX = gmtl::makeTrans<gmtl::Matrix44f>( gmtl::Vec3f(1.f, 0.f, 0.f) );

  // transforming the point, first by rotating and then by translating it. 
  // try changing the order and observe the result.
  gmtl::Point3f result   = transX*rotY*point;											
  std::cout << "| point | " << point << std::endl;
  std::cout << "| Result | " << result << std::endl;

  // translating a vector
  gmtl::Point3f transVector = transX*vector;
  std::cout << "| vector | " << vector << std::endl;
  std::cout << "| After translating a vector | " << transVector << std::endl; // translation does not affect a vector


  // *** Set matrix elements manually *** //
 

  // intialize a matrix with 90-degree rotation about Y-axis, with elements entered in row-major order
  gmtl::Matrix44f xformMatrix;
  xformMatrix.set( 0/*cos(90)*/,  0, 1/*sin(90)*/, 0,
                   0,             1, 0,            0,
                  -1/*-sin(90)*/, 0, 0/*cos(90)*/, 0,
                   0,             0, 0,            1);
  // elements of fourth column set to translation amount:
  xformMatrix[0][3] = 1.f; 
  xformMatrix[1][3] = 0.f; 
  xformMatrix[2][3] = 0.f;
  // set state of the matrix
  xformMatrix.setState( gmtl::Matrix44f::AFFINE );

  // print results
  gmtl::Point3f result2 = xformMatrix*point;
  std::cout << "| Result2 | " << result2 << std::endl;
  // smart inversion; think about how the matrix is inverted
  gmtl::invert(xformMatrix);
  gmtl::Point3f result3 = xformMatrix*point;
  std::cout << "| Result3 | " << result3 << std::endl;


  // *** Set matrix elements manually, second approach *** //


  // give GMTL a 16-element array with elements stored in column-major order
  const float mat_array[16] = {0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1}; // 90-degree Y rotation followed by 1 unit +X translation
  gmtl::Matrix44f xformMatrix2;
  xformMatrix2.set( mat_array ); 
  xformMatrix2.setState( gmtl::Matrix44f::AFFINE );

  // print results
  gmtl::Point3f result4 = xformMatrix2*point;
  std::cout << "| Result4 | " << result4 << std::endl;

  return 0;
}