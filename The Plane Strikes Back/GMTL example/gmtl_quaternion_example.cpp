//|___________________________________________________________________
//|
//| GMTL example showing:
//|   - Using explicit homogeneous form for point and vector
//|   - How to convert a rotation in angle-axis representation to matrix representation or quaternion representation
//|   - Using quaternion representation to rotate point and vector (see Lecture Note)
//|___________________________________________________________________

#include <iostream>
#include <gmtl/gmtl.h>

int main()
{
  // Variations of the same point
  gmtl::Point3f p1(5, -3, 7);    // Implicit homogeneous form (forth element is set to 1 implicitly by definition of point)
  gmtl::Point4f p2(5, -3, 7, 1); // Explicit homogeneous form


//|___________________________________________________________________
//|
//| Compute rotation in various forms to be used later
//|___________________________________________________________________
  
  // Specify an angle-axis, aa
  gmtl::Vec3f rot_axis(1, 1, 1);
  gmtl::normalize( rot_axis );
  gmtl::AxisAnglef aa(gmtl::Math::deg2Rad(45.0f), rot_axis);  

  // Convert aa to matrix form, rot_mat
  gmtl::Matrix44f rot_mat;
  gmtl::set(rot_mat, aa);

  // Convert aa to quaternion form, q
  gmtl::Quatf q;
  gmtl::set(q, aa);


//|___________________________________________________________________
//|
//| You can use Point4f in place of Point3f
//|___________________________________________________________________

  // Set translation matrix
  gmtl::Matrix44f tran_mat = gmtl::makeTrans<gmtl::Matrix44f>( gmtl::Vec3f(-1.0f, 5.0f, -2.0f) );

  // Check that Point4f has the same result as Point3f
  std::cout << "Point3f after matrix transform = " << rot_mat*tran_mat*p1 << std::endl;
  std::cout << "Point4f after matrix transform = " << rot_mat*tran_mat*p2 << std::endl << std::endl;


//|___________________________________________________________________
//|
//| You can use quaternion to compute rotated point or vector
//|___________________________________________________________________

  std::cout << "p1 after matrix rotation = " << rot_mat*p1 << std::endl;

  // Compute rotated point using quaternion
  gmtl::Quatf q_con = gmtl::makeConj(q); // Find complex conjugate of q
  gmtl::Quatf p1_q = q*gmtl::Quatf(p1[0], p1[1], p1[2], 0)*q_con; // Rotated point in quaternion representation (see Lecture Note)
  std::cout << "p1 after quaternion rotation = " << gmtl::Point3f(p1_q[0], p1_q[1], p1_q[2]) << std::endl << std::endl; 


//|___________________________________________________________________
//|
//| Local translation with p-q pair (pose; p is "p2" and q is "q" from the above)
//|___________________________________________________________________

  // Local translation vector (defined in p-q frame; note the explicit homogeneous form; implicit homogeneous form would be gmtl::Vec3f v(7, -6, 1) where the last element is set to 0 implicitly)
  gmtl::Vec4f v(7, -6, 1, 0); // last element is 0 by definition of vector; vector transform will be invariant to translation.

  // Use quaternion to compute the translation vector in parent frame of p-q frame
  gmtl::Quatf v_q = q*gmtl::Quatf(v[0], v[1], v[2], 0)*q_con; 
  std::cout << "p2 after local translation (using quaternion) = " << p2 + v_q.mData << std::endl;

  // Check results with matrix computation
  gmtl::Vec4f v_m = rot_mat*v;
  std::cout << "p2 after local translation (using matrix) = " << p2 + v_m << std::endl;


  return 0;
}