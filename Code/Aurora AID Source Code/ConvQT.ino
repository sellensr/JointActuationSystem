// Needs to be set appropriately to reduce errors when rotation matrix gets to be near symmetric.
// A value of 1.0e-6 seems a good compromise for UNO etc. based on the ATMEGA 328
// A value more like 1.0e-12 is better for ARM based processors
#define epsilon  1.0e-12
#define pi 3.141592654

void rQuat(double r[4][4],double q[4]){
  // get the quaternion that matches the rotation matrix portion of the input transform.
  // About 5 orders of magnitude more robust than Shoemake when running as double on an ARM processor
  // Similar improvements when running as de facto float on an ATMEGA processor.  
  // function q=r2q(R)
  // Translated from MatLab code by Rick Sellens 2015-11-27
  //% Q=R2Q(R) finds the unit quaternion corresponding to the rotation matrix R.
  //% Randy Ellis, retreived Nov 6 2014
  double angle = acos((r[0][0] + r[1][1] + r[2][2] - 1.0d)/2.0d);
  double axis[3] = {0.0d, 0.0d, 0.0d};
  double qvec[3] = {0.0d, 0.0d, 0.0d};
  double norm;
  if(abs(angle) > epsilon){
    axis[0] = r[2][1] - r[1][2];     axis[1] = r[0][2] - r[2][0];    axis[2] = r[1][0] - r[0][1];
    norm = sqrt( axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    axis[0] /= norm; axis[1] /= norm; axis[2] /= norm; 
    qvec[0] = r[2][1] - r[1][2];     qvec[1] = r[0][2] - r[2][0];    qvec[2] = r[1][0] - r[0][1];
  }else{
    axis[0] = axis[1] = axis[2] = 0.0d;
  }
  double q0 = sqrt(1.0d + r[0][0] + r[1][1] + r[2][2]) / 2.0d;
  q[0] = q0; 
  double d = 4.0d * q0;
  q[1] = qvec[0]/d; q[2] = qvec[1]/d; q[3] = qvec[2]/d; 
  norm = sqrt( q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
}

void qRot(double q[4],double out[4][4]){
  // Get the reverse rotation matrix from an input quaternion
  // This is the rotation matrix to get from the frame described by the input quaternion
  // back to the original frame in which the quaternion is expressed.
  // Take the transpose to get the opposite rotation.
  // following http://run.usc.edu/cs520-s12/assign2/p245-shoemake.pdf
  // Animating rotation with quaternion curves. Ken Shoemake Computer Graphics 19(3):245-254, 1985
  doubleT(mid);     // the rotation as given in the paper, which is the reverse of the quaternion
  double x22 = 2.0d * q[1] * q[1];
  double y22 = 2.0d * q[2] * q[2];
  double z22 = 2.0d * q[3] * q[3];
  double xy2 = 2.0d * q[1] * q[2];
  double xz2 = 2.0d * q[1] * q[3];
  double yz2 = 2.0d * q[3] * q[2];
  double wx2 = 2.0d * q[0] * q[1];
  double wy2 = 2.0d * q[0] * q[2];
  double wz2 = 2.0d * q[0] * q[3];
  mid[0][0] = 1.0d - y22 -z22;
  mid[0][1] = xy2 + wz2;
  mid[0][2] = xz2 - wy2;
  mid[1][0] = xy2 - wz2;
  mid[1][1] = 1.0d - x22 - z22;
  mid[1][2] = yz2 + wx2;
  mid[2][0] = xz2 + wy2;
  mid[2][1] = yz2 - wx2;
  mid[2][2] = 1.0d - x22 - y22;
  transposeT(mid,out);
}

void rQuatRShoemake(double in[4][4],double q[4]){
  // fill in the reverse quaternion q based on the rotation matrix portion of the transform
  // following http://run.usc.edu/cs520-s12/assign2/p245-shoemake.pdf
  // Animating rotation with quaternion curves. Ken Shoemake Computer Graphics 19(3):245-254, 1985
  // Fails if there is symmetry or near symmetry about the diagonal! Close to 180 degree rotation
  // USE SOMETHING ELSE!!! Just here for history and comparison purposes
  double w2 = 0.25d * (1.0d + in[0][0] + in[1][1] + in[2][2]);
  if(w2 > epsilon){
    q[0] = sqrt(w2);
    q[1] = (in[1][2]-in[2][1])/4.0d/q[0];
    q[2] = (in[2][0]-in[0][2])/4.0d/q[0];
    q[3] = (in[0][1]-in[1][0])/4.0d/q[0];
  } else{
    q[0] = 0.0d;
    double x2 = -0.5d * (in[1][1]+in[2][2]);
    if(x2 > epsilon){
      q[1] = sqrt(x2);
      q[2] = in[0][1] / 2.0d / q[1];
      q[3] = in[0][2] / 2.0d / q[1];
    } else{
      q[1] = 0.0d;
      double y2 = 0.5d * (1.0d - in[2][2]);
      if(y2 > epsilon){
        q[2] = sqrt(y2);
        q[3] = in[1][2] / 2.0d / q[2];
      } else{
        q[2] = 0.0d;
        q[3] = 1.0d;
      }
    }
  }
}

void rEuler(double in[4][4], double eu1[3], double eu2[3]){
  // follows pseudocode of http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf
  // to calculate a vector of Euler angles from a rotation matrix.
  // eu vectors are in order psi, theta, pi
  if(abs(in[2][0]) != 1){
    eu1[1] = -asin(in[2][0]);
    eu2[1] = pi - eu1[1];
    double cost1 = cos(eu1[1]); double cost2 = cos(eu2[1]);
    eu1[0] = atan2(in[2][1]/cost1,in[2][2]/cost1);
    eu2[0] = atan2(in[2][1]/cost2,in[2][2]/cost2);
    eu1[2] = atan2(in[1][0]/cost1,in[0][0]/cost1);
    eu2[2] = atan2(in[1][0]/cost2,in[0][0]/cost2);
  }else{
    eu1[2] = eu2[2] = 0.0;
    if(in[2][0] < 0.0){
      eu1[1] = eu2[1] = pi/2.0;
      eu1[0] = eu2[0] = atan2(in[0][1],in[0][2]);
    }else{
      eu1[1] = eu2[1] = -pi/2.0;
      eu1[0] = eu2[0] = atan2(-in[0][1],-in[0][2]);
    }
  }
}
