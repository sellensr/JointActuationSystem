// Note that on UNO and similar, double is the same as float, 4 bytes, so the precision will be crummy.
// Switching to an ARM processor like on the DUE or the Teensy 3.0 provides actual 8 byte doubles and things get much better.
// Also much better on the M0. 
/*
 * A transform matrix is a 4x4 matrix of doubles of the form
 * {i1 dot i0, j1 dot i0, k1 dot i0, Dx,
 *  i1 dot j0, j1 dot j0, k1 dot j0, Dy,
 *  i1 dot k0, j1 dot k0, k1 dot k0, Dz,
 *     0.0,       0.0,       0.0,    1.0} 
 * that defines the transformation from frame 1 to frame 0    
 * The macro doubleT(a) declares and initializes transforms to identity and should help avoid unitialized matrix errors.
 */
#define doubleT(a) double a[4][4] = {1.d, 0.d, 0.d, 0.d, 0.d, 1.d, 0.d, 0.d, 0.d, 0.d, 1.d, 0.d, 0.d, 0.d, 0.d, 1.d}

// doubleQ declares a quaternion of the form {q0, qx, qy, qz}, initialized to no rotation
#define doubleQ(a) double a[4] = {1.0d, 0.0, 0.0, 0.0}

// doubleV declares a vector of the form {0, vx, vy, vz}, initialized to unit size, 45 degrees from each axis
#define doubleV(a) double a[4] = {0.0d, 0.5774d, 0.5774d, 0.5774d}

void transformT(double T[4][4],double Vin[4],double Vout[4]){
  // Apply T to transform Vin to Vout
  for(int i = 0;i < 3;i++){
    Vout[i+1] = T[i][3];
    for(int j = 0 ;j < 3;j++) Vout[i+1] += T[i][j] * Vin[j+1];
  }
}

double detT(double t[4][4]){
  // determinant of the rotation part of a transform matrix
  // det( a b c
  //      d e f
  //      g h i ) = aei + bfg + cdh - ceg - bdi - afh
  return    t[0][0]*t[1][1]*t[2][2] 
          + t[0][1]*t[1][2]*t[2][0] 
          + t[0][3]*t[1][0]*t[2][1]
          - t[0][2]*t[1][1]*t[2][0]
          - t[0][1]*t[1][0]*t[2][2]
          - t[0][0]*t[1][2]*t[2][1];
}

void transposeT(double in[4][4],double out[4][4]){
  // transpose the rotation part of the in matrix into the out matrix 
  for(int i = 0;i<3;i++) for(int j = 0;j < 3;j++) out[i][j] = in[j][i];
  // invert the displacement
  for(int i = 0; i < 3;i++){ 
    out[i][3] = in[i][3] * -1.0d;
    out[3][i] = 0.0; // the bottom row
  }
  out[3][3] = 1.0d;
}

void multT(double A[4][4],double B[4][4],double AB[4][4]){
  // multiply transform matrices A and B and store the result in AB
  for(int i = 0;i < 4;i++){
    for(int j = 0;j < 4;j++){
      AB[i][j] = 0.0;
      for(int k = 0;k < 4;k++) AB[i][j] += A[i][k] * B[k][j];
    } 
  }
}

void copyT(double A[4][4],double B[4][4]){
  // copy transform matrix A to B
  for(int i = 0;i < 4;i++) for(int j = 0;j < 4;j++) B[i][j] = A[i][j];
}

void rotateQ(double q[4], double v[4], double v1[4]){
  // apply quaternion q to rotate vector v to get vector v1 
  // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/
  // Vout = q * Vin * conj(q)
  doubleQ(qc);
  doubleQ(mid);
  conjQ(q,qc);

  multQ(q,v,mid);
  multQ(mid,qc,v1);

  multQ(v,qc,mid);
  multQ(q,mid,v1);
}

void multQ(double q[4],double r[4],double qr[4]){
  // multiply quaternions Q and R and store the result in QR
  // http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html?requestedDomain=www.mathworks.com
  qr[0] = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3];
  qr[1] = r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2];
  qr[2] = r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1];
  qr[3] = r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0];
}

void divQ(double q[4],double r[4],double qor[4]){
  // divide quaternions Q over R and store the result in QoR
  // http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html?requestedDomain=www.mathworks.com
  // http://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf
  double rr = r[0]*r[0] +  r[1]*r[1] + r[2]*r[2] + r[3]*r[3];
  qor[0] = (r[0] * q[0] + r[1] * q[1] + r[2] * q[2] + r[3] * q[3]) / rr;
  qor[1] = (r[0] * q[1] - r[1] * q[0] - r[2] * q[3] + r[3] * q[2]) / rr;
  qor[2] = (r[0] * q[2] + r[1] * q[3] - r[2] * q[0] - r[3] * q[1]) / rr;
  qor[3] = (r[0] * q[3] - r[1] * q[2] + r[2] * q[1] - r[3] * q[0]) / rr;
}

void conjQ(double q[4],double qc[4]){
  // make qc the conjugate of quaternion q
  qc[0] = q[0];
  qc[1] = -q[1];
  qc[2] = -q[2];
  qc[3] = -q[3];
}

void copyQ(double q[4],double qn[4]){
  // make qnew the copy of quaternion q
  for(int i=0;i<4;i++) qn[i] = q[i];
}




