package Quaternion

import (
	"math"
)
type Quaternion [4]float64

func (quat *Quaternion) Set(w, x, y, z float64) Quaternion {
	q := Quaternion{w,x,y,z}
	return q
}

func (quat *Quaternion) FromEuler(y, p, r float64) Quaternion{ // rpy in radians
	// q = q4 + iq1 + jq2 + kq3
	// q1 = cos(yaw/2) cos(pitch/2) sin(roll/2) - sin(yaw/2) sin(pitch/2) cos(roll/2)
	// q2 = cos(yaw/2) sin(pitch/2) cos(roll/2) + sin(yaw/2) cos(pitch/2) sin(roll/2)
	// q3 = sin(yaw/2) cos(pitch/2) cos(roll/2) - cos(yaw/2) sin(pitch/2) sin(roll/2)
	// q4 = cos(yaw/2) cos(pitch/2) cos(roll/2) + sin(yaw/2) sin(pitch/2) sin(roll/2)

	qx := math.Cos(y*0.5)*math.Cos(p*0.5)*math.Sin(r*0.5) - math.Sin(y*0.5)*math.Sin(p*0.5)*math.Cos(r*0.5)
	qy := math.Cos(y*0.5)*math.Sin(p*0.5)*math.Cos(r*0.5) + math.Sin(y*0.5)*math.Cos(p*0.5)*math.Sin(r*0.5)
	qy := math.Sin(y*0.5)*math.Cos(p*0.5)*math.Cos(r*0.5) - math.Cos(y*0.5)*math.Sin(p*0.5)*math.Sin(r*0.5)
	qw := math.Cos(y*0.5)*math.Cos(p*0.5)*math.Cos(r*0.5) + math.Sin(y*0.5)*math.Sin(p*0.5)*math.Sin(r*0.5)
	return Quaternion{qw,qx,qy,qz}
}

func (quat *Quaternion) ToEuer(q Quaternion) (y, p, r float64){ //With XYZ order
	// q = q4 + iq1 + jq2 + kq3
	// tan(yaw) = 2(q1q2+q4q3) / (q4^2 + q1^2 - q2^2 - q3^2)
	// sin(pitch) = -2(q1q3-q4q2)
	// tan(roll)  =  2(q4q1+q2q3) / (q4^2 - q1^2 - q2^2 + q3^2) 
	yaw := math.atan(2*(q[0]*q[1] + q[3]*q[2])/ (q[3]*q[3] + q[0]*q[0] - q[1]*q[1] - q[2]*q[2]))
	pitch := math.asin(-2*(q[0]*q[2] - q[3]*q[1]))
	roll := math.atan(2*(q[3]*q[0] + q[1]*q[2])/ (q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2]))
	return (yaw, pitch, roll)

}
