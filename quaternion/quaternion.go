package Quaternion

import (
	"math"
)
type Quaternion [4]float64

func (quat *Quaternion) Set(x, y, z, w float64) Quaternion {
	q := Quaternion{x,y,z,w}
	return q
}

func (quat *Quaternion) FromEuler(r, p, y float64) Quaternion{ // rpy in radians
	qx := math.Cos(r*0.5)*math.Cos(p*0.5)*math.Cos(y*0.5) + math.Sin(r*0.5)*math.Sin(p*0.5)*math.Sin(y*0.5)
	qy := math.Sin(r*0.5)*math.Cos(p*0.5)*math.Cos(y*0.5) - math.Cos(r*0.5)*math.Sin(p*0.5)*math.Sin(y*0.5)
	qz := math.Cos(r*0.5)*math.Sin(p*0.5)*math.Cos(y*0.5) + math.Sin(r*0.5)*math.Cos(p*0.5)*math.Sin(y*0.5)
	qw := math.Cos(r*0.5)*math.Cos(p*0.5)*math.Sin(y*0.5) - math.Sin(r*0.5)*math.Sin(p*0.5)*math.Cos(y*0.5)
	return Quaternion{qx,qy,qz,qw}
}
