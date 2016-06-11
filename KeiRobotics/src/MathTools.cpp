/*
 * MathTools.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <MathTools.h>
#include <math.h>
#include <inttypes.h>
#include <stdio.h>
#include <AdditionalTools.h>
#include <Led.h>

using namespace Math;

float MathTools::PI = 3.14159f;
float MathTools::DEGREE_PER_RADIAN = 57.2958f;
float MathTools::RADIAN_PER_DEGREE = 0.01745f;

int MathTools::FloatToHalfInt(float _float){
	union{
		float f;
		uint32_t d;
	} x;
	x.f = _float;

	int fbits = x.d;
	int sign = fbits >> 16 & 0x8000;
	int val = ( fbits & 0x7fffffff ) + 0x1000;

	if( val >= 0x47800000 )
	{
		if( ( fbits & 0x7fffffff ) >= 0x47800000 )
		{
			if( val < 0x7f800000 ){
				return sign | 0x7c00;
			}
			return sign | 0x7c00 | ( fbits & 0x007fffff ) >> 13;
		}
		return sign | 0x7bff;
	}
	if( val >= 0x38800000 ){
		return sign | val - 0x38000000 >> 13;
	}
	if( val < 0x33000000 ){
		return sign;
	}
	val = ( fbits & 0x7fffffff ) >> 23;
	return sign | ( ( (fbits & 0x7fffff) | 0x800000 ) + ( 0x800000 >> val - 102 ) >> 126 - val );
}

float MathTools::HalfIntToFloat(int hbits){

	int32_t mant = hbits & 0x03ff;
	int32_t exp =  hbits & 0x7c00;
	    if( exp == 0x7c00 ){
	        exp = 0x3fc00;
	    }
	    else if( exp != 0 ){
	        exp += 0x1c000;
	        if( mant == 0 && exp > 0x1c400 ){
	        	union{
	        		float f;
	        		int32_t d;
	        	} x;
	        	x.d = ( hbits & 0x8000 ) << 16 | exp << 13 | 0x3ff;
	            return x.f;
	        }
	    }
	    else if( mant != 0 ){
	        exp = 0x1c400;
	        do {
	            mant <<= 1;
	            exp -= 0x400;
	        } while( ( mant & 0x400 ) == 0 );
	        mant &= 0x3ff;
	    }
	    union{
			float f;
			int32_t d;
		} x;
		x.d = ( hbits & 0x8000 ) << 16 | ( exp | mant ) << 13 ;
	    return x.f;
}

float MathTools::Sqrt(float x)
{
	float xhalf = 0.5f * x;

  union
  {
	float x;
    int i;
  } u;

  u.x = x;
  u.i = 0x5f3759df - (u.i >> 1);
  return x * u.x*(1.5f - xhalf * u.x * u.x);
}

float MathTools::CalcLength(float* x, int length)
{
	float sum2 = 0;
	for(int i = 0; i < length; i++){
		sum2 += x[i] * x[i];
	}
	return sqrt(sum2);
}

float MathTools::Sign(float value){

	return value / fabs(value);

}

float MathTools::DegreeToRadian(float degree){
	return degree * RADIAN_PER_DEGREE;
}

float MathTools::RadianToDegree(float radian){
	return radian * DEGREE_PER_RADIAN;
}

float MathTools::Trim(float lowerBound, float value, float upperBound){

	return (value > upperBound ? upperBound : value < lowerBound ? lowerBound : value);

}

float MathTools::CutOff(float value, float BoundValue, float Bound){

	return ((value < Bound && value > -Bound) ? BoundValue : value);

}

float MathTools::QuadRoot(float value){
	return sqrt(sqrt(value));
}

float MathTools::OctRoot(float value){
	return sqrt(sqrt(sqrt(value)));
}

float MathTools::TrimResolution(float value){
	union {
		uint32_t d;
		float f;
	} x;
	x.f = value;
	x.d &= 0xfffffff0;
	return x.f;
}

bool MathTools::CheckWithInInterval(float value, float mean, float var){
	if(fabs(value - mean) < var){
		return true;
	}
	else{
		return false;
	}
}
