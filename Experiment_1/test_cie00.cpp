//#include <iostream>
//#include <pcl/common/point_tests.h> // for pcl::isFinite
//#include <pcl/features/pfh_tools.h>
//#include <pcl/common/colors.h>
//#include <set> // for std::set
//#include <pcl/features/feature.h>
//#include <array>  // for sRGB_LUT, sXYZ_LUT
//#include <float.h> // FLT_MAX
//
//#include <color-util/RGB_to_XYZ.hpp>
//#include <color-util/XYZ_to_Lab.hpp>
//#include <color-util/CIEDE2000.hpp>
//
//
//float to_rad(float degree)
//{
//	return degree * ((float)M_PI / 180.f);
//}
//
//float to_deg(float radians)
//{
//	return radians * (180.f / (float)M_PI);
//}
//
//double calculateDeltaCIE2000(const double* LAB1,const double* LAB2)
//{
//	// Adapted from Sharma et al's MATLAB implementation at
//	// http://www.ece.rochester.edu/~gsharma/ciede2000/
//	// http://www.ece.rochester.edu/~gsharma/ciede2000/dataNprograms/deltaE2000.m
//
//	//// Convert to regular LAB range
//	//const double L1 = LAB1[0] * 100.0;
//	//const double A1 = LAB1[1] * 256.0 - 128.0;
//	//const double B1 = LAB1[2] * 256.0 - 128.0;
//	//const double L2 = LAB2[0] * 100.0;
//	//const double A2 = LAB2[1] * 256.0 - 128.0;
//	//const double B2 = LAB2[2] * 256.0 - 128.0;
//	
//	// Convert to regular LAB range
//	const double L1 = LAB1[0];
//	const double A1 = LAB1[1];
//	const double B1 = LAB1[2];
//	const double L2 = LAB2[0];
//	const double A2 = LAB2[1];
//	const double B2 = LAB2[2];
//
//	// Parametric factors, use defaults
//	const double kl = 1;
//	const double kc = 1;
//	const double kh = 1;
//	const double pi = M_PI;
//
//	// Compute terms
//	const double chroma1 = std::sqrt(A1 * A1 + B1 * B1);
//	const double chroma2 = std::sqrt(A2 * A2 + B2 * B2);
//	const double chroma = (chroma1 + chroma2) * 0.5;
//	const double chroma7 = std::pow(chroma, 7);
//	const double g = (1.0 - std::sqrt(chroma7 / (chroma7 + std::pow(25.0, 7)))) * 0.5;
//	const double ap1 = (g + 1.0) * A1;
//	const double ap2 = (g + 1.0) * A2;
//	const double cp1 = std::sqrt(ap1 * ap1 + B1 * B1);
//	const double cp2 = std::sqrt(ap2 * ap2 + B2 * B2);
//	const double cpp = cp1 * cp2;
//
//	// Ensure hue is between 0 and 2pi
//	double hp1 = std::atan2(B1, ap1);
//	if (hp1 < 0) {
//		hp1 += pi * 2.0;
//	}
//	double hp2 = std::atan2(B2, ap2);
//	if (hp2 < 0) {
//		hp2 += pi * 2.0;
//	}
//
//	double dl = L2 - L1;
//	double dc = cp2 - cp1;
//	double dhp = hp2 - hp1;
//	if (dhp > +pi) {
//		dhp -= pi * 2.0;
//	}
//	if (dhp < -pi) {
//		dhp += pi * 2.0;
//	}
//	if (cpp == 0) {
//		dhp = 0;
//	}
//
//	// Note that the defining equations actually need
//	// signed Hue and chroma differences which is different
//	// from prior color difference formulae
//	double dh = std::sqrt(cpp) * std::sin(dhp / 2.0) * 2.0;
//
//	// Weighting functions
//	const double lp = (L1 + L2) * 0.5;
//	const double cp = (cp1 + cp2) * 0.5;
//	const double cp7 = std::pow(cp, 7);
//
//	// Average hue is computed in radians and converted to degrees where needed
//	double hp = (hp1 + hp2) * 0.5;
//
//	// Identify positions for which abs hue diff exceeds 180 degrees
//	if (std::abs(hp1 - hp2) > pi) {
//		hp -= pi;
//	}
//	if (hp < 0) {
//		hp += pi * 2.0;
//	}
//
//	// Check if one of the chroma values is zero, in which case set
//	// mean hue to the sum which is equivalent to other value
//	if (cpp == 0) {
//		hp = hp1 + hp2;
//	}
//
//	const double lpm502 = (lp - 50.0) * (lp - 50.0);
//	const double sl = 1.0 + 0.015 * lpm502 / std::sqrt(20.0 + lpm502);
//	const double sc = 1.0 + 0.045 * cp;
//	const double t = 1
//		- 0.17 * std::cos(hp - pi / 6.0)
//		+ 0.24 * std::cos(2.0 * hp)
//		+ 0.32 * std::cos(3.0 * hp + pi / 30.0)
//		- 0.20 * std::cos(4.0 * hp - 63.0 * pi / 180.0);
//	const double sh = 1.0 + 0.015 * cp * t;
//	const double ex = (180.0 / pi * hp - 275.0) / 25.0;
//	const double d = (30.0 * pi / 180.0) * std::exp(-1 * (ex * ex));
//	const double rc = 2.0 * std::sqrt(cp7 / (cp7 + std::pow(25.0, 7)));
//	const double rt = -1.0 * std::sin(2.0 * d) * rc;
//
//	dl = dl / (kl * sl);
//	dc = dc / (kc * sc);
//	dh = dh / (kh * sh);
//
//	// The CIE 2000 color difference
//	return std::sqrt(dl * dl + dc * dc + dh * dh + rt * dc * dh);
//}
//
/////////////////////////////////////////////////////////////////////////////////
////RGBtoCIELab
////////////////////////////////////////////////////////////////////////////////////////////////
//std::array<float, 256> sRGB_LUT = pcl::RGB2sRGB_LUT<float, 8>();
//
////////////////////////////////////////////////////////////////////////////////////////////////
//std::array<float, 4000> sXYZ_LUT = pcl::XYZ2LAB_LUT<float, 4000>();
//
//void RGB2CIELAB(unsigned char R, unsigned char G, unsigned char B, float& L, float& A, float& B2)
//{
//	float fr = sRGB_LUT[R];
//	float fg = sRGB_LUT[G];
//	float fb = sRGB_LUT[B];
//
//	// Use white = D65
//	const float x = fr * 0.412453f + fg * 0.357580f + fb * 0.180423f;
//	const float y = fr * 0.212671f + fg * 0.715160f + fb * 0.072169f;
//	const float z = fr * 0.019334f + fg * 0.119193f + fb * 0.950227f;
//
//	float vx = x / 0.95047f;
//	float vy = y;
//	float vz = z / 1.08883f;
//
//	vx = sXYZ_LUT[int(vx * 4000)];
//	vy = sXYZ_LUT[int(vy * 4000)];
//	vz = sXYZ_LUT[int(vz * 4000)];
//
//	L = 116.0f * vy - 16.0f;
//	if (L > 100)
//		L = 100.0f;
//
//	A = 500.0f * (vx - vy);
//	if (A > 120)
//		A = 120.0f;
//	else if (A < -120)
//		A = -120.0f;
//
//	B2 = 200.0f * (vy - vz);
//	if (B2 > 120)
//		B2 = 120.0f;
//	else if (B2 < -120)
//		B2 = -120.0f;
//}
//
//void RGBtoHSV(Eigen::Vector4i& in, Eigen::Vector4f& out)
//{
//	const unsigned char max = std::max(in[0], std::max(in[1], in[2]));
//	const unsigned char min = std::min(in[0], std::min(in[1], in[2]));
//
//	out[2] = static_cast <float> (max) / 255.f;//V ·¶Î§0-1
//
//	if (max == 0) // division by zero
//	{
//		out[1] = 0.f;
//		out[0] = 0.f; // h = -1.f;
//		return;
//	}
//
//	const float diff = static_cast <float> (max - min);
//	out[1] = diff / static_cast <float> (max);//S ·¶Î§0-1
//
//	if (min == max) // diff == 0 -> division by zero
//	{
//		out[0] = 0;
//		return;
//	}
//
//	if (max == in[0]) out[0] = 60.f * (static_cast <float> (in[1] - in[2]) / diff);
//	else if (max == in[1]) out[0] = 60.f * (2.f + static_cast <float> (in[2] - in[0]) / diff);
//	else                  out[0] = 60.f * (4.f + static_cast <float> (in[0] - in[1]) / diff); // max == b
//
//	if (out[0] < 0.f) out[0] += 360.f;
//}
//
//
//
//int main()
//{
//	//RGB×ªLAB
//	int red_1 = 200;
//	int green_1 = 100;
//	int blue_1 = 20;
//	int red_2 = 100;
//	int green_2 = 200;
//	int blue_2 = 50;
//	std::cout << "RGB_1: " << red_1 << "--" << green_1 << "--" << blue_1 << std::endl;
//	std::cout << "RGB_2: " << red_2 << "--" << green_2 << "--" << blue_2 << std::endl << std::endl;
//
//	float L1, a1, b1;
//	float L2, a2, b2;
//	RGB2CIELAB(red_1, green_1, blue_1, L1, a1, b1);
//	RGB2CIELAB(red_2, green_2, blue_2, L2, a2, b2);
//	std::cout << "PCL-LAB_1: " << L1/100 << "--" << a1/120 << "--" << b1/120 << std::endl;
//	std::cout << "PCL-LAB_2: " << L2/100 << "--" << a2/120 << "--" << b2/120 << std::endl << std::endl;
//
//	Eigen::Vector4i rr1(red_1, green_1, blue_1, 0);
//	Eigen::Vector4i rr2(red_2, green_2, blue_2, 0);
//	Eigen::Vector4f hsv1, hsv2;
//	RGBtoHSV(rr1, hsv1);
//	RGBtoHSV(rr2, hsv2);
//	std::cout << "MY_HSV_1: " << hsv1[0] << "--" << hsv1[1] << "--" << hsv1[2] << std::endl;
//	std::cout << "MY_HSV_2: " << hsv2[0] << "--" << hsv2[1] << "--" << hsv2[2] << std::endl << std::endl;
//
//	//CIELAB
//	colorutil::RGB rgb_color_1(red_1 / 255.0, green_1 / 255.0, blue_1 / 255.0);
//	colorutil::RGB rgb_color_2(red_2 / 255.0, green_2 / 255.0, blue_2 / 255.0);
//
//	colorutil::XYZ xyz_color_1 = colorutil::convert_RGB_to_XYZ(rgb_color_1);
//	colorutil::XYZ xyz_color_2 = colorutil::convert_RGB_to_XYZ(rgb_color_2);
//	colorutil::Lab lab_color_1 = colorutil::convert_XYZ_to_Lab(xyz_color_1);
//	colorutil::Lab lab_color_2 = colorutil::convert_XYZ_to_Lab(xyz_color_2);
//
//	double lab1[3] = { lab_color_1[0],lab_color_1[1],lab_color_1[2] };
//	double lab2[3] = { lab_color_2[0],lab_color_2[1],lab_color_2[2] };
//
//	std::cout << "LAB_1: " << lab1[0]/100 << "--" << lab1[1]/128 << "--" << lab1[2]/128 << std::endl;
//	std::cout << "LAB_2: " << lab2[0]/100 << "--" << lab2[1]/128 << "--" << lab2[2]/128 << std::endl << std::endl;
//
//	//to CIE00
//	float cie_00 = calculateDeltaCIE2000(lab1, lab2);
//	std::cout << "CIE00: " << cie_00 << std::endl;
//
//	double difference = colorutil::calculate_CIEDE2000(lab_color_1, lab_color_2);
//	std::cout << "yuki---CIE00: " << difference << std::endl;
//}