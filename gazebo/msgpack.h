#pragma once
#include <complex>
#include <iostream>
#include <QDataStream>
#include <eigen3/Eigen/Dense>

typedef std::complex<double> Complex;

template<class T>
std::vector<T> GetRobotPoints()  
{
	 /*
	 return std::vector<T>({
			 {0.130-0.33, -0.447675},
			 {0.5366-0.33, -0.447675},
			 {1.25095-0.33, -0.1383},
			 {1.25095-0.33, 0.1383},
			 {0.5366-0.33, 0.447675},
			 {0.1302-0.33, 0.447675},
			 {0.0-0.33, 0.2286},
			 {0.0-0.33, -0.2286}
	 });
	 */

	 return std::vector<T>({
			 {0.57 + 0.36 , 0.605},
			 {0.57 + 0.36 , -0.605},
			 {-0.57 + 0.36, -0.605},
			 {-0.57 + 0.36, 0.605}       
	 });
};

namespace Robot
{

struct ImgData
{
	std::vector<unsigned char> data;
	unsigned int width;
	unsigned int height;
};


struct DepthImgData
{
	std::vector<float> data;
	unsigned int width;
	unsigned int height;
	float hfov;
};

}

QDataStream &operator<<(QDataStream &out, const Robot::ImgData &item);
QDataStream &operator>>(QDataStream &in, Robot::ImgData &item);

QDataStream &operator<<(QDataStream &out, const Robot::DepthImgData &item);
QDataStream &operator>>(QDataStream &in, Robot::DepthImgData &item);


template <typename T, int _Rows>
QDataStream &operator<<(QDataStream &out, const Eigen::Matrix<T, _Rows, 1> &item)
{
	for(int i = 0; i < _Rows; i++)
		out << item[i];

	return out;
}

template <typename T, int _Rows>
QDataStream &operator>>(QDataStream &in, Eigen::Matrix<T, _Rows, 1> &item)
{
	for(int i = 0; i < _Rows; i++)
		in >> item[i];

	return in;
}

/*
#include <msgpack_fwd.hpp>
namespace msgpack {

MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {

/////////////////////////
/// Eigen vectors
/////////////////////////
template <typename T, int _Rows>
inline object const& operator>> (object const& o, Eigen::Matrix<T, _Rows, 1>& v) {
		if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
		if (o.via.array.size != _Rows) throw msgpack::type_error();

		for(int i = 0; i < _Rows; i++)
				o.via.array.ptr[i].convert(v[i]);

		return o;
}

template <typename Stream, typename T, int _Rows>
inline packer<Stream>& operator<< (packer<Stream>& o, Eigen::Matrix<T, _Rows, 1> const& v) {
		// packing member variables as an array.
		o.pack_array(_Rows);
		
		for(int i = 0; i < _Rows; i++)
				o.pack(v[i]);

		return o;
}

/////////////////////////
/// Complex numbers
/////////////////////////
inline object const& operator>> (object const& o, std::complex<double>& v) {
		if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
		if (o.via.array.size != 2) throw msgpack::type_error();

		double real, imag;

		o.via.array.ptr[0].convert(real);
		o.via.array.ptr[1].convert(imag);

		v = std::complex<double>(real, imag);

		return o;
}

template <typename Stream>
inline packer<Stream>& operator<< (packer<Stream>& o, std::complex<double>const& v) {
		// packing member variables as an array.
		o.pack_array(2);
		
		
		o.pack_double(std::real(v));
		o.pack_double(std::imag(v));

		return o;
}


/////////////////////////
/// Image data
/////////////////////////
inline object const& operator>> (object const& o, Robot::ImgData& v) {
		if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
		if (o.via.array.size != 3) throw msgpack::type_error();

		o.via.array.ptr[0].convert(v.width);
		o.via.array.ptr[1].convert(v.height);

		v.data.resize(v.width * v.height * 3);
		std::memcpy(v.data.data(), o.via.array.ptr[2].via.bin.ptr, o.via.array.ptr[2].via.bin.size);

		return o;
}

template <typename Stream>
inline packer<Stream>& operator<< (packer<Stream>& o, Robot::ImgData const& v) {
		// packing member variables as an array.
		o.pack_array(3);
		
		o.pack(v.width);
		o.pack(v.height);

		o.pack_bin(v.width * v.height * 3);
		o.pack_bin_body(reinterpret_cast<const char*>(v.data.data()), v.width * v.height * 3);

		return o;
}

/////////////////////////
/// Depth Image data
/////////////////////////
inline object const& operator>> (object const& o, Robot::DepthImgData& v) {
		if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
		if (o.via.array.size != 4) throw msgpack::type_error();

		o.via.array.ptr[0].convert(v.width);
		o.via.array.ptr[1].convert(v.height);
		o.via.array.ptr[2].convert(v.hfov);

		v.data.resize(v.width * v.height);
		std::memcpy(v.data.data(), o.via.array.ptr[3].via.bin.ptr, o.via.array.ptr[3].via.bin.size);

		return o;
}

template <typename Stream>
inline packer<Stream>& operator<< (packer<Stream>& o, Robot::DepthImgData const& v) {
		// packing member variables as an array.
		o.pack_array(4);
		
		o.pack(v.width);
		o.pack(v.height);
		o.pack(v.hfov);

		o.pack_bin(v.width * v.height * sizeof(float));
		o.pack_bin_body(reinterpret_cast<const char*>(v.data.data()), v.width * v.height * sizeof(float));

		return o;
}





} // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack

#include <msgpack.hpp>
*/