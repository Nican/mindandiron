#pragma once

#include <msgpack_fwd.hpp>
namespace msgpack {

MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {

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








} // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack

#include <msgpack.hpp>