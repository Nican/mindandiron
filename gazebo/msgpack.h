#pragma once

#include <msgpack_fwd.hpp>
namespace msgpack {

MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {

/*
inline object const& operator>> (object const& o, Eigen::Vector3d& v) {
    if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
    if (o.via.array.size != 3) throw msgpack::type_error();

    o.via.array.ptr[0].convert(v.x());
    o.via.array.ptr[1].convert(v.y());
    o.via.array.ptr[2].convert(v.z());

    return o;
}

template <typename Stream>
inline packer<Stream>& operator<< (packer<Stream>& o, Eigen::Vector3d const& v) {
    // packing member variables as an array.
    o.pack_array(3);
    o.pack(v.x());
    o.pack(v.y());
    o.pack(v.z());
    return o;
}
*/

template <int _Rows>
inline object const& operator>> (object const& o, Eigen::Matrix<double, _Rows, 1>& v) {
    if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
    if (o.via.array.size != _Rows) throw msgpack::type_error();

    for(int i = 0; i < _Rows; i++)
        o.via.array.ptr[i].convert(v[i]);

    return o;
}

template <typename Stream, int _Rows>
inline packer<Stream>& operator<< (packer<Stream>& o, Eigen::Matrix<double, _Rows, 1> const& v) {
    // packing member variables as an array.
    o.pack_array(_Rows);
    
    for(int i = 0; i < _Rows; i++)
        o.pack(v[i]);

    return o;
}



} // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack

#include <msgpack.hpp>